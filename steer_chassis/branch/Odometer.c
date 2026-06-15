/**
 ******************************************************************************
 * @file    Odometer.c
 * @brief   底盘里程计观测器 — 基于论文 Batch-LIWO 式3.84~3.96
 *
 * 功能: 运动学正解、线性卡尔曼滤波、RMS打滑检测、云台相对角计算
 * 调用周期: 1ms (由 FreeRTOS 任务 Odometer_run 驱动)
 ******************************************************************************
 */

#include "Odometer.h"
#include "CAN_receive.h"
#include "bsp_dwt.h"
#include "bsp_transmit.h"
#include "cmsis_os.h"
#include "string.h"

/* ---------- 轮子位置→象限映射 ---------- */
/* steer_motor[i] → quadrant index for H matrix
 * [0]=FR→QI(+a,+b), [1]=FL→QII(-a,+b),
 * [2]=BL→QIII(-a,-b), [3]=BR→QIV(+a,-b)
 */
/* steer_motor[i] → quadrant: FL(0)→QII, BL(1)→QIII, BR(2)→QIV, FR(3)→QI */static const signed char kQuadrantMap[4] = {1, 2, 3, 0};  // <-- 按实车CAN ID重映射

/* 象限符号: 式3.84中的 ±a·s ± b·c
 * QI:   a·s - b·c   (+a, +b)
 * QII: -a·s - b·c   (-a, +b)
 * QIII:-a·s + b·c   (-a, -b)
 * QIV:  a·s + b·c   (+a, -b)
 */
static const float kQuadrantSignA[4] = { 1.0f, -1.0f, -1.0f,  1.0f};
static const float kQuadrantSignB[4] = {-1.0f, -1.0f,  1.0f,  1.0f};

/* 全局实例 */
WheelObserver_t wheel_obs;

/* ======================== 内部辅助函数 ======================== */

/* 归一化角度到 [-PI, PI] */
static inline float NormalizeAngle(float a) {
    while (a >  PI) a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
}

/* 3x3 矩阵乘 3x1 向量: y = A * x */
static void Mat3MulVec3(const float A[9], const float x[3], float y[3]) {
    y[0] = A[0]*x[0] + A[1]*x[1] + A[2]*x[2];
    y[1] = A[3]*x[0] + A[4]*x[1] + A[5]*x[2];
    y[2] = A[6]*x[0] + A[7]*x[1] + A[8]*x[2];
}

/* 3x3 矩阵相加: C = A + B */
static void Mat3Add(const float A[9], const float B[9], float C[9]) {
    for (int i = 0; i < 9; i++) C[i] = A[i] + B[i];
}

/* 3x3 矩阵乘 3x3: C = A * B */
static void Mat3MulMat3(const float A[9], const float B[9], float C[9]) {
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            C[r*3+c] = A[r*3+0]*B[0*3+c] +
                        A[r*3+1]*B[1*3+c] +
                        A[r*3+2]*B[2*3+c];
}

/* 3x3 转置: B = A^T */
static void Mat3Transpose(const float A[9], float B[9]) {
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            B[r*3+c] = A[c*3+r];
}

/* 3x3 LDL^T 分解求解 Ax = b (对角线LDU, A正定) */
static int Mat3Solve(const float A[9], const float b[3], float x[3]) {
    /* Cholesky: A = L * L^T */
    float L[9] = {0};
    for (int j = 0; j < 3; j++) {
        float s = 0;
        for (int k = 0; k < j; k++) s += L[j*3+k] * L[j*3+k];
        float d = A[j*3+j] - s;
        if (d <= 1e-12f) return -1;
        L[j*3+j] = sqrtf(d);
        for (int i = j+1; i < 3; i++) {
            s = 0;
            for (int k = 0; k < j; k++) s += L[i*3+k] * L[j*3+k];
            L[i*3+j] = (A[i*3+j] - s) / L[j*3+j];
        }
    }
    /* 前代: L * y = b */
    float y[3];
    for (int i = 0; i < 3; i++) {
        float s = 0;
        for (int j = 0; j < i; j++) s += L[i*3+j] * y[j];
        y[i] = (b[i] - s) / L[i*3+i];
    }
    /* 回代: L^T * x = y */
    for (int i = 2; i >= 0; i--) {
        float s = 0;
        for (int j = i+1; j < 3; j++) s += L[j*3+i] * x[j];
        x[i] = (y[i] - s) / L[i*3+i];
    }
    return 0;
}

/* 计算 4×3 H 矩阵 (论文式3.84, 3.94) 
 * H(i,0)=cos(θ_i), H(i,1)=sin(θ_i), 
 * H(i,2)=signA_i·a·sin(θ_i)+signB_i·b·cos(θ_i)
 */
static void BuildHMatrix(const float steer_rad[4], float a, float b, float H[12]) {
    for (int i = 0; i < 4; i++) {
        float c = cosf(steer_rad[i]);
        float s = sinf(steer_rad[i]);
        int qi = kQuadrantMap[i];
        H[i*3 + 0] = c;
        H[i*3 + 1] = s;
        H[i*3 + 2] = kQuadrantSignA[qi] * a * s + kQuadrantSignB[qi] * b * c;
    }
}

/* ======================== 初始化 ======================== */

void WheelObserver_Init(void) {
    memset(&wheel_obs, 0, sizeof(wheel_obs));

    /* --- 运动学参数 (从 system.h 的宏推导) --- */
    wheel_obs.a          = (float)distance_x;                     // 0.18 m
    wheel_obs.b          = (float)distance_y;                     // 0.18 m
    wheel_obs.wheel_perimeter = WHEEL_PERIMETER;                  // 376.99 mm
    wheel_obs.gear_ratio = (float)M3508_RATIO;                    // 19.2032
    /* 编码器每计数 = 轮周长[mm] / 减速比 / 编码器线数 / 1000 -> [m] */
    wheel_obs.enc_to_m   = wheel_obs.wheel_perimeter /
                           (wheel_obs.gear_ratio * 8192.0f) / 1000.0f;
    wheel_obs.dt         = 0.001f;   // 1ms (与 osDelay(1) 匹配)

    /* --- 卡尔曼参数 (论文式3.95, 3.96) --- */
    float dt = wheel_obs.dt;
    float a_max = 3.0f;     // 最大加速度 [m/s^2] (可调)
    float w_max = 6.0f;     // 最大角加速度 [rad/s^2]
    wheel_obs.q_vx  = (a_max * dt) * (a_max * dt);   // 式3.95
    wheel_obs.q_vy  = (a_max * dt) * (a_max * dt);
    wheel_obs.q_wz  = (w_max * dt) * (w_max * dt);

    wheel_obs.r_base      = 0.01f;    // 基础测量噪声 (速度[m/s]的方差)
    wheel_obs.r_slip_scale = 0.5f;    // RMS缩放因子 k

    /* --- 舵角校准偏移 --- */
    wheel_obs.init_ecd[0] = 3132;
    wheel_obs.init_ecd[1] = 5218;
    wheel_obs.init_ecd[2] = 5791;
    wheel_obs.init_ecd[3] = 7662;

    /* --- 初始状态 --- */
    wheel_obs.x[0] = 0; wheel_obs.x[1] = 0; wheel_obs.x[2] = 0;
    /* 初始协方差 (较大, 表示对初始状态不确定) */
    wheel_obs.P[0] = 1.0f; wheel_obs.P[1] = 0;    wheel_obs.P[2] = 0;
    wheel_obs.P[3] = 0;    wheel_obs.P[4] = 1.0f; wheel_obs.P[5] = 0;
    wheel_obs.P[6] = 0;    wheel_obs.P[7] = 0;    wheel_obs.P[8] = 1.0f;

    /* --- Q 矩阵 (论文式3.95) --- */
    memset(wheel_obs.Q, 0, sizeof(wheel_obs.Q));
    wheel_obs.Q[0] = wheel_obs.q_vx;
    wheel_obs.Q[4] = wheel_obs.q_vy;
    wheel_obs.Q[8] = wheel_obs.q_wz;

    /* --- F 矩阵 (论文式3.93): 匀速模型, dt很小所以用I --- */
    wheel_obs.F[0] = 1.0f; wheel_obs.F[1] = 0;    wheel_obs.F[2] = 0;
    wheel_obs.F[3] = 0;    wheel_obs.F[4] = 1.0f; wheel_obs.F[5] = 0;
    wheel_obs.F[6] = 0;    wheel_obs.F[7] = 0;    wheel_obs.F[8] = 1.0f;

    /* --- R 矩阵: 初始化为对角 --- */
    memset(wheel_obs.R, 0, sizeof(wheel_obs.R));

    wheel_obs.init_complete = 1;
}

/* ======================== 主更新函数 ======================== */

void WheelObserver_Update(void) {
    if (!wheel_obs.init_complete) return;
    /* 外部变量声明 (来自 CAN_receive 或外部模块) */

    /* ---- 第一步: 读取编码器增量并计算轮速 (m/s) ---- */
    for (int i = 0; i < 4; i++) {
        static int32_t last_ecd[4] = {0};
        /* 电机总编码 */
        int32_t current_ecd = chassis_motor[i].motor_measure.total_ecd;
        int32_t diff_enc = current_ecd - last_ecd[i];
        last_ecd[i] = current_ecd;

        /* 防止溢出 (电机正反转时total_ecd会重置) */
        if (diff_enc >  32767) diff_enc -= 65536;
        if (diff_enc < -32768) diff_enc += 65536;

        /* 舵角 (raw ecd → rad) */
        int32_t steer_raw = steer_motor[i].motor_measure.ecd;

        /* 储存在内部 */
        wheel_obs.diff_enc[i] = diff_enc;

        /* 用 AVERAGE dt (1ms) 计算轮速, 单位 m/s */
        wheel_obs.wheel_mps[i] = (float)diff_enc * wheel_obs.enc_to_m / wheel_obs.dt;

        /* 舵角: ecd差 → rad (8192 counts/rev, 公式: rad=ecd/8192*2PI) */
        float raw_rad = (float)(steer_raw - wheel_obs.init_ecd[i]) / 8192.0f * 2.0f * PI;
        wheel_obs.steer_angle[i] = NormalizeAngle(raw_rad);
    }

    /* ---- 第二步: 构建 H 矩阵 (论文式3.94) ---- */
    BuildHMatrix(wheel_obs.steer_angle, wheel_obs.a, wheel_obs.b, wheel_obs.H);

    /* ---- 第三步: 卡尔曼预测 (匀速模型) ---- */
    /* x_pred = F * x (F=I, 所以 x_pred = x) */ 
    /* 使用带旋转的F矩阵: 论文式3.93 */
    float w_last = wheel_obs.x[2];         // 上一拍的 omega
    float cw = cosf(w_last * wheel_obs.dt);
    float sw = sinf(w_last * wheel_obs.dt);
    float F[9] = {cw, -sw, 0,  sw, cw, 0,  0, 0, 1};
    float x_pred[3];
    Mat3MulVec3(F, wheel_obs.x, x_pred);

    /* P_pred = F * P * F^T + Q */
    float PFt[9], FPFt[9];
    float Ft[9];
    Mat3Transpose(F, Ft);
    Mat3MulMat3(F, wheel_obs.P, PFt);
    Mat3MulMat3(PFt, Ft, FPFt);
    Mat3Add(FPFt, wheel_obs.Q, wheel_obs.P);  // P ← P_pred

    /* ---- 第四步: 最小二乘估计 (论文第28页, 用于RMS计算) ---- */
    /* x_ls = (H^T * H)^{-1} * H^T * z */
    /* 4×3系统, 先计算 H^T * H (3×3) 和 H^T * z (3×1) */
    float HtH[9] = {0};
    float Htz[3] = {0};
    for (int i = 0; i < 4; i++) {
        const float* Hi = &wheel_obs.H[i * 3];
        float z_i = wheel_obs.wheel_mps[i];
        for (int r = 0; r < 3; r++) {
            Htz[r] += Hi[r] * z_i;
            for (int c = 0; c < 3; c++) {
                HtH[r * 3 + c] += Hi[r] * Hi[c];
            }
        }
    }
    /* 解 (H^T*H) * x_ls = H^T * z */
    float x_ls[3];
    if (Mat3Solve(HtH, Htz, x_ls) != 0) {
        return;  // 矩阵奇异, 跳过本帧
    }
    wheel_obs.raw_vel[0] = x_ls[0];
    wheel_obs.raw_vel[1] = x_ls[1];
    wheel_obs.raw_vel[2] = x_ls[2];

    /* ---- 第五步: 计算打滑残差 RMS (论文第28页) ---- */
    /* w_pred = H * x_ls, loss_i = (w_pred_i - z_i)^2 */
    wheel_obs.rms = 0;
    for (int i = 0; i < 4; i++) {
        const float* Hi = &wheel_obs.H[i * 3];
        float pred = Hi[0] * x_ls[0] + Hi[1] * x_ls[1] + Hi[2] * x_ls[2];
        wheel_obs.wheel_pred[i] = pred;
        float err = pred - wheel_obs.wheel_mps[i];
        wheel_obs.loss[i] = err * err;
        wheel_obs.rms += wheel_obs.loss[i];
    }

    /* ---- 第六步: 自适应测量噪声 R (论文式3.96) ---- */
    /* sigma_i^2 = r_base + k * loss_i */
    memset(wheel_obs.R, 0, sizeof(wheel_obs.R));
    for (int i = 0; i < 4; i++) {
        wheel_obs.R[i * 4 + i] = wheel_obs.r_base +
            wheel_obs.r_slip_scale * wheel_obs.loss[i];
    }

    /* ---- 第七步: 卡尔曼更新 ---- */
    /* S = H * P * H^T + R (4×4) */
    /* K = P * H^T * S^{-1} (3×4) */
    /* x = x + K * (z - H * x) */
    /* P = (I - K * H) * P */

    /* 计算残差: y = z - H * x_pred (4×1) */
    float y[4];
    for (int i = 0; i < 4; i++) {
        const float* Hi = &wheel_obs.H[i * 3];
        y[i] = wheel_obs.wheel_mps[i] -
              (Hi[0] * x_pred[0] + Hi[1] * x_pred[1] + Hi[2] * x_pred[2]);
    }

    /* S = H * P * H^T + R (4×4) 
     * 为避免大矩阵操作, 直接计算 K * y:
     * 等效于求解 (H*P*H^T + R) * K_y = H * y  其中 K_y = K^T * y (4×1)
     * 或者直接算 3 维增益:
     * K = P * H^T * S^{-1} → (K*y) = P * H^T * S^{-1} * y
     * 先算 HTy = H^T * y (3×1)
     * 再算 (H*P*H^T + R) * alpha = y, 其中 alpha = ? 
     * 
     * 标准公式: 
     *   K = P * H^T * S^{-1}
     *   dx = K * y = P * H^T * (S^{-1} * y)
     * 令 S = H*P*H^T + R, 求解 S * beta = y (4×4系统)
     * 则 dx = P * H^T * beta
     */

    /* 构建 S = H * P * H^T + R (4×4) */
    float S[16] = {0};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            const float* Hi = &wheel_obs.H[i * 3];
            const float* Hj = &wheel_obs.H[j * 3];
            /* (H * P * H^T)[i][j] = Hi * P * Hj^T */
            float sum = 0;
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++)
                    sum += Hi[r] * wheel_obs.P[r*3+c] * Hj[c];
            S[i * 4 + j] = sum;
        }
        S[i * 4 + i] += wheel_obs.R[i * 4 + i];  // 加 R
    }

    /* 求解 S * beta = y (4×4, 用高斯消元) */
    float beta[4];
    {
        /* 增广矩阵 [S | y] */
        float aug[4][5];
        for (int r = 0; r < 4; r++) {
            for (int c = 0; c < 4; c++) aug[r][c] = S[r*4+c];
            aug[r][4] = y[r];
        }
        /* 高斯消元 (列主元) */
        for (int col = 0; col < 4; col++) {
            /* 选主元 */
            int mr = col;
            float mv = fabsf(aug[col][col]);
            for (int r = col+1; r < 4; r++) {
                if (fabsf(aug[r][col]) > mv) {
                    mv = fabsf(aug[r][col]); mr = r;
                }
            }
            if (mv < 1e-15f) { memset(beta, 0, sizeof(beta)); goto skip_update; }
            /* 交换行 */
            if (mr != col) {
                for (int c = col; c <= 4; c++) {
                    float t = aug[col][c]; aug[col][c] = aug[mr][c]; aug[mr][c] = t;
                }
            }
            /* 消去 */
            float piv = aug[col][col];
            for (int r = col+1; r < 4; r++) {
                float f = aug[r][col] / piv;
                for (int c = col; c <= 4; c++)
                    aug[r][c] -= f * aug[col][c];
            }
        }
        /* 回代 */
        for (int i = 3; i >= 0; i--) {
            float s = aug[i][4];
            for (int j = i+1; j < 4; j++) s -= aug[i][j] * beta[j];
            beta[i] = s / aug[i][i];
        }
    }

    /* dx = P * H^T * beta = P * (H^T * beta) */
    {
        float HTbeta[3] = {0};
        for (int i = 0; i < 4; i++) {
            const float* Hi = &wheel_obs.H[i * 3];
            for (int r = 0; r < 3; r++)
                HTbeta[r] += Hi[r] * beta[i];
        }
        float dx[3];
        Mat3MulVec3(wheel_obs.P, HTbeta, dx);
        wheel_obs.x[0] = x_pred[0] + dx[0];
        wheel_obs.x[1] = x_pred[1] + dx[1];
        wheel_obs.x[2] = x_pred[2] + dx[2];
    }

skip_update:
    /* P更新: P = (I - K*H) * P, 等效于 P = P - P*H^T*S^{-1}*H*P */
    /* 直接用 Joseph 形式: P = (I - K*H) * P */
    /* P 更新 (标准卡尔曼): P = (I - K*H) * P
     * 先用 P_update = P - P*H^T * S^{-1} * H * P
     * 其中 S = H*P*H^T + R 已在上文构建
     * 计算: KHP = P * H^T * S^{-1} * H * P (3×3)
     * 效率: 复用已有 beta 求解器
     * 简化方案: 对每列 i, 求解 S * col_i = e_i (4×1)
     *   → K_col_i = P * H^T * col_i (3×1)
     *   → (K*H)[r][c] += K_col_i[r] * H[i][c]
     */
    {
        float KH[9] = {0};  // K*H (3×3)
        for (int col = 0; col < 4; col++) {
            /* 求解 S * s = e_col */
            float e[4] = {0,0,0,0}; e[col] = 1.0f;
            float s[4];
            /* 高斯消元 (复用之前的消元算法) */
            float aug2[4][5];
            for (int r = 0; r < 4; r++) {
                for (int c = 0; c < 4; c++) aug2[r][c] = S[r*4+c];
                aug2[r][4] = e[r];
            }
            for (int c = 0; c < 4; c++) {
                int mr = c; float mv = fabsf(aug2[c][c]);
                for (int r = c+1; r < 4; r++)
                    if (fabsf(aug2[r][c]) > mv) { mv = fabsf(aug2[r][c]); mr = r; }
                if (mv < 1e-15f) continue;
                if (mr != c) for (int cc = c; cc <= 4; cc++) {
                    float t = aug2[c][cc]; aug2[c][cc] = aug2[mr][cc]; aug2[mr][cc] = t;
                }
                float piv = aug2[c][c];
                for (int r = c+1; r < 4; r++) {
                    float f = aug2[r][c] / piv;
                    for (int cc = c; cc <= 4; cc++) aug2[r][cc] -= f * aug2[c][cc];
                }
            }
            for (int i = 3; i >= 0; i--) {
                float sum = aug2[i][4];
                for (int j = i+1; j < 4; j++) sum -= aug2[i][j] * s[j];
                s[i] = sum / aug2[i][i];
            }
            /* K_col = P * H^T * s (3×1) */
            float HTs[3] = {0};
            for (int i = 0; i < 4; i++)
                for (int r = 0; r < 3; r++)
                    HTs[r] += wheel_obs.H[i*3+r] * s[i];
            float Kcol[3];
            Mat3MulVec3(wheel_obs.P, HTs, Kcol);
            /* KH[r][c] += Kcol[r] * H[col][c] */
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++)
                    KH[r*3+c] += Kcol[r] * wheel_obs.H[col*3+c];
        }
        /* P = (I - KH) * P */
        float I_minus_KH[9];
        for (int i = 0; i < 9; i++) I_minus_KH[i] = (i % 4 == 0 ? 1.0f : 0.0f) - KH[i];
        float newP[9];
        Mat3MulMat3(I_minus_KH, wheel_obs.P, newP);
        memcpy(wheel_obs.P, newP, sizeof(newP));
    }

    /* ---- 第八步: 计算云台相对角 (gimbal_yaw) ---- */
    /* gimbal_yaw = big_yaw_ecd - yaw_ecd_diff, 归一化到 [-PI, PI] */
    {
        float yaw_diff_rad = USART_Rx_data.yaw_diff * 2.0f * PI / 360.0f;
        float raw_diff = USART_Rx_data.big_yaw_ecd - yaw_diff_rad;
        float gimbal_yaw_calc = forword_ecd - raw_diff;
        float yaw_rad = gimbal_yaw_calc;
        wheel_obs.gimbal_yaw = NormalizeAngle(yaw_rad);
    }
}

/* Odometer_run — FreeRTOS任务: 底盘里程计观测器主循环 */
void Odometer_run(void const * argument)
{
    WheelObserver_Init();
    for(;;)
    {
        WheelObserver_Update();
        osDelay(1);
    }
}