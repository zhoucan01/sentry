/**
 ******************************************************************************
 * @file    Odometer.h
 * @brief   底盘里程计观测器：线性卡尔曼滤波 + RMS打滑检测
 *
 * 基于论文 Batch-LIWO 式3.84~3.96
 * 状态: x = [vx, vy, omega]^T  (底盘坐标系, m/s, rad/s)
 * 测量: z = [v0, v1, v2, v3]^T (4个轮速, m/s)
 ******************************************************************************
 */

#ifndef __ODOMETER_H
#define __ODOMETER_H

#include "system.h"
#include "math.h"

#define PI  3.14159265358979f

/* 轮子位置命名 (从车尾往前看)
 * [0] FR = 右前, quadrant I:   (+a, +b)
 * [1] FL = 左前, quadrant II:  (-a, +b)
 * [2] BL = 左后, quadrant III: (-a, -b)
 * [3] BR = 右后, quadrant IV:  (+a, -b)
 */

typedef struct {
    /* ---- 运动学参数 (论文式3.84) ---- */
    float a;                /* 底盘半长 X方向 (m) */
    float b;                /* 底盘半宽 Y方向 (m) */
    float wheel_perimeter;  /* 车轮周长 (mm) */
    float gear_ratio;       /* 电机减速比 */
    float enc_to_m;         /* 编码器计数→米 比例因子 */
    float dt;               /* 标称时间步长 (s) */

    /* ---- 卡尔曼滤波参数 ---- */
    float q_vx;             /* 过程噪声 Q = diag(q_vx, q_vy, q_wz)  论文式3.95 */
    float q_vy;
    float q_wz;
    float r_base;           /* 基础测量噪声 R₀ (m/s方差) */
    float r_slip_scale;     /* RMS缩放因子: σᵢ² = r_base + k*lossᵢ */

    /* ---- 内部状态 ---- */
    float x[3];             /* 滤波后状态 [vx, vy, omega] (m/s, m/s, rad/s) */
    float P[9];             /* 协方差矩阵 3×3 (行优先) */
    float H[12];            /* 测量矩阵 4×3 (论文式3.94) */
    float F[9];             /* 状态转移矩阵 3×3 (论文式3.93) */
    float Q[9];             /* 过程噪声 3×3 */
    float R[16];            /* 测量噪声 4×4 (对角) */

    /* ---- 输出 ---- */
    float raw_vel[3];       /* 最小二乘估计 [vx, vy, omega] (论文第28页) */
    float wheel_mps[4];     /* 四个轮速 (m/s) */
    float wheel_pred[4];    /* 预测轮速 (用于RMS) */
    float loss[4];          /* 残差 lossᵢ = (predᵢ - measᵢ)² */
    float rms;              /* RMS = Σ(lossᵢ) */
    float gimbal_yaw;       /* 云台相对底盘 yaw角 (rad) */

    /* ---- 调试 ---- */
    float   steer_angle[4]; /* 当前舵角 (rad) */
    int32_t init_ecd[4];    /* 舵角编码器初始偏移 */
    int32_t diff_enc[4];    /* 本帧编码器增量 */
    uint32_t init_complete; /* 初始化完成标志 */
} WheelObserver_t;

/* 全局实例 */
extern WheelObserver_t wheel_obs;

/* ===== 接口函数 ===== */

void WheelObserver_Init(void);
void WheelObserver_Update(void);
void Odometer_run(void const *argument);

/* 内联获取函数 */
static inline float* WheelObserver_GetVel(void)       { return wheel_obs.x; }
static inline float* WheelObserver_GetWheelMps(void)   { return wheel_obs.wheel_mps; }
static inline float  WheelObserver_GetGimbalYaw(void)  { return wheel_obs.gimbal_yaw; }
static inline float  WheelObserver_GetRMS(void)        { return wheel_obs.rms; }
static inline float* WheelObserver_GetLoss(void)       { return wheel_obs.loss; }

#endif /* __ODOMETER_H */
