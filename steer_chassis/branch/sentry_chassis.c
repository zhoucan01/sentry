/**
 ******************************************************************************
 * @file    sentry_chassis.c
 * @brief   哨兵底盘运动控制 —— 全向舵轮底盘核心算法
 *
 * 控制流水线 (每 1ms 执行):
 *   [1] 坐标系旋转   — 云台速度 → 底盘坐标系 (speed_in_reslove_1)
 *   [2] 逆运动学分解 — 底盘 vx/vy/ω → 四轮 vx/vy (chassis_speed_get)
 *   [3] 舵角解算     — vx/vy → atan2 → 编码器值 + 劣弧优化 (steer_angle_get)
 *   [4] 速度合成修正 — 方向修正 + cos衰减 (chassis_speed_set)
 *   [5] PID 计算     — 舵角PID + 舵速PID + 轮速PID (chassis_clac)
 *   [6] 功率限制     — 超电自适应功率分配 (chassis_power_limit_set)
 *
 * 舵轮编号 (CAN_receive.h 定义):
 *   FR(3)=右前  BR(2)=右后  BL(1)=左后  FL(0)=左前
 ******************************************************************************
 */

#include "sentry_chassis.h"
#include "system.h"
#include "CAN_receive.h"
#include "cmsis_os.h"
#include "bsp_pid.h"
#include "bsp_transmit.h"
#include "Odometer.h"
#include "arm_math.h"
#include "can.h"
#include "motor.h"
#include "SuperCAP.h"

/* ======================== 全局 PID 控制器 ======================== */
pid_struct_t pid_steer_ecd[4];      /* 舵角位置环 */
pid_struct_t pid_steer_speed[4];    /* 舵角速度环 */
pid_struct_t pid_chassis_speed[4];  /* 轮速环 */
pid_struct_t pid_chaais_Sx;         /* 里程计 X 位置环 */
pid_struct_t pid_chaais_Sy;         /* 里程计 Y 位置环 */
pid_struct_t chassis_buffer;        /* 功率缓冲 PID */
pid_struct_t cap_buff;              /* 超电缓冲 PID */

/* 条件编译 */
#define steer
#define chassis_open

/* 调试输出 */
float get_speed_vx, get_speed_vy;
float get_power, remain_power;
float real_chassis = 0.0f;

/* 功率限制内部状态 */
static float input_power = 0.0f;

/* 移动检测状态 */
static int real_move_state = 0;

/* ======================== 内部辅助函数声明 ======================== */

/* 将速度从云台坐标系旋转到底盘坐标系 */
static void RotateSpeedToBase(const speed_t *in, float diff_angle, speed_t *out);

/* 计算单个电机的功率 P = kp·I·ω + kw·ω? + ki·I? + const */
static float CalcMotorPower(float kp, float kw, float ki, float constant,
                            float speed_rpm, float set_current);

/* 功率限制核心: 将一组电机功率缩放到目标上限, 返回限制后的实际总功率 */
static float ApplyPowerLimit(float kp, float kw, float ki, float constant,
                              motor_data_t *motors, float power_budget, int count);
/* 舵角劣弧优化: 在8192编码器范围内选择最短路径 */
static void ResolveSteerShortArc(float *ecd_target, const float *ecd_current,
                                 float *speed_direct);

/* ======================== 内部辅助函数实现 ======================== */

/**
 * @brief  坐标系旋转: 云台系 → 底盘系
 * @param  in          云台坐标系速度 {vx, vy, wz}
 * @param  diff_angle  云台-底盘夹角 (rad)
 * @param  out         输出底盘坐标系速度
 */
static void RotateSpeedToBase(const speed_t *in, float diff_angle, speed_t *out)
{
    float c = cosf(diff_angle);
    float s = sinf(diff_angle);
    out->vx =  in->vx * c + in->vy * s;
    out->vy = -in->vx * s + in->vy * c;
    out->wz =  in->wz;
}

/**
 * @brief  电机功率模型: P = kp·I·ω + kw·ω? + ki·I? + constant
 */
static float CalcMotorPower(float kp, float kw, float ki, float constant,
                            float speed_rpm, float set_current)
{
    return kp * speed_rpm * set_current
         + kw * speed_rpm * speed_rpm
         + ki * set_current * set_current
         + constant;
}

/**
 * @brief  功率限制: 等比例缩放电流使总功率不超过 budget
 *
 * @note   从 P = kp·I·ω + kw·ω? + ki·I? + const 反解 I:
 *         ki·I? + kp·ω·I + (kw·ω? + const - P_target) = 0
 *         I = (-kp·ω ± sqrt((kp·ω)? - 4·ki·(kw·ω?+const-P_target))) / (2·ki)
 * @return  未超限时返回原始总功率, 超限时返回 power_budget
 */
static float ApplyPowerLimit(float kp, float kw, float ki, float constant,
                              motor_data_t *motors, float power_budget, int count)
{
    /* 计算当前总功率 */
    float total_power = 0.0f;
    float power[4];
    for (int i = 0; i < count; i++) {
        power[i] = CalcMotorPower(kp, kw, ki, constant,
                                  motors[i].motor_measure.speed_rpm,
                                  motors[i].motor_tar.set_current);
        if (power[i] > 0.0f) total_power += power[i];
    }

    if (total_power <= power_budget) return total_power;

    /* 超限: 等比例缩放电流至 budget, 直接返回 budget (误差可忽略) */
    float scale = power_budget / total_power;
    for (int i = 0; i < count; i++) {
        float target_power = power[i] * scale;
        if (target_power <= 0.0f) continue;

        float w   = motors[i].motor_measure.speed_rpm;
        float b   = kp * w;
        float c   = kw * w * w + constant - target_power;
        float disc = b * b - 4.0f * ki * c;

        if (disc < 0.0f) continue;

        float sqrt_disc = sqrtf(disc);
        float I;
        if (motors[i].motor_tar.set_current > 0) {
            I = (-b + sqrt_disc) / (2.0f * ki);
            if (I > CURRENT_LIMIT) I = CURRENT_LIMIT;
        } else {
            I = (-b - sqrt_disc) / (2.0f * ki);
            if (I < -CURRENT_LIMIT) I = -CURRENT_LIMIT;
        }
        motors[i].motor_tar.set_current = I;
    }
    return power_budget;
}

/**
 * @brief  舵角劣弧优化: 在8192范围内选择编码器最短路径
 *
 * 编码器为 0~8191 循环，需要选择距离当前位置最近的目标值。
 * 如果差距 > 2048 (即 > 90°)，则反向转 180° 并通过 speed_direct 反转轮速。
 */
static void ResolveSteerShortArc(float *ecd_target, const float *ecd_current,
                                 float *speed_direct)
{
    for (int i = 0; i < 4; i++) {
        float target = ecd_target[i];
        float current = ecd_current[i];

        /* 归一化到 [-4096, 4096] */
        while (target - current >  4096.0f) target -= 8192.0f;
        while (target - current < -4096.0f) target += 8192.0f;

        /* 劣弧选择: >90°则反向转 180° */
        if (target - current > 2048.0f) {
            speed_direct[i] = -1.0f;
            target -= 4096.0f;
        } else if (target - current < -2048.0f) {
            speed_direct[i] = -1.0f;
            target += 4096.0f;
        } else {
            speed_direct[i] = 1.0f;
        }

        ecd_target[i] = target;
    }
}

/* ======================== 主任务 ======================== */

/**
 * @brief  FreeRTOS 底盘控制主任务 (优先级: AboveNormal, 栈: 512)
 *
 * 每 2ms 执行一次完整控制循环 (两个 osDelay(1) 中间插入超电CAN发送)
 */
void sentry_chassis_run(void const *argument)
{
    (void)argument;
    chassis_pid_init();

    for (;;) {
        /* [1] 坐标系旋转 */
        speed_in_reslove_1(&chassis);

        /* [2] 逆运动学分解 */
        chassis_speed_get(&chassis);

        /* [3] 舵角解算 + 劣弧优化 */
        steer_angle_get(&chassis);

        /* [4] 速度合成修正 */
        chassis_speed_set(&chassis);

        /* [5] PID 计算 */
        chassis_clac(&chassis);

        /* [6] 功率限制 */
        chassis_power_limit_set();

        /* CAN 发送 (条件编译控制正常/错误帧) */
#ifdef steer
        steer_normol_send();
#else
        steer_error_send();
#endif

#ifdef chassis_open
        chassis_normol_send();
#else
        chassis_error_send();
#endif

        osDelay(1);
        Send_SupPower(&hcan2);
        osDelay(1);
    }
}

/* ======================== PID 初始化 ======================== */

/**
 * @brief  初始化所有 PID 控制器参数
 *
 * 舵角PID:   P=0.5  (位置环, 输出限幅 400~500)
 * 舵速PID:   P=50~55 (速度环, 输出限幅 16384)
 * 轮速PID:   P=5.0, I=0.1 (输出限幅 2500~4000, 积分限幅 16000)
 */
void chassis_pid_init(void)
{
    /* ---- 轮速PID (4轮) ---- */
    for (int i = 0; i < 4; i++) {
        pid_init(&pid_chassis_speed[i], 5.0f, 0.1f, 0.0f, 2500.0f, 16000.0f);
    }
    /* BR/BL 轮摩擦力更大, 调高积分限幅 */
    pid_init(&pid_chassis_speed[BR], 5.0f, 0.1f, 0.0f, 4000.0f, 16000.0f);
    pid_init(&pid_chassis_speed[BL], 5.0f, 0.1f, 0.0f, 2500.0f, 16000.0f);

    /* ---- 舵角位置PID (4舵) ---- */
    pid_init(&pid_steer_ecd[FR], 0.5f, 0.0f, 0.0f, 0.0f, 400.0f);
    pid_init(&pid_steer_ecd[BR], 0.5f, 0.0f, 0.0f, 0.0f, 500.0f);
    pid_init(&pid_steer_ecd[BL], 0.5f, 0.0f, 0.0f, 0.0f, 400.0f);
    pid_init(&pid_steer_ecd[FL], 0.5f, 0.0f, 0.0f, 0.0f, 400.0f);

    /* ---- 舵速PID (4舵) ---- */
    pid_init(&pid_steer_speed[FR], 55.0f, 0.0f, 0.0f, 0.0f, 16384.0f);
    pid_init(&pid_steer_speed[BR], 50.0f, 0.0f, 0.0f, 0.0f, 16384.0f);
    pid_init(&pid_steer_speed[BL], 50.0f, 0.0f, 0.0f, 0.0f, 16384.0f);
    pid_init(&pid_steer_speed[FL], 50.0f, 0.0f, 0.0f, 0.0f, 16384.0f);

    /* ---- 辅助PID ---- */
    pid_init(&chassis_buffer,  0.5f,  0.0f,   0.0f, 0.0f,   30.0f);
    pid_init(&pid_chaais_Sx,  10.0f,  0.0f,   0.0f, 0.0f, 1500.0f);
    pid_init(&pid_chaais_Sy,  10.0f,  0.0f,   0.0f, 0.0f, 1500.0f);
    pid_init(&cap_buff,        0.5f,  0.001f, 0.0f, 5.0f, 16000.0f);
}

/* ======================== 速度解算 ======================== */

/**
 * @brief  坐标系旋转 (串口通信模式)
 *
 * 将云台坐标系下的目标速度旋转到底盘坐标系。
 * 当三轴速度全为0时, 临时设 vx=1 以保持舵角朝正前方。
 */
static void DoSpeedResolve(chassis_t *mode)
{
    speed_t temp_in = mode->speed_in;

    /* 速度全零时: 保持舵角朝正前方 */
    int is_zero = (fabsf(temp_in.vx) < 1e-6f)
               && (fabsf(temp_in.vy) < 1e-6f)
               && (fabsf(temp_in.wz) < 1e-6f);
    if (is_zero) temp_in.vx = 1.0f;

    RotateSpeedToBase(&temp_in, mode->diff_angle, &mode->speed_reslove);

    if (is_zero) mode->speed_in.vx = 0.0f;
}

void speed_in_reslove_1(chassis_t *mode)
{
    move_state_change(mode);

    if (mode->move_flag == 0) {
        mode->speed_in.vx = 0.0f;
        mode->speed_in.vy = 0.0f;
        mode->speed_in.wz = 0.0f;
    }

    switch (mode->chassis_mode) {
    case no_move:
        DoSpeedResolve(mode);
        break;
    case normol_move:
        DoSpeedResolve(mode);
        break;
    default:
        DoSpeedResolve(mode);
        break;
    }

    get_speed_vx = mode->speed_reslove.vx;
    get_speed_vy = mode->speed_reslove.vy;
}

/* ======================== 速度限幅 ======================== */

/**
 * @brief  速度增量限幅 (防阶跃)
 */
float limit_addspeed(float speed_set, float speed_ref, float addspeed_limit)
{
    float diff = speed_set - speed_ref;
    if (fabsf(diff) > addspeed_limit) {
        speed_set = (diff > 0.0f)
            ? speed_ref + addspeed_limit
            : speed_ref - addspeed_limit;
    }
    return speed_set;
}

/* ======================== 逆运动学分解 ======================== */

/**
 * @brief  底盘三轴速度 → 四轮速度分量 (逆运动学)
 *
 * 几何关系 (舵轮位于正方形四角, 半径 R = distance_x = distance_y):
 *   轮i线速度 = v_底盘 + ω × r_i
 *
 * 四个轮子分别位于:
 *   FR(+a,+b)  BR(+a,-b)  BL(-a,-b)  FL(-a,+b)
 *
 * @note  sin(π/4) = cos(π/4) ≈ 0.7071, 因正方形布局用 π/4
 */
void chassis_speed_get(chassis_t *mode)
{
    const float SIN45 = 0.70710678f;  /* sin(π/4) */
    float wz = mode->speed_reslove.wz * SIN45;

    /* FR: 右前 (+a,+b) */
    mode->chassis[FR].vx = mode->speed_reslove.vx - wz;
    mode->chassis[FR].vy = mode->speed_reslove.vy + wz;

    /* BR: 右后 (+a,-b) */
    mode->chassis[BR].vx = mode->speed_reslove.vx - wz;
    mode->chassis[BR].vy = mode->speed_reslove.vy - wz;

    /* BL: 左后 (-a,-b) */
    mode->chassis[BL].vx = mode->speed_reslove.vx + wz;
    mode->chassis[BL].vy = mode->speed_reslove.vy - wz;

    /* FL: 左前 (-a,+b) */
    mode->chassis[FL].vx = mode->speed_reslove.vx + wz;
    mode->chassis[FL].vy = mode->speed_reslove.vy + wz;
    /* speed_set 由 chassis_speed_set 统一计算, 此处不重复 */
}

/* ======================== 舵角解算 ======================== */

/**
 * @brief  舵角解算 + 劣弧优化 + 方向修正
 *
 * 流程:
 *   1. atan2(vy, vx) → 舵角 (rad)
 *   2. rad → 编码器值 (0~8191)
 *   3. 劣弧优化: 选择最短旋转路径, >90°则反向180°+轮速反转
 *   4. no_move 模式下锁定舵角
 *   5. move_flag=0 时保持上次舵角
 */
void steer_angle_get(chassis_t *mode)
{
    /* ---- 第一步: 四轮舵角计算 ---- */
    for (int i = 0; i < 4; i++) {
        float angle = atan2f(mode->chassis[i].vy, mode->chassis[i].vx);
        if (angle < 0.0f) angle += 2.0f * PI;
        mode->steer_angle[i] = angle;
    }

    /* ---- 第二步: 角度 → 编码器值 ---- */
    static const float steer_init[4] = {
        steer_init_angle_FR, steer_init_angle_BR,
        steer_init_angle_BL, steer_init_angle_FL
    };
    for (int i = 0; i < 4; i++) {
        mode->steer_resolve_ecd[i] = mode->steer_angle[i]
            / (2.0f * PI) * 8192.0f + steer_init[i];
    }

    /* ---- 第三步: 劣弧优化 + 方向修正 ---- */
    float ecd_now[4];
    for (int i = 0; i < 4; i++) {
        ecd_now[i] = (float)steer_motor[i].motor_measure.ecd;
    }
    ResolveSteerShortArc(mode->steer_resolve_ecd, ecd_now,
                         mode->speed_direct);

    /* 将结果写入 steer_ecd */
    for (int i = 0; i < 4; i++) {
        mode->steer_ecd[i] = mode->steer_resolve_ecd[i];
    }

    /* ---- 第四步: no_move 模式下记录当前编码器用于锁定 ---- */
    if (mode->chassis_mode == no_move) {
        for (int i = 0; i < 4; i++) {
            mode->last_ecd[i] = (float)steer_motor[i].motor_measure.ecd;
        }
    }

    /* ---- 第五步: 最终舵角选择 ---- */
    if (mode->move_flag == 0) {
        /* 停止时锁定舵角 */
        for (int i = 0; i < 4; i++) {
            mode->steer_final_ecd[i] = mode->last_ecd[i];
        }
    } else {
        for (int i = 0; i < 4; i++) {
            mode->steer_final_ecd[i] = mode->steer_ecd[i];
        }
    }

    /* 更新 last_ecd */
    for (int i = 0; i < 4; i++) {
        mode->last_ecd[i] = mode->steer_final_ecd[i];
    }
}

/* ======================== 速度合成修正 ======================== */

/**
 * @brief  速度合成 + 方向修正 + cos?衰减
 *
 * 当不在陀螺状态时, 检测舵角误差并用 cos? 衰减轮速,
 * 防止舵向未到位时轮子猛转。
 */
void chassis_speed_set(chassis_t *mode)
{
    /* 合成四轮线速度 */
    for (int i = 0; i < 4; i++) {
        mode->speed_set[i] = sqrtf(mode->chassis[i].vx * mode->chassis[i].vx
                                 + mode->chassis[i].vy * mode->chassis[i].vy);
    }

    /* 零速清零 */
    if (mode->chassis_mode == no_move
        || (fabsf(mode->speed_in.vx) < 1e-6f
         && fabsf(mode->speed_in.vy) < 1e-6f
         && fabsf(mode->speed_in.wz) < 1e-6f)) {
        for (int i = 0; i < 4; i++) mode->speed_set[i] = 0.0f;
    }

    /* 方向修正 */
    for (int i = 0; i < 4; i++) {
        mode->speed_set[i] *= (float)mode->speed_direct[i];
    }

    /* cos? 衰减 (非陀螺时) */
    if (fabsf(mode->speed_in.wz) < 1e-6f) {
        for (int i = 0; i < 4; i++) {
            float angle_err = fabsf(mode->steer_ecd[i]
                - (float)steer_motor[i].motor_measure.ecd)
                * 2.0f * PI / 8192.0f;
            float k = arm_cos_f32(angle_err);
            mode->speed_set[i] *= k * k * k;
        }
    }
}

/* ======================== PID 计算输出 ======================== */

/**
 * @brief  级联 PID: 舵角位置环 → 舵速环 → 轮速环
 */
void chassis_clac(chassis_t *mode)
{
    for (int i = 0; i < 4; i++) {
        /* 舵角位置环 */
        mode->steer_speed_set[i] = pid_calc(&pid_steer_ecd[i],
            steer_motor[i].motor_measure.ecd, mode->steer_final_ecd[i]);

        /* 舵速环 */
        steer_motor[i].motor_tar.set_current = pid_calc(&pid_steer_speed[i],
            steer_motor[i].motor_measure.speed_rpm, mode->steer_speed_set[i]);

        /* 轮速环 */
        chassis_motor[i].motor_tar.set_current = pid_calc(&pid_chassis_speed[i],
            chassis_motor[i].motor_measure.speed_rpm, mode->speed_set[i]);
    }
}

/* ======================== 功率限制 ======================== */

/**
 * @brief  底盘功率限制 (超电自适应)
 *
 * 功率分配策略:
 *   1. 裁判系统限制基础功率 + 超电额外功率
 *   2. 舵机优先分配 (上限 = 总功率 × 80%)
 *   3. 剩余功率分配给底盘电机
 *   4. 超限时通过功率模型反解电流进行等比例缩放
 */
void chassis_power_limit_set(void)
{
    uint16_t max_power_limit = 100;  /* 哨兵裁判系统功率上限 */
    float chassis_max_power;
    float chassis_remain_power;
    float chassis_power_buffer;
    float steer_power_budget;

    /* ---- 1. 读取缓冲能量, 计算可用功率 ---- */
    chassis_power_buffer = USART_Rx_data.chassis_buff;
    input_power = (float)max_power_limit
        - pid_calc(&chassis_buffer, chassis_power_buffer, 30.0f);

    /* ---- 2. 超电检测 ---- */
    if (SuperCAP.C_Vol >= 13.5f) {
        chassis_max_power = (USART_Rx_data.chassis_Power_limit < 40)
            ? input_power + 20.0f
            : input_power + 60.0f;
    } else {
        chassis_max_power = input_power;
    }

    /* ---- 3. 舵机功率限制 (上限80%) ---- */
    steer_power_budget = chassis_max_power * 0.8f;
    float actual_steer_power = ApplyPowerLimit(motor_6020_kp, motor_6020_kw,
                                    motor_6020_ki, motor_6020_constant,
                                    steer_motor, steer_power_budget, 4);
    get_power = actual_steer_power;

    /* ---- 4. 底盘轮向功率限制 ---- */
    chassis_remain_power = chassis_max_power - actual_steer_power - 3.0f;
    remain_power = chassis_remain_power;
    if (chassis_remain_power < 0.0f) chassis_remain_power = 0.0f;

    ApplyPowerLimit(motor_3508_kp, motor_3508_kw, motor_3508_ki,
                    motor_3508_constant, chassis_motor, chassis_remain_power, 4);
}

/* ======================== 功率检测 (调试用) ======================== */

void chassis_power_get(void)
{
    float total_steer = 0.0f, total_chassis = 0.0f;

    for (int i = 0; i < 4; i++) {
        float sp = CalcMotorPower(motor_6020_kp, motor_6020_kw,
                                  motor_6020_ki, motor_6020_constant,
                                  steer_motor[i].motor_measure.speed_rpm,
                                  steer_motor[i].motor_tar.set_current);
        if (sp > 0.0f) total_steer += sp;

        float cp = CalcMotorPower(motor_3508_kp, motor_3508_kw,
                                  motor_3508_ki, motor_3508_constant,
                                  chassis_motor[i].motor_measure.speed_rpm,
                                  chassis_motor[i].motor_tar.set_current);
        if (cp > 0.0f) total_chassis += cp;
    }

    real_chassis = total_steer + total_chassis;
    if (real_chassis < 0.0f) real_chassis = 0.0f;
}

/* ======================== 低通滤波 ======================== */

float LowPass_SetChassis(float old, float In)
{
    return (1.0f - K_Low_setchassis) * old + K_Low_setchassis * In;
}

float LowPass_SetSteer(float old, float In)
{
    return (1.0f - K_Low_setSteer) * old + K_Low_setSteer * In;
}

/* ======================== 移动检测 ======================== */

/**
 * @brief  检测底盘是否正在真实移动 (四轮平均转速 > 150rpm)
 */
int chassis_real_move_get(void)
{
    uint16_t avg_speed = 0;
    for (int i = 0; i < 4; i++) {
        int16_t rpm = chassis_motor[i].motor_measure.speed_rpm;
        avg_speed += (uint16_t)(rpm < 0 ? -rpm : rpm);
    }
    avg_speed /= 4;

    real_move_state = (avg_speed < 150) ? move_off : move_on;
    return real_move_state;
}

/* ======================== 模式切换过渡 ======================== */

void move_state_change(chassis_t *mode)
{
    (void)mode;
    /* 预留: 模式切换时的状态过渡 */
}

/* ======================== 遥控器模式 (保留兼容) ======================== */

void speed_in_reslove(chassis_t *mode)
{
#ifndef communicate
    /* 遥控器输入处理 */
    mode->speed_in.vx = (abs(rc_ctrl.rc.ch3) < 200) ? 0.0f
                        : rc_ctrl.rc.ch3 * 3.5f;
    mode->speed_in.vy = (abs(rc_ctrl.rc.ch2) < 200) ? 0.0f
                        : -rc_ctrl.rc.ch2 * 3.5f;
    mode->speed_in.wz = (rc_ctrl.rc.s_l == RC_SW_UP) ? 3500.0f : 0.0f;
    mode->if_chassis_open = (rc_ctrl.rc.s_l != RC_SW_DOWN) ? 1 : 0;
#else
    /* 串口通信模式: 速度已在别处赋值 */
    (void)mode;
#endif
}
