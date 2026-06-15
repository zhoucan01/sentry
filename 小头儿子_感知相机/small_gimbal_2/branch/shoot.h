/**
 ******************************************************************************
 * @file    shoot.h
 * @brief   射击控制头文件 - 摩擦轮转速 + 拨盘触发 + 射速自适应 + 热量管理
 ******************************************************************************
 */

#ifndef __SHOOT_H
#define __SHOOT_H

#include "stdbool.h"

#define YES  1
#define NO   0

/* ---- 射击模式 ---- */
typedef enum {
    SHOOT_MODE_SINGLE = 0,  /* 单发模式 */
    SHOOT_MODE_BURST,       /* 连发模式 */
} shoot_fire_mode_t;

/* ---- 射击状态机 ---- */
typedef enum {
    SHOOT_STATE_IDLE = 0,       /* 空闲 */
    SHOOT_STATE_SPINUP,         /* 摩擦轮加速中 */
    SHOOT_STATE_READY,          /* 待击发 */
    SHOOT_STATE_FIRING,         /* 击发中 */
    SHOOT_STATE_COOLDOWN,       /* 冷却中(热量过高) */
} shoot_state_t;

/* ---- 射击控制结构体 ---- */
typedef struct {
    float   shoot_speed_set;        /* 摩擦轮目标转速 (rpm) */
    float   shoot_speed_fix;        /* 射速自适应补偿 */
    float   shoot_tem_fix;          /* 温度补偿 */
    float   get_now_speed;          /* 当前实际弹速 */

    shoot_state_t    state;         /* 射击状态机 */
    shoot_fire_mode_t fire_mode;    /* 单发/连发 */

    bool    if_shoot_speed_yes;     /* 转速是否到位 */
    bool    if_heat_ok;             /* 热量是否允许击发 */
    bool    if_trigger;             /* 拨盘触发信号 */
    uint8_t burst_count;            /* 连发计数 */
    uint8_t burst_target;           /* 连发目标数 */
} shoot_t;

/* ---- PID参数 ---- */
#define PID_shoot_KP    5.0f
#define PID_shoot_KI    0.0f
#define PID_shoot_IMAX  100.0f
#define PID_shoot_MAX   16384.0f

/* ---- 弹速自适应参数 ---- */
#define MIN_SPEED       22.2f
#define MAX_SPEED       23.0f
#define UP_NUM          10.0f
#define DOWN_NUM        10.0f

/* ---- 摩擦轮基准转速 ---- */
#define FRICTION_L3_SPEED       6500.0f
#define FRICTION_READY_THRESH   500.0f   /* 转速到位阈值 (rpm) */

/* ---- 拨盘参数 ---- */
#define TRIGGER_SPEED       3000.0f      /* 拨弹电机转速 */
#define TRIGGER_DELAY_MS    50           /* 拨盘触发延时 (ms) */
#define BURST_DEFAULT_COUNT  3           /* 默认连发数 */

/* ---- 热量阈值 (裁判系统) ---- */
#define HEAT_SHOOT_LIMIT    240          /* 枪口热量上限 */
#define HEAT_COOLDOWN_BELOW 200          /* 冷却到该值以下恢复 */

/* ---- 全局变量 ---- */
extern shoot_t stander_shoot;

/* ---- 函数声明 ---- */
void shoot_run(void const *argument);
void shoot_pid_init(void);
void shoot_pid_clac(void);
void shoot_speed_set(shoot_t *mode);
void shoot_state_machine(shoot_t *mode);
void shoot_trigger_fire(shoot_t *mode);
void if_shoot_com(void);
bool Report_IF_Fric3508_SetSpeed(void);
void SpeedAdapt(float real_S, float min_S, float max_S,
                float *fix, float up_num, float down_num);
void Temp_Fix_30S(void);

#endif /* __SHOOT_H */