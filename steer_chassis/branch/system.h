/**
 ******************************************************************************
 * @file    system.h
 * @brief   底盘系统核心头文件：状态定义、运动模式枚举、底盘结构体
 ******************************************************************************
 */

#ifndef __SYSTEM_H
#define __SYSTEM_H

#include "struct_typedef.h"
#include "bsp_pid.h"

/* ======================== 舵机初始角度校准 ======================== */
/* 舵机初始化使底盘朝正方向，对应舵机装上时的偏置角 */
#define steer_init_angle_FR   7836    /* 360° - 45° */
#define steer_init_angle_BR   2986    /* 45° */
#define steer_init_angle_BL   6423    /* 135° */
#define steer_init_angle_FL   132     /* 360° - 135° */

/* ======================== 底盘物理参数 ======================== */
#define WHEEL_PERIMETER       376.99f  /* 车轮周长 (mm) */
#define M3508_RATIO           19.2032f /* 电机减速比 */
#define Radius                62.0f    /* 轮径 (mm) */
#define distance_x            0.18f    /* 舵轮中心到底盘中心 X 距离 (m) */
#define distance_y            0.18f    /* 舵轮中心到底盘中心 Y 距离 (m) */
#define WHEEL_FACTOR          0.00034f /* 电机转速 rpm → 真实速度 m/s */
#define forword_ecd           -1.57f   /* 正前方对应的编码器角度 */

/* ======================== 类型定义 ======================== */

/** 三轴速度 */
typedef struct {
    float vx;
    float vy;
    float wz;
} speed_t;

/** 底盘运动模式 */
typedef enum {
    no_move         = 0,    /* 底盘不动 */
    normol_move,            /* 常规移动 */
    navigation_move,        /* 导航移动 */
} chassis_mode_t;

/** 陀螺自旋状态 */
typedef enum {
    no_spine    = 0,
    low_spine,
    mid_spine,
    high_spine,
    lock_spine,
} Vz_state_t;

/** 真实移动状态 */
typedef enum {
    move_on  = 0,
    move_off,
} real_move_state_t;

/** 底盘主状态结构体 */
typedef struct {
    chassis_mode_t      chassis_mode;
    float               init_yaw;
    speed_t             speed_in;               /* 输入目标速度 */
    speed_t             speed_usart_get;        /* 串口接收速度 */
    speed_t             speed_reslove;          /* 解算后底盘坐标系速度 */
    speed_t             last_reslove;
    float               dm_ecd;                 /* 大云台编码器角度 */
    float               diff_angle;             /* 云台-底盘夹角 */
    uint8_t             if_chassis_open;
    Vz_state_t          Vz_state;
    speed_t             chassis[4];             /* 四轮速度分量 */
    float               speed_set[4];           /* 四轮目标速度 */
    float               last_speed_set[4];
    float               steer_angle[4];         /* 舵角 (rad) */
    float               steer_ecd[4];           /* 舵角 (编码器值) */
    float               steer_final_ecd[4];     /* 最终输出舵角 */
    float               decay_ecd[4];           /* 舵角衰减系数 */
    float               speed_direct[4];        /* 速度方向 (±1) */
    float               steer_resolve_ecd[4];
    float               steer_speed_set[4];
    float               last_ecd[4];
    int                 speed_flag[4];
    float               chassis_last_total_ecd[4];
    float               chassis_diff_ecd[4];
    float               Sx, Sy;                 /* 里程计位置 (m) */
    float               relative_init_angle;    /* 相对初始角度 */
    float               chassis_ecd_diff[4];
    uint8_t             game_progress;
    uint8_t             if_outpost_des;
    int8_t              wz_back;
    float               steer_diff_angle;
    int                 move_flag;
    real_move_state_t   real_move_state;
} chassis_t;

/* ======================== 全局变量声明 ======================== */
extern chassis_t         chassis;
extern chassis_mode_t    last_chassis_mode;
extern pid_struct_t      pid_steer_ecd[4];
extern pid_struct_t      pid_steer_speed[4];

/* ======================== 函数声明 ======================== */
void sentry_chassis_init(void);
void get_diff_angle(chassis_t *mode);
void chassis_data_updata(chassis_t *mode);
void chassis_normol_set(chassis_t *mode);
void chassis_follow_set(chassis_t *mode);
void chassis_spine_set(chassis_t *mode);
void chassis_mode_chose(chassis_t *mode);
void chassis_no_move_set(chassis_t *mode);
void chassis_mode_updata(chassis_t *mode);
void chassis_lock_move(chassis_t *mode);
void chassis_low_spine_set(chassis_t *mode);
void chassis_mid_spine_set(chassis_t *mode);
void chassis_navi_set(chassis_t *mode);
void chassis_weak_handle(chassis_t *mode);
float speed_filter(float speed_in);

#endif /* __SYSTEM_H */


