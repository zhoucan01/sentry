/**
 ******************************************************************************
 * @file    sentry_chassis.h
 * @brief   哨兵底盘运动控制头文件：功率限制、运动学解算、PID控制
 ******************************************************************************
 */

#ifndef __SENTRY_CHASSIS_H
#define __SENTRY_CHASSIS_H

#include "system.h"

/* ======================== 低通滤波系数 ======================== */
#define K_Low_setSteer      0.05f
#define K_Low_setchassis    0.03f

/* ======================== 电机功率模型参数 ======================== */
/* 模型: Pm = kp*Icmd*ω + kw*ω? + ki*Icmd? + constant */
/* M3508 底盘电机 */
#define motor_3508_kw       2.0854e-07f
#define motor_3508_ki       5.0323e-07f
#define motor_3508_kp       1.99688994e-6f
#define motor_3508_constant 0.72f

/* 6020 舵机电机 */
#define motor_6020_kw       8.9596e-07f
#define motor_6020_ki       3.0609e-07f
#define motor_6020_kp       1.42074e-6f
#define motor_6020_constant 0.69f

/* ======================== 功率限制参数 ======================== */
#define CONST__MAX_POWERBUFFER          60
#define CONST__POWERBUFFER_Urgent       10
#define CONST__CHASSIS_TOTAL_OUTPUT_MAX 90000  /* 底盘最大总输出 */
#define CURRENT_LIMIT                   16000.0f

/* 条件编译开关 */
#define communicate   /* 启用串口通信模式 */

/* ======================== 全局变量声明 ======================== */
extern float real_chassis;
extern float remain_power;
extern float get_power;
extern float get_speed_vx;
extern float get_speed_vy;

/* ======================== 函数声明 ======================== */
void chassis_pid_init(void);
void speed_in_reslove(chassis_t *mode);
void chassis_speed_get(chassis_t *mode);
void steer_angle_get(chassis_t *mode);
void chassis_clac(chassis_t *mode);
void chassis_speed_set(chassis_t *mode);
void speed_in_reslove_1(chassis_t *mode);
void chassis_power_limit_set(void);
void chassis_power_get(void);
void chassis_weak_handle(chassis_t *mode);
void chassis_speed_weak_handle(chassis_t *mode);
void move_state_change(chassis_t *mode);
float LowPass_SetSteer(float old, float In);
float LowPass_SetChassis(float old, float In);
float limit_addspeed(float speed_set, float speed_ref, float addspeed_limit);
float limit_addspeed1(float speed_set, float speed_ref, float addspeed_limit);
int chassis_real_move_get(void);

#endif /* __SENTRY_CHASSIS_H */
