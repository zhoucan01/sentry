/**
 ******************************************************************************
 * @file    shoot.h
 * @brief   射击控制头文件 - 摩擦轮转速 + 射速自适应 + 温度补偿
 ******************************************************************************
 */

#ifndef __SHOOT_H
#define __SHOOT_H

#include "stdbool.h"
#include <stdint.h>

#define YES  1
#define NO   0

/* ---- 射击控制结构体 ---- */
typedef struct {
    float   shoot_speed_set;        /* 摩擦轮目标转速 (rpm) */
    float   shoot_speed_fix;        /* 射速自适应补偿 */
    float   shoot_tem_fix;          /* 温度补偿 */
    float   get_now_speed;          /* 当前实际弹速 */
    bool    if_shoot_speed_yes;     /* 转速是否到位 */
} shoot_t;

/* ---- PID参数 ---- */
#define PID_shoot_IMAX  100.0f
#define PID_shoot_MAX   16384.0f

/* ---- 弹速自适应参数 ---- */
#define MIN_SPEED       22.2f
#define MAX_SPEED       23.0f
#define UP_NUM          10.0f
#define DOWN_NUM        10.0f

/* ---- 摩擦轮基准转速 ---- */
#define FRICTION_L3_SPEED       6500.0f

/* ---- 全局变量 ---- */
extern shoot_t stander_shoot;

/* ---- 函数声明 ---- */
void shoot_run(void const *argument);
void shoot_pid_init(void);
void shoot_pid_clac(void);
void shoot_speed_set(shoot_t *mode);
void if_shoot_com(void);
bool Report_IF_Fric3508_SetSpeed(void);
void SpeedAdapt(float real_S, float min_S, float max_S,
                float *fix, float up_num, float down_num);
void Temp_Fix_30S(void);

#endif /* __SHOOT_H */