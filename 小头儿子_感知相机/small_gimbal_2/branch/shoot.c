/**
 ******************************************************************************
 * @file    shoot.c
 * @brief   摩擦轮射击控制 - 转速PID + 温度补偿 + 射速自适应
 ******************************************************************************
 */

#include "shoot.h"
#include "bsp_pid.h"
#include "system.h"
#include "CAN_receive.h"
#include "ins_task.h"
#include "cmsis_os.h"
#include "struct_typedef.h"
#include "stm32f4xx_hal.h"
#include "remote_control.h"
#include "Nautilus_Vision.h"
#include "stdbool.h"

shoot_t stander_shoot = {
    .shoot_speed_set = 0,
    .if_shoot_speed_yes = 0,
    .get_now_speed = 0,
    .shoot_speed_fix = 0.0f,
};

pid_struct_t pid_shoot_speed[2];
com_mode_t   shoot_com;

static int      fire_num = 0;
static bool     if_num_updata = NO;
static uint8_t  SpeedErr_cnt = 0;

void shoot_run(void const *argument)
{
    (void)argument;
    vTaskDelay(1);
    shoot_pid_init();
    for (;;)
    {
        if_shoot_com();
        switch (shoot_com)
        {
        case com_nom:
        {
            shoot_speed_set(&stander_shoot);
            shoot_pid_clac();
            break;
        }
        case com_err:
        {
            stander_shoot.shoot_speed_set = 0;
            shoot_pid_clac();
            break;
        }
        }
        vTaskDelay(1);
    }
}

void shoot_pid_init(void)
{
    pid_init(&pid_shoot_speed[0], 5.0f, 0.0f, 0.0f, PID_shoot_IMAX, PID_shoot_MAX);
    pid_init(&pid_shoot_speed[1], 5.0f, 0.0f, 0.0f, PID_shoot_IMAX, PID_shoot_MAX);
}

void shoot_speed_set(shoot_t *mode)
{
    if (fire_num != control_data.shoot_num)
    {
        if_num_updata = YES;
        fire_num = control_data.shoot_num;
    }
    else
    {
        if_num_updata = NO;
    }

    if (if_num_updata == YES && control_data.shoot_speed > 15.0f)
    {
        Temp_Fix_30S();
        SpeedAdapt(control_data.shoot_speed, MIN_SPEED, MAX_SPEED,
                   &mode->shoot_speed_fix, UP_NUM, DOWN_NUM);
    }

    if (control_data.shoot_mode == shoot_on)
    {
        mode->get_now_speed = control_data.shoot_speed;
        mode->shoot_speed_set = FRICTION_L3_SPEED
                              + mode->shoot_speed_fix
                              + mode->shoot_tem_fix;
    }
    else
    {
        mode->shoot_speed_set = 0.0f;
    }
}

void if_shoot_com(void)
{
    shoot_com = (control_data.shoot_mode != shoot_no
              && gimbal_system.control_com == com_nom)
              ? com_nom : com_err;
}

void shoot_pid_clac(void)
{
    if (control_data.shoot_mode == shoot_on)
    {
        shoot_motor[0].motor_tar.set_current = pid_calc(&pid_shoot_speed[0],
            shoot_motor[0].motor_measure.speed_rpm,  stander_shoot.shoot_speed_set);
        shoot_motor[1].motor_tar.set_current = pid_calc(&pid_shoot_speed[1],
            shoot_motor[1].motor_measure.speed_rpm, -stander_shoot.shoot_speed_set);
    }
    else
    {
        shoot_motor[0].motor_tar.set_current = pid_calc(&pid_shoot_speed[0],
            shoot_motor[0].motor_measure.speed_rpm, 0.0f);
        shoot_motor[1].motor_tar.set_current = pid_calc(&pid_shoot_speed[1],
            shoot_motor[1].motor_measure.speed_rpm, 0.0f);
    }
}

/* 反馈3508电机是否达到目标转速 */
bool Report_IF_Fric3508_SetSpeed(void)
{
    return (fabsf(shoot_motor[0].motor_measure.speed_rpm
                  - stander_shoot.shoot_speed_set) < 500.0f
         && fabsf(-shoot_motor[1].motor_measure.speed_rpm
                  - stander_shoot.shoot_speed_set) < 500.0f);
}

void SpeedAdapt(float real_S, float min_S, float max_S,
                float *fix, float up_num, float down_num)
{
    if (real_S < min_S && real_S > 8.0f)
    {
        SpeedErr_cnt++;
    }
    else if (real_S >= min_S && real_S <= max_S)
    {
        SpeedErr_cnt = 0;
    }
    if (SpeedErr_cnt == 1)
    {
        SpeedErr_cnt = 0;
        *fix += up_num;
    }
    if (real_S > max_S)
    {
        *fix -= down_num;
    }
}

void Temp_Fix_30S(void)
{
    float temp_real = ((float)shoot_motor[0].motor_measure.temperate
                    + (float)shoot_motor[1].motor_measure.temperate) / 2.0f;
    stander_shoot.shoot_tem_fix = (temp_real - 35.0f) * 0.5f;
}