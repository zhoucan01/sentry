/**
 ******************************************************************************
 * @file    gimbal_big_yaw.c
 * @brief   �?Yaw云台控制 �? 遥控/视�??/导航三模式切�? + 限位巡航
 * @author  周灿
 ******************************************************************************
 */

#include "gimbal_big_yaw.h"
#include "system.h"
#include "cmsis_os.h"
#include "referee.h"
#include "remote_control.h"
#include "can_receive.h"
#include "ins_task.h"
#include "bsp_pid.h"
#include "navigation.h"
#include "stdbool.h"

/* ---- PID控制�? ---- */
pid_struct_t pid_yaw_angle;
pid_struct_t pid_yaw_speed;
pid_struct_t pid_vision_yaw_angle;
pid_struct_t pid_vision_yaw_speed;
pid_struct_t pid_navi_yaw_speed;

/* ---- 全局状�? ---- */
com_mode_t gim_com;
com_mode_t last_gim;
gimbal_t gimbal;

int      big_yaw_lost_count;   /* �?yaw�?电�?�数 */
static uint8_t  last_vision_mode;
static lock_state_t     Last_lock_state;
static Vision_look_state_t Last_look_state;

/* ---- 内部函数声明 ---- */
static void get_big_gimbal_com(void);
static void gimbal_curise_set(gimbal_t *mode);
static void gimbal_vision_set(gimbal_t *mode);
static bool get_gimbal_response_state(gimbal_t *mode);
static float compare_yaw_add(float yaw_in);

/* ======================== 初�?�化 ======================== */

void gimbal_pid_init(void)
{
    pid_init(&pid_yaw_angle,        1.0f, 0.0f, 0.0f, 0.0f, 10.0f);
    pid_init(&pid_yaw_speed,        1.0f, 0.0f, 0.0f, 0.0f, 10.0f);
    pid_init(&pid_navi_yaw_speed,   2.5f, 0.0f, 0.0f, 0.0f,  5.0f);
    pid_init(&pid_vision_yaw_angle, 1.0f, 0.0f, 0.0f, 0.0f, 10.0f);
    pid_init(&pid_vision_yaw_speed, 1.0f, 0.0f, 0.0f, 0.0f, 10.0f);
}

/* ======================== 主任�? ======================== */

void big_yaw_run(void const *argument)
{
    (void)argument;
    gimbal.curise_direction = 1;
    gimbal_pid_init();

    for (;;) {
        get_big_gimbal_com();

        switch (gim_com) {
        case com_err:
            gimbal.yaw_set        = INS.YawTotalAngle;
            gimbal.yaw_vision_set  = INS.YawTotalAngle;
            gimbal.if_big_yaw_can  = 0;
            break;
        case com_nom:
            gimbal.if_big_yaw_can = 1;
            gimbal_mode_set(&gimbal);
            break;
        }
        vTaskDelay(1);
    }
}

/* ======================== 通信状态�?��? ======================== */

static void get_big_gimbal_com(void)
{
    big_yaw_lost_count++;

    if (sentry_system.chassis_mode == no_move
        || get_if_communite_broke() == 1
        || robot_status.power_management_gimbal_output == 0
        || big_yaw_lost_count > 200) {
        gim_com = com_err;
    } else {
        gim_com = com_nom;
    }

    if (robot_status.power_management_gimbal_output == 0) {
        yaw_motor.Err = 0;
    }

    last_gim = gim_com;
}

/* ======================== 模式调度 ======================== */

void gimbal_mode_set(gimbal_t *mode)
{
    switch (sentry_system.big_yaw_mode) {
    case big_yaw_off:
        mode->yaw_set       = INS.YawTotalAngle;
        mode->yaw_vision_set = INS.YawTotalAngle;
        break;
    case big_yaw_rc:
        gimbal_vision_no_mode();
        break;
    case big_yaw_pc:
        gimbal_vision_on_mode(mode);
        break;
    }
}

/* ======================== 遥控模式 (无�?��??) ======================== */

void gimbal_vision_no_mode(void)
{
    gimbal.yaw_set -= sentry_system.set_yaw_in;
    gimbal.yaw_vision_set = INS.YawTotalAngle;
    gimbal.if_update = 1;

    gimbal.big_yaw_speed_set = pid_calc(&pid_yaw_angle,
        INS.YawTotalAngle, gimbal.yaw_set);
    yaw_motor.Torque_SET = pid_calc(&pid_yaw_speed,
        INS.Gyro[2], gimbal.big_yaw_speed_set);
}

/* ======================== 视�?�模�? ======================== */

void gimbal_vision_on_mode(gimbal_t *mode)
{
    if (decision.point == WE_FORTRESS_POINT
        && decision.Judge_condition.IF_Arrived == 1) {
        mode->gimbal_mode = gimbal_vision_mode;
        gimbal_vision_set(mode);
        mode->if_update = 1;
    } else if (receive_gimbal_data.vision_state == vision_lost) {
        mode->gimbal_mode = gimbal_curise_mode;
        gimbal_curise_set(mode);
        mode->if_update = 1;
    } else {
        mode->gimbal_mode = gimbal_vision_mode;
        gimbal_vision_set(mode);
    }
}

static void gimbal_curise_set(gimbal_t *mode)
{
    mode->yaw_vision_set += mode->curise_direction * yaw_cusise_diff;
    mode->if_update       = 1;
    mode->get_current_yaw = INS.YawTotalAngle;
    gimbal.yaw_set        = INS.YawTotalAngle;

    gimbal.big_yaw_speed_set = pid_calc(&pid_vision_yaw_angle,
        INS.YawTotalAngle, gimbal.yaw_vision_set);
    yaw_motor.Torque_SET = pid_calc(&pid_vision_yaw_speed,
        INS.Gyro[2], gimbal.big_yaw_speed_set);
}

static void gimbal_vision_set(gimbal_t *mode)
{
    gimbal.yaw_set = INS.YawTotalAngle;
    get_gimbal_response_state(mode);

    /* 左限�? �? 反向巡航 */
    if (receive_gimbal_data.vision_state != vision_lost
        && receive_gimbal_data.lock_state == left_lock
        && mode->if_update == 1) {
        mode->yaw_add         = receive_gimbal_data.small_yaw_add;
        mode->get_current_yaw = INS.YawTotalAngle;
        mode->curise_direction = -1 * direc;
        mode->if_update       = 0;
    }
    /* 右限�? �? 反向巡航 */
    else if (receive_gimbal_data.vision_state != vision_lost
        && receive_gimbal_data.lock_state == right_lock
        && mode->if_update == 1) {
        mode->yaw_add         = receive_gimbal_data.small_yaw_add;
        mode->get_current_yaw = INS.YawTotalAngle;
        mode->curise_direction = 1 * direc;
        mode->if_update       = 0;
    }
    /* 无阻�? �? 保持 */
    else if (receive_gimbal_data.vision_state != vision_lost
        && receive_gimbal_data.lock_state == no_block
        && mode->if_update == 1) {
        /* 保持当前�?�? */
    }

    gimbal.big_yaw_speed_set = pid_calc(&pid_vision_yaw_angle,
        INS.YawTotalAngle, gimbal.yaw_vision_set);
    yaw_motor.Torque_SET = pid_calc(&pid_vision_yaw_speed,
        INS.Gyro[2], gimbal.big_yaw_speed_set);
}

/* ======================== 限位响应检�? ======================== */

static bool get_gimbal_response_state(gimbal_t *mode)
{
    if (fabsf(mode->yaw_vision_set - mode->get_current_yaw)
        < compare_yaw_add(mode->yaw_add) && mode->if_update == 0) {
        mode->yaw_vision_set += (mode->curise_direction == -1) ? -0.2f : 0.2f;
        mode->if_update = 0;
    } else {
        mode->if_update = 1;
    }
    return (bool)mode->if_update;
}

static float compare_yaw_add(float yaw_in)
{
    return (fabsf(yaw_in) < 50.0f) ? 50.0f : fabsf(yaw_in);
}

/* ======================== 导航模式 ======================== */

void gimbal_navi_set(gimbal_t *mode)
{
    (void)mode;
    yaw_motor.Torque_SET = pid_calc(&pid_navi_yaw_speed,
        INS.Gyro[2] * 1.0f, navigation_rx.navi_wz * 1.9f);
    gimbal.yaw_set       = INS.YawTotalAngle;
    gimbal.yaw_vision_set = INS.YawTotalAngle;
}



