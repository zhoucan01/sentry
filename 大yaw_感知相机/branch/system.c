/**
 ******************************************************************************
 * @file    system.c
 * @brief   哨兵系统任务 - 遥控/自动模式切换 + 底盘速度设定
 ******************************************************************************
 */

#include "bsp_transmit.h"
#include "cmsis_os.h"
#include "system.h"
#include "remote_control.h"
#include <stdlib.h>
#include "referee.h"
#include "stdbool.h"
#include "CAN_receive.h"
#include "decision.h"
#include "navigation.h"
#include "ins_task.h"

sentry_system_t sentry_system;
robot_data_t    robot_data;
pid_struct_t    pid_chaais_Sx;
pid_struct_t    pid_chaais_Sy;

static int16_t  navi_weak_state;

static void chassis_speed_set(sentry_system_t *mode);

void system_run(void const *argument)
{
    (void)argument;
    pid_init(&pid_chaais_Sx, 0.1f, 0.0f, 0.0f, 0.0f, 1.0f);
    pid_init(&pid_chaais_Sy, 0.1f, 0.0f, 0.0f, 0.0f, 1.0f);
    sentry_system.control_mode = rc_mode;
    robot_status.power_management_chassis_output = 1;
    robot_status.power_management_gimbal_output  = 1;
    robot_status.power_management_shooter_output = 1;

    for (;;) {
        robot_status.power_management_chassis_output = 1;
        robot_status.power_management_gimbal_output  = 1;
        robot_status.power_management_shooter_output = 1;

        if (get_if_communite_broke() == 1
            || decision.keyboard_disable == 1
            || judg_if_imu_error()) {
            remote_offline_set(&sentry_system);
        } else {
            choose_control_mode(&sentry_system);
            switch (sentry_system.control_mode) {
            case rc_mode:
                AGV_mode_chose(&sentry_system);
                AGV_big_yaw(&sentry_system);
                shoot_mode_chose(&sentry_system);
                small_gimbal_mode_chose(&sentry_system);
                break;
            case auto_mode:
                AGV_auto_chassis(&decision);
                AUTO_big_yaw(&sentry_system);
                Auto_shoot_mode_chose(&sentry_system);
                Auto_small_gimbal_mode(&sentry_system);
                break;
            }
        }
        vTaskDelay(1);
    }
}

uint8_t judg_if_imu_error(void)
{
    return (fabsf(INS.Pitch) > 65.0f || fabsf(INS.Roll) > 65.0f) ? 1 : 0;
}

void Auto_shoot_mode_chose(sentry_system_t *mode)
{
    if (game_state.game_progress == 4) {
        mode->shoot_mode = (rc_ctrl.rc.s_l != 2) ? shoot_on : shoot_no;
        mode->trig_mode  = (rc_ctrl.rc.s_l != 2) ? trig_on  : trig_off;
    } else {
        mode->shoot_mode = shoot_no;
        mode->trig_mode  = trig_off;
    }
}

void AGV_mode_chose(sentry_system_t *mode)
{
    switch (rc_ctrl.rc.s_l) {
    case 2:
        mode->chassis_mode = no_move;
        mode->chassis_set.Vz_state = no_spine;
        break;
    case 3:
        mode->chassis_mode = normol_move;
        mode->chassis_set.Vz_state = no_spine;
        break;
    case 1:
        if (rc_ctrl.rc.s_r == 1) {
            if (decision.Judge_condition.IF_Arrived == 0) {
                mode->chassis_mode = navigation_move;
                mode->chassis_set.Vz_state = no_spine;
            } else {
                mode->chassis_mode = normol_move;
                mode->chassis_set.Vz_state = low_spine;
            }
        } else {
            mode->chassis_set.Vz_state = high_spine;
            mode->chassis_mode = normol_move;
        }
        break;
    }
    chassis_speed_set(mode);
}

static void chassis_speed_set(sentry_system_t *mode)
{
    switch (mode->chassis_mode) {
    case no_move:
        mode->chassis_set.set_vx = 0.0f;
        mode->chassis_set.set_vy = 0.0f;
        break;
    case normol_move:
        mode->chassis_set.set_vx = -rc_ctrl.rc.ch1 * 4.5f * WHEEL_FACTOR;
        mode->chassis_set.set_vy = -rc_ctrl.rc.ch0 * 4.5f * WHEEL_FACTOR;
        break;
    case navigation_move:
        mode->chassis_set.set_vx = -navigation_rx.navi_vx;
        mode->chassis_set.set_vy =  navigation_rx.navi_vy;
        break;
    }

    navi_tx_count++;
    if (fabsf(navigation_rx.navi_vx) < 0.15f
        && fabsf(navigation_rx.navi_vy) < 0.15f) {
        navi_weak_state++;
    } else {
        navi_weak_state = 0;
    }
    navigation_rx.if_lost_navi = (navi_tx_count > 500 || navi_weak_state > 100) ? 1 : 0;

    if (decision.Judge_condition.If_chassis_weak == 1) {
        mode->chassis_set.set_vx *= 0.33f;
        mode->chassis_set.set_vy *= 0.33f;
    }
}
