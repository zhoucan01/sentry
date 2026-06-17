/**
 ******************************************************************************
 * @file    system.c
 * @brief   小云台系统任务 - 通信检测 + 视觉重置 + 离线检测
 ******************************************************************************
 */

#include "system.h"
#include "CAN_receive.h"
#include "ins_task.h"
#include "cmsis_os.h"
#include "struct_typedef.h"
#include "stm32f4xx_hal.h"
#include "remote_control.h"
#include "Nautilus_Vision.h"

gimbal_system_t gimbal_system;
toe_offline_t   toe_offline[ERROR_LIST_LENGHT] = {0};
Wheel_State_t   Wheel_State;

#if INCLUDE_uxTaskGetStackHighWaterMark
    uint32_t detect_task_stack;
#endif

/* ---- 内部函数 ---- */
static void detect_init(void);

void system_run(void const *argument)
{
    (void)argument;
    for (;;)
    {
        get_control_mode(&control_data);
        Tj_Send_Data(&TJ_Vision_Tx);
        communicate_pin_state(&control_data);
        vision_reset(&TJ_Vision_Rx);
        vTaskDelay(1);
    }
}

void get_control_mode(control_data_t *mode)
{
    if (toe_offline[BOARD_TOE].communication_state == COMMUNICATION_NONE)
    {
        gimbal_system.control_com = com_err;
        com_data_reset(&control_data);
    }
    else
    {
        gimbal_system.control_com = com_nom;
    }
    gimbal_system.last_control_com = gimbal_system.control_com;
}

void communicate_pin_state(control_data_t *mode)
{
    (void)mode;
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10,
        (gimbal_system.control_com == com_nom) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void vision_reset(TJ_Vision_Rx_t *data)
{
    if (toe_offline[VISION_TOE].communication_state == COMMUNICATION_NONE)
    {
        memset(data, 0, sizeof(TJ_Vision_Rx_t));
    }
}

void com_data_reset(control_data_t *mode)
{
    mode->data_wheel       = 0;
    mode->gimbal_mode      = small_gimbal_off;
    mode->gimbal_pitch_in  = 0;
    mode->gimbal_yaw_in    = 0;
    mode->if_gimbal_can    = 0;
    mode->robot_color      = NO_CONTACT;
    mode->shoot_mode       = shoot_no;
    mode->other_data.wheel_state = 0;
}

/* ======================== 离线检测任务 ======================== */

static void detect_init(void)
{
    static const uint16_t max_rate[ERROR_LIST_LENGHT] = {
        FRIC_R_MAX_OFFLINE_FRAME_RATE,
        FRIC_L_MAX_OFFLINE_FRAME_RATE,
        TRIG_MAX_OFFLINE_FRAME_RATE,
        YAW_MAX_OFFLINE_FRAME_RATE,
        PITCH_MAX_OFFLINE_FRAME_RATE,
        RC_RECIVE_MAX_OFFLINE_FRAME_RATE,
        VISION_MAX_OFFLINE_FRAME_RATE,
        BOARD_MAX_OFFLINE_FRAME_RATE,
    };

    for (int i = 0; i < ERROR_LIST_LENGHT; i++)
    {
        toe_offline[i].offline_frame_rate    = 0;
        toe_offline[i].max_offline_frame_rate = max_rate[i];
        toe_offline[i].communication_state    = COMMUNICATION_MORMAL;
        toe_offline[i].toe_offline_data_handle_f = NULL;
        toe_offline[i].toe_unable_f               = NULL;
        toe_offline[i].toe_connect_soft_restart_f = NULL;
    }
}

void DETECT_task(void const *argument)
{
    (void)argument;
    detect_init();

    for (;;)
    {
        for (int i = 0; i < ERROR_LIST_LENGHT; i++)
        {
            toe_offline[i].offline_frame_rate++;
            if (toe_offline[i].offline_frame_rate
                > toe_offline[i].max_offline_frame_rate)
            {
                toe_offline[i].communication_state = COMMUNICATION_NONE;
                toe_offline[i].offline_frame_rate =
                    toe_offline[i].max_offline_frame_rate;
                if (toe_offline[i].toe_offline_data_handle_f != NULL)
                {
                    toe_offline[i].toe_offline_data_handle_f();
                }
            }
            else
            {
                toe_offline[i].communication_state = COMMUNICATION_MORMAL;
            }
        }
        vTaskDelay(DETECT_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
        detect_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

bool toe_is_error(uint8_t toe)
{
    return (COMMUNICATION_NONE == toe_offline[toe].communication_state);
}

void communication_frame_rate_update(uint8_t toe)
{
    toe_offline[toe].offline_frame_rate = 0;
}