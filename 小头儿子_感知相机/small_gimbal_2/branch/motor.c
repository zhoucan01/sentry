/**
 ******************************************************************************
 * @file    motor.c
 * @brief   电机发送任务 - CAN发送 + DM电机控制
 ******************************************************************************
 */

#include "motor.h"
#include "cmsis_os.h"
#include "struct_typedef.h"
#include "stm32f4xx_hal.h"
#include "ins_task.h"
#include "CAN_receive.h"
#include "system.h"
#include "CAN_transmit.h"
#include "Nautilus_Vision.h"
#include "gimbal.h"

#define gimbal_send
#define shoot_send

void motor_run(void)
{
    vTaskDelay(2);
    for (;;)
    {
        dm_pitch_start();
        motor_Send();
        osDelay(1);
    }
}

void dm_pitch_start(void)
{
    if (gimbal.if_pitch_can == 1)
    {
        if (pitch_motor.Err == 0)
        {
            pitch_Enable();
        }
    }
    else
    {
        if (pitch_motor.Err == 1)
        {
            pitch_Unable();
        }
    }
}

void pitch_Enable(void)
{
    for (int i = 0; i < 10; i++)
    {
        dm_motor_START(dm_pitch);
        vTaskDelay(2);
    }
}

void pitch_Unable(void)
{
    dm_motor_STOP(dm_pitch);
    vTaskDelay(2);
}

void motor_Send(void)
{
#ifdef gimbal_send
    gimbal_normol_send();
#else
    gimbal_error_send();
#endif

#ifdef shoot_send
    shoot_normol_send();
#else
    shoot_error_send();
#endif
    osDelay(1);
    small_to_big(&hcan1, 0x402);
}

void gimbal_normol_send(void)
{
    set_motor_current(can2_1FF, yaw_motor.motor_tar.set_current, 0, 0, 0);
    osDelay(1);
    dm_motor_Ctrl(dm_pitch, 0, 0, 0, 0, pitch_motor.Torque_SET);
}

void gimbal_error_send(void)
{
    set_motor_current(can2_1FF, 0, 0, 0, 0);
    osDelay(1);
    dm_motor_Ctrl(dm_pitch, 0, 0, 0, 0, 0);
}

void shoot_normol_send(void)
{
    set_motor_current(can1_200, 0,
        shoot_motor[0].motor_tar.set_current,
        shoot_motor[1].motor_tar.set_current,
        trig_motor.motor_tar.set_current);
}

void shoot_error_send(void)
{
    set_motor_current(can1_200, 0, 0, 0, 0);
}

void small_to_big(CAN_HandleTypeDef *hcan, uint32_t id_range)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t             tx_data[8];
    uint32_t            send_mail_box;

    tx_header.StdId = id_range;
    tx_header.IDE   = CAN_ID_STD;
    tx_header.RTR   = CAN_RTR_DATA;
    tx_header.DLC   = 8;

    int16_to_8 yaw_add, ecd_trans;
    yaw_add.data  = (int16_t)get_yaw_diff;
    ecd_trans.data = (int16_t)gimbal.transform.ANTI_current_transform_angle;

    tx_data[0] = gimbal.vision_state;
    tx_data[1] = gimbal.lock_state;
    tx_data[2] = yaw_add.d[0];
    tx_data[3] = yaw_add.d[1];
    tx_data[4] = Rx_Vision.armor_id;
    tx_data[5] = Rx_Vision.armor_dist;
    tx_data[6] = ecd_trans.d[0];
    tx_data[7] = ecd_trans.d[1];

    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &send_mail_box);
}