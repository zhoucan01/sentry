/**
 ******************************************************************************
 * @file    navigation.c
 * @brief   导航通信 - USB虚拟串口收发 + 坐标系转�?
 ******************************************************************************
 */

#include "ins_task.h"
#include "navigation.h"
#include <stdlib.h>
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "decision.h"
#include "bsp_transmit.h"
#include "referee.h"

/* ---- 全局变量 ---- */
location_t       location = { .Sx = 0, .Sy = 0, .Sx_set = 0, .Sy_set = 0, .yaw_update = 0 };
navigation_rx_t  navigation_rx;
navigation_tx_t  navigation_tx;
uint8_t          TX_Buff[navigation_tx_len];

int navi_tx_count;

void navigation_rx_handle(uint8_t *buff, uint32_t Len, navigation_rx_t *data)
{
    if (buff == NULL) return;
    if (buff[0] != CONST_HEAD0 || buff[Len - 1] != CONST_END0) return;

    navi_tx_count = 0;

    u8_to_float Vx, Vy, Vz, Sx, Sy;
    for (int i = 0; i < 4; i++) {
        Vx.d[i] = buff[i + 1];
        Vy.d[i] = buff[i + 5];
        Vz.d[i] = buff[i + 9];
        Sx.d[i] = buff[i + 13];
        Sy.d[i] = buff[i + 17];
    }

    data->If_get_path = buff[Len - 3];
    data->navi_vx     = Vx.data;
    data->navi_vy     = Vy.data;
    data->navi_wz     = Vz.data;
    data->current_x   = Sx.data;
    data->current_y   = Sy.data;
}

void Navigation_Tx_Send(navigation_tx_t *data)
{
    data->current_yaw   = INS.Yaw;
    data->current_pitch = INS.Roll;
    data->current_roll  = INS.Pitch;
    data->Odom_Vx       = 0.0f;
    data->Odom_Vy       = 0.0f;
    data->yaw_Wz        = INS.Gyro[2];

    TX_Buff[0] = CONST_HEAD0;
    memcpy(TX_Buff + 1,  &data->navi_set_x_pos, 4);
    memcpy(TX_Buff + 5,  &data->navi_set_y_pos, 4);
    memcpy(TX_Buff + 9,  &data->current_yaw,    4);
    memcpy(TX_Buff + 13, &data->current_pitch,  4);
    TX_Buff[navigation_tx_len - 1] = CONST_END0;

    CDC_Transmit_FS(TX_Buff, navigation_tx_len);
}
