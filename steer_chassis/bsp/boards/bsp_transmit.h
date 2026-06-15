/**
 ******************************************************************************
 * @file    bsp_transmit.h
 * @brief   串口收发模块头文件
 ******************************************************************************
 */

#ifndef __BSP_TRANSMIT_H
#define __BSP_TRANSMIT_H

#include "struct_typedef.h"
#include "system.h"

/* ---- 帧协议定义 ---- */
#define DATA_COUNT_RX   100
#define DATA_COUNT      25
#define DATA_Tx_count   18

#define USART_RX_HAED   0xA5
#define USART_RX_END    0xAA
#define USART_TX_HAED   0xA5
#define USART_TX_END    0xAA

/* ---- 类型定义 ---- */

/** 字节→浮点数 联合体 */
typedef union {
    float   data;
    uint8_t d[4];
} u8_to_float;

/** 字节→uint16 联合体 */
typedef union {
    uint16_t data;
    uint8_t  d[2];
} u8_to_u16;

/** 字节→int16 联合体 */
typedef union {
    int16_t data;
    uint8_t d[2];
} u8_to_int16;

/** 上位机→底盘 接收数据结构 */
typedef struct {
    uint8_t     if_chassis_open;
    uint8_t     chassis_mode;
    float       chassis_vx;
    float       chassis_vy;
    float       chassis_wz;
    float       up_yaw;
    float       big_yaw_ecd;
    float       buffer_energy;
    uint16_t    chassis_buff;
    uint8_t     game_progress;
    uint8_t     if_outpost_des;
    uint16_t    chassis_Power_limit;
    uint8_t     trans_seq;
    uint8_t     if_ecd_update;
    Vz_state_t  Vz_state;
    float       yaw_diff;
    float       ecd_diff;
    uint8_t     if_arrived;
    uint8_t     position_x;
    uint8_t     position_y;
    float       current_x;
    float       current_y;
} USART_Rx_data_t;

/** 底盘→上位机 发送数据结构 */
typedef struct {
    float odom_Sx;
    float odom_Sy;
    float odom_Vx;
    float odom_Vy;
} USART_Tx_data_t;

/* ---- 全局变量声明 ---- */
extern uint8_t          USART_Rx_data_handle[DATA_COUNT_RX];
extern uint8_t          Tx_Buff[DATA_Tx_count];
extern USART_Rx_data_t  USART_Rx_data;
extern USART_Tx_data_t  USART_Tx_data;
extern uint8_t          transmit_seq;

/* ---- 函数声明 ---- */
void RX_USART_data_Handle(uint8_t *buff, USART_Rx_data_t *data);
void USART_TX_send(uint8_t *buff, USART_Tx_data_t *data);

#endif /* __BSP_TRANSMIT_H */
