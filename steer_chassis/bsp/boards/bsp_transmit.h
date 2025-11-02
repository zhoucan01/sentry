#ifndef __BSP_TRANSMIT_H
#define __BSP_TRANSMIT_H
#include "struct_typedef.h"
#include "system.h"

#define DATA_COUNT_RX	100
//#define DATA_COUNT_TX	22
#define DATA_COUNT	25

#define DATA_Tx_count 18

#define USART_RX_HAED   0XA5
#define USART_RX_END    0XAA
#define USART_TX_HAED   0XA5
#define USART_TX_END    0XAA

//typedef struct
//{
//	uint8_t if_chassis_open;
//	uint8_t chassis_mode;
//	float chassis_vx;
//	float chassis_vy;
//	float chassis_wz;
//	float up_yaw;
//  uint16_t chassis_buff;
//  
//}USART_Tx_data_t;



typedef union 
{
	float data;
	uint8_t d[4];
}u8_to_float;


typedef union
{
  uint16_t data;
  uint8_t d[2];
}u8_to_u16;

typedef union
{
  int16_t data;
  uint8_t d[2];
}u8_to_int16;

typedef struct
{
 
 
	uint8_t if_chassis_open;
	uint8_t chassis_mode;
	float chassis_vx;
	float chassis_vy;
	float chassis_wz;
	float up_yaw;
  float big_yaw_ecd;
  float buffer_energy;
  uint16_t chassis_buff;
  uint8_t game_progress;
  uint8_t if_outpost_des;
   uint16_t chassis_Power_limit;
   uint8_t trans_seq;

   uint8_t if_ecd_update;
   Vz_state_t Vz_state;
   float yaw_diff;
   float ecd_diff;
   
   uint8_t if_arrived;
   
  uint8_t position_x;
  uint8_t position_y;
  float current_x;
  float current_y;
   
   
}USART_Rx_data_t;

typedef struct 
{

  float odom_Sx;
  float odom_Sy;
  float odom_Vx;
  float odom_Vy;
  
}USART_Tx_data_t;
extern uint8_t USART_Rx_data_handle[DATA_COUNT_RX];
extern uint8_t Tx_Buff[DATA_Tx_count];
extern USART_Rx_data_t  USART_Rx_data;
extern USART_Tx_data_t USART_Tx_data;
//extern uint8_t USART_Tx_buff[DATA_COUNT_TX]; 
//void Transmit_data_send(USART_Tx_data_t *data);
void RX_USART_data_Handle(uint8_t *buff,USART_Rx_data_t *data);
extern uint8_t transmit_seq;
void USART_TX_send(uint8_t *buff,USART_Tx_data_t *data);
//extern USART_Rx_data_t USART_Rx_data;
//void Head1_data_Handle(uint8_t *buff,USART_Rx_data_t *data);
#endif