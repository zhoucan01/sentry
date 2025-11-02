#ifndef __BSP_TRANSMIT_H
#define __BSP_TRANSMIT_H
#include "struct_typedef.h"
#include "small_gimbal.h"

#define DATA_COUNT_TX	25
#define DATA_COUNT	25



#define DATA_COUNT_RX	72
#define DATA_RX_count 18

#define USART_RX_HAED   0XA5
#define USART_RX_END    0XAA
#define USART_TX_HAED   0XA5
#define USART_TX_END    0XAA


//#define SEND_ID_HEAD1            0x301
//#define SHOOT_SEND_ID_HEAD1      0x303
//#define SEND_ID_HEAD_REMOTE1     0x305

//#define SEND_ID_HEAD2            0x302
//#define SHOOT_SEND_ID_HEAD2      0x304
//#define SEND_ID_HEAD_REMOTE2     0x306
typedef struct
{
	uint8_t if_chassis_open;
	uint8_t chassis_mode;
	float chassis_vx;
	float chassis_vy;
	float chassis_wz;
	int16_t up_yaw;
  uint16_t chassis_buff;
  uint8_t if_outpost_des;
  uint8_t game_progress;
  uint8_t IF_STOP;
  uint8_t trans_seq;
  uint8_t If_chassis_weak;
  uint16_t chassis_power_limit;
  uint8_t position_x;
  uint8_t position_y;
}USART_Tx_data_t;


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
}int16_to_8;

typedef struct
{
  float odom_Sx;
  float odom_Sy;
  float odom_Vx;
  float odom_Vy;
	
}USART_Rx_data_t;

extern USART_Rx_data_t USART_Rx_data; 
extern uint8_t USART_Rx_data_handle[DATA_COUNT_RX];
extern uint8_t USART_Tx_buff[DATA_COUNT_TX]; 
extern USART_Tx_data_t  USART_TX_data;
extern small_gimbal_send_t small_gimbal_send;
void Transmit_data_send(USART_Tx_data_t *data);
void get_referr_data();
void reslove_referee_data();

void reslove_referee_data();
void usart_data_updata(USART_Tx_data_t *data);
void RX_USART_data_Handle(uint8_t *buff,USART_Rx_data_t *data);
//uint8_t USART_Rx_data_handle[DATA_COUNT_RX];


//extern USART_Rx_data_t USART_Rx_data;
//void Head1_data_Handle(uint8_t *buff,USART_Rx_data_t *data);
#endif