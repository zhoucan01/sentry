#ifndef __BSP_TRANSMIT_H
#define __BSP_TRANSMIT_H
#include "struct_typedef.h"
#define DATA_COUNT_RX	96
#define DATA_COUNT_TX	24
#define DATA_COUNT	24


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
	float up_yaw;
  uint16_t chassis_buff;
  uint8_t if_outpost_des
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


extern uint8_t USART_Rx_data_handle[DATA_COUNT_RX];
extern uint8_t USART_Tx_buff[DATA_COUNT_TX]; 
extern USART_Tx_data_t  USART_TX_data;
void Transmit_data_send(USART_Tx_data_t *data);
void get_referr_data();
void reslove_referee_data();
void get_referr_data();
void reslove_referee_data();
void usart_data_updata(USART_Tx_data_t *data);



//extern USART_Rx_data_t USART_Rx_data;
//void Head1_data_Handle(uint8_t *buff,USART_Rx_data_t *data);
#endif