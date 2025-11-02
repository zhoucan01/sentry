#include "bsp_transmit.h"
#include "cmsis_os.h"
#include "system.h"
#include "struct_typedef.h"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "ins_task.h"
#include "CAN_receive.h"
#include "referee.h"
#include "string.h"
#include "Odometer.h"
//#include "shoot.h"
uint8_t USART_Rx_data_handle[DATA_COUNT_RX];


USART_Rx_data_t  USART_Rx_data;

uint8_t transmit_seq;
int hhhhhh;
void RX_USART_data_Handle(uint8_t *buff,USART_Rx_data_t *data)
{
u8_to_float vx,vy,ecd;
u8_to_u16 buff_energy;

u8_to_int16 up_yaw,chassis_Power_limit;

  if(buff == NULL)
  {
  
   return;
  }
  if(buff[0]==USART_RX_HAED&&buff[DATA_COUNT-1]==USART_RX_END)
  {
   data->if_chassis_open=buff[1];
   data->chassis_mode=buff[2];
   for(int i=0;i<4;i++)
   { 
   
     vx.d[i]=buff[i+3];
     vy.d[i]=buff[i+7];
     ecd.d[i]=buff[i+11];
   }
   for(int i=0;i<2;i++)
   {
//     up_yaw.d[i]=buff[i+17];
     chassis_Power_limit.d[i]=buff[i+19];
     buff_energy.d[i]=buff[i+21];
   }
   
   
   data->if_arrived=buff[15];
   
   data->Vz_state=buff[16];

   data->position_x=buff[17];
   data->position_y=buff[18];
   transmit_seq++;

   data->trans_seq=buff[23];
   data->chassis_vx=-vx.data*4;
   data->chassis_vy=-vy.data*4;
   data->big_yaw_ecd=ecd.data;
   data->up_yaw=(float)up_yaw.data/100;
   data->chassis_Power_limit=chassis_Power_limit.data;
   data->chassis_buff=buff_energy.data;
   data->current_x= (float)data->position_x/10;
   data->current_y= (float)data->position_y/10;   
   data->yaw_diff=up_yaw.data-0;
   
   data->ecd_diff=data->yaw_diff*2*3.14/360;
   
   location.relative_init_angle=data->big_yaw_ecd-data->ecd_diff;
   
  }
}

HAL_StatusTypeDef ioioj;
uint8_t Tx_Buff[DATA_Tx_count];
USART_Tx_data_t USART_Tx_data;
int jjjj;
void USART_TX_send(uint8_t *buff,USART_Tx_data_t *data)
{
data->odom_Sx=location.Sx;
data->odom_Sy=location.Sy;
  data->odom_Vx++;
  
  Tx_Buff[0]=USART_TX_HAED;
  memcpy(Tx_Buff+1,&data->odom_Sx,4);
  memcpy(Tx_Buff+5,&data->odom_Sy,4);
  memcpy(Tx_Buff+9,&data->odom_Vx,4);
  memcpy(Tx_Buff+13,&data->odom_Vy,4);
  Tx_Buff[DATA_Tx_count-1]=USART_RX_END;
  jjjj++;
 ioioj= HAL_UART_Transmit_DMA(&huart1,Tx_Buff,DATA_Tx_count);
}






