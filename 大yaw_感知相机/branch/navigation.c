


/*导航信息处理*/

//#include "Odometer.h"
#include "ins_task.h"
#include "navigation.h"
#include <stdlib.h>
#include "usbd_cdc_if.h"
#include "usb_device.h"
//#include "bsp_transmit.h"
#include "decision.h"
#include "bsp_transmit.h"
#include "referee.h"
#include "decision.h"
location_t location=
{
  .Sx=0,
  .Sy=0,
  .Sx_set=0,
  .Sy_set=0,
  .yaw_update=0,
};

//如果红色开始建图
/*  
1如果车体是红色的，那么发过去的坐标都不需要转换
2如果车体是蓝色的，那么发过去的坐标需要经过一个坐标系转换后才可以发过去
3接收到云台手的坐标也是对的，不需要经过任何转换
*/
/*如果蓝色开始建图
1如果车体是红色的，那么发过去的坐标需要转换、
2如果车体是蓝色的，那么发过去的需要转换
3云台手的标点需要经过完整的坐标系转换才能正常
*/


navigation_rx_t navigation_rx;
navigation_tx_t navigation_tx;

uint8_t TX_Buff[navigation_tx_len];
int navi_tx_count;
int last_seq;
uint8_t last_point_get;
uint16_t opopok;
uint8_t navi_state_get;
//uint8_t buff_const;
//uint8_t buff_head;
uint8_t buff_count[100];
int i_count=0;
int olklk;
//uint8_t rx_data_navi[24]={0};
void navigation_rx_handle(uint8_t *buff,uint32_t Len,navigation_rx_t *data)
{



//  for(int i=0;i<23;i++)
//  {
//    rx_data_navi[i]=buff[i];
//  }

  last_seq++;
//  buff_head=buff[0];

//  buff_const=Len;
   if (buff == NULL)
    {
        return ;
    }
  if(buff[0]==CONST_HEAD0&&buff[Len-1]==CONST_END0)
  {
  
  navi_tx_count=0;

      u8_to_float Vx,Vy,Vz,Sx,Sy;
      
      for(int i=0;i<4;i++)
      {
      
      
        Vx.d[i]=buff[i+1];
        Vy.d[i]=buff[i+5];
        Vz.d[i]=buff[i+9];
        Sx.d[i]=buff[i+13];
        Sy.d[i]=buff[i+17];
        
        
      }
      
       navi_tx_count=0;

      data->If_get_path=buff[Len-3];

      
      data->navi_vx=Vx.data;
      data->navi_vy=Vy.data;
      data->navi_wz=Vz.data;
      data->current_x=Sx.data;
      data->current_y=Sy.data;

      
      
  }
  
}

uint8_t last_point;
uint8_t clear_count;
USBD_StatusTypeDef IOIOL;
void Navigation_Tx_Send(navigation_tx_t *data)
{
  data->current_yaw=INS.Yaw;
  data->current_pitch=INS.Roll;
  data->current_roll=INS.Pitch;

  data->Odom_Vx=0;
  data->Odom_Vy=0;
  data->yaw_Wz=INS.Gyro[2];
  
  
  TX_Buff[0]=CONST_HEAD0;
  memcpy(TX_Buff+1,&data->navi_set_x_pos ,4);
  memcpy(TX_Buff+5,&data->navi_set_y_pos ,4);
  memcpy(TX_Buff+9,&data->current_yaw,4);
  memcpy(TX_Buff+13,&data->current_pitch,4);  
  
  TX_Buff[navigation_tx_len-1]=CONST_END0;
  
  
  IOIOL=CDC_Transmit_FS(TX_Buff,navigation_tx_len);
}


 void Serial_Data_Handle()
 {
   
 }