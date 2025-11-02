#include "bsp_transmit.h"
#include "cmsis_os.h"
#include "system.h"
#include "struct_typedef.h"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "ins_task.h"
#include "CAN_receive.h"
#include "referee.h"
#include  "chassis_control.h"
#include "remote_control.h"
#include "small_gimbal.h"
#include "can.h"
#include "gimbal_big_yaw.h"
#include "navigation.h"
//#include "shoot.h"
uint8_t USART_Rx_data_handle[DATA_COUNT_RX];
uint8_t USART_Tx_buff[DATA_COUNT_TX] = {0};
small_gimbal_send_t small_gimbal_send;
//USART_Rx_data_t USART_Rx_data;
USART_Tx_data_t  USART_TX_data=
{
.if_chassis_open=0,
.chassis_mode=0,
.chassis_vx=0,
.chassis_vy=0,
.chassis_wz=0,
.up_yaw=0,
};




void Transmit_run()
{
    
    vTaskDelay(20);
    
		for(;;)
		{

    usart_data_updata(&USART_TX_data);
    
		Transmit_data_send(&USART_TX_data);

       send_gimbal_mode_2_shoot(&hcan1,SHOOT_SEND_ID_HEAD2,&small_gimbal_send);
      vTaskDelay(2);
    send_gimbal_mode_2_gimbal(&hcan1,SEND_ID_HEAD2,&small_gimbal_send);
      
      vTaskDelay(2);
    
      
		}
}

int p =0;

//}




void usart_data_updata(USART_Tx_data_t *data)
{
  data->chassis_buff=power_heat_data.buffer_energy;
  data->chassis_mode=sentry_system.chassis_mode;
  
  if(chassis_com==com_nom)
  data->if_chassis_open=1;
  else data->if_chassis_open=0;
  data->chassis_vx=sentry_chassis.chassis_vx;
  data->chassis_vy=sentry_chassis.chassis_vy;
  data->up_yaw=INS.Yaw*100;
  data->game_progress=game_state.game_progress;
  data->If_chassis_weak=decision.Judge_condition.If_chassis_weak;
  data->chassis_power_limit=robot_status.chassis_power_limit;
//  data->position_x=(uint8_t)((navigation_rx.current_x+4.2)*10);
//  data->position_y=(navigation_rx.current_y+7.5)*10;
  data->position_x=0;
  data->position_y=0;
}
int gggg;
uint8_t TX_buff[DATA_COUNT]={0};
uint16_t last_remain_time;
void Transmit_data_send(USART_Tx_data_t *data)
{
  


  u8_to_u16 data_buff; 
  
  
  

  data_buff.data=power_heat_data.buffer_energy;
  data->trans_seq++;
  
  TX_buff[0]=0XA5;
  TX_buff[1]=data->if_chassis_open;
  TX_buff[2]=data->chassis_mode;
  
  //在导航模式下这个速度很重要，导航发送过来的速度乘的系数必须是一致的，不能经过不同的系数，不然会出现走不直
  memcpy(TX_buff+3,&data->chassis_vx,4);
  memcpy(TX_buff+7,&data->chassis_vy,4);
  
  memcpy(TX_buff+11,&yaw_motor.Position,4);
  TX_buff[15]=decision.Judge_condition.IF_Arrived;
  TX_buff[16]=sentry_system.chassis_set.Vz_state;
  memcpy(TX_buff+17,&data->position_x,1);
  memcpy(TX_buff+18,&data->position_y,1);

  memcpy(TX_buff+19,&data->chassis_power_limit,2);
  memcpy(TX_buff+21,&data_buff,2);
  TX_buff[23]=data->trans_seq;
  TX_buff[24]=0XAA;

  	HAL_UART_Transmit_DMA(&huart1,TX_buff,DATA_COUNT_TX);
}



void send_gimbal_mode_2_gimbal(CAN_HandleTypeDef *hcan,uint32_t id_range,small_gimbal_send_t *data)
{
 CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
	
	tx_header.StdId = id_range;
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = 8;
  
  
  int16_to_8 data_yaw,data_pitch,data_wheel;
  data_yaw.data=rc_ctrl.rc.ch2;
  data_pitch.data=rc_ctrl.rc.ch3;
  data_wheel.data=rc_ctrl.rc.wheel;
//  data->gimbal_mode=;

/**
*@note small_control_data是打包数据，按照每个数据的占位拼成一个数据包
*@note 这里是更新数据包的数据
*/
  data->small_control_data.gimbal_mode=sentry_system.small_gimbal_mode;
  data->small_control_data.pitch_can=small_gimbal.pitch_can;
  data->small_control_data.robot_color=decision.robot_data.robot_color;
  data->small_control_data.shoot_mode=sentry_system.shoot_mode;
  data->small_control_data.vision_mode=sentry_system.vision_mode;
  data->small_control_data.trig_mode=sentry_system.trig_mode;
  
  data->other_data.outpost_state=decision.Judge_condition.If_enemy_outpost_lock;
  data->other_data.wheel_state=Wheel_State;
  data->other_data.if_arrived=decision.Judge_condition.IF_Arrived;//


  tx_data[0]=*((uint8_t *)&data->small_control_data);
  tx_data[1]=data_yaw.d[0];
  tx_data[2]=data_yaw.d[1];
  tx_data[3]=data_pitch.d[0];
  tx_data[4]=data_pitch.d[1];
  tx_data[5]=*((uint8_t *)&data->other_data);
  
  tx_data[6]=decision.top_senior_priority;
  tx_data[7]=0;
  HAL_CAN_AddTxMessage( hcan, &tx_header,tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}





void send_gimbal_mode_2_shoot(CAN_HandleTypeDef *hcan,uint32_t id_range,small_gimbal_send_t *data)
{
 CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
	
	tx_header.StdId = id_range;
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = 8;
  
 u8_to_u16 data_num,data_speed,data_power;
  data_num.data=SHOOT_NUM_1;//后续考虑只将速度发过去，在小头端可以直接判断速度有没有变化而算出弹丸数量变换，这一位可以用来发送发弹标志位
  data_speed.data=speed_gun_1*10;//将小数转换成整数节省资源
  data_power.data=power_heat_data.shooter_17mm_1_barrel_heat;
   
   
   if(decision.Judge_condition.IF_Arrived==1)
   {
       if(gimbal.curise_direction==0)
     {
       data->curise_mode=curise_stop;
     }  
     else if(gimbal.curise_direction==-1)
     { 
       data->curise_mode=curise_minus;
     }
     else if(gimbal.curise_direction==1)
     {
       data->curise_mode=curise_add;
     }
   }
   else data->curise_mode=curise_stop;
   


  tx_data[0]=*((uint8_t *)&decision.Vision_ByteBits);
  tx_data[1]=data_num.d[0];
  tx_data[2]=data_num.d[1];
  tx_data[3]=data_speed.d[0];
  tx_data[4]=data_speed.d[1];
  tx_data[5]=data_power.d[0];
  tx_data[6]=data_power.d[1];
  tx_data[7]=data->curise_mode;
  HAL_CAN_AddTxMessage( hcan, &tx_header,tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}
//uint8_t USART_Rx_data_handle[DATA_COUNT_RX];
USART_Rx_data_t USART_Rx_data;
uint8_t lllll;
void RX_USART_data_Handle(uint8_t *buff,USART_Rx_data_t *data)
{
  u8_to_float Sx,Sy,Vx,Vy;

  lllll++;
  if(buff[0]==USART_RX_HAED&&buff[DATA_RX_count-1]==USART_RX_END)
  {
  
  
    for(int i=0;i<4;i++)
    {
      Sx.d[i]=buff[1+i];
      Sy.d[i]=buff[5+i];
      Vx.d[i]=buff[9+i];
      Vy.d[i]=buff[13+i];
    }
    
    
    data->odom_Sx=Sx.data;
    data->odom_Sy=Sy.data;
    data->odom_Vx=Vx.data;
    data->odom_Vy=Vy.data;
  }
  
}
