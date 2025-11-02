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
 
		for(;;)
		{
    get_referr_data();
    reslove_referee_data();
    usart_data_updata(&USART_TX_data);
    
		Transmit_data_send(&USART_TX_data);
    send_gimbal_mode_1_gimbal(&hcan2,SEND_ID_HEAD1,&small_gimbal_send);
    
    send_gimbal_mode_2_shoot(&hcan1,SHOOT_SEND_ID_HEAD2,&small_gimbal_send);
    
		  
      send_gimbal_mode_1_shoot(&hcan2,SHOOT_SEND_ID_HEAD1,&small_gimbal_send);
      
send_gimbal_mode_2_gimbal(&hcan1,SEND_ID_HEAD2,&small_gimbal_send);
      
      vTaskDelay(1);
      
		}
}

int p =0;

//}

void get_referr_data()
{
 if( robot_status.robot_id <= 9 && robot_status.robot_id >0 )
		  robot_data.robot_color = red ;
		 else if( robot_status.robot_id >= 101)
			robot_data.robot_color = blue ;
		 else
			robot_data.robot_color = NO_CONTACT ;
 
}

void reslove_referee_data()
{
  if(robot_data.robot_color==red)
  {
    if(game_robot_HP.red_outpost_HP<500)
    {
     USART_TX_data.if_outpost_des=1;
    }
    else USART_TX_data.if_outpost_des=0;
    
  }
  if(robot_data.robot_color==red)
  {
    if(game_robot_HP.blue_outpost_HP<500)
    {
     USART_TX_data.if_outpost_des=1;
    }
    else USART_TX_data.if_outpost_des=0;
    
  }
}
void usart_data_updata(USART_Tx_data_t *data)
{
  data->chassis_buff=power_heat_data.buffer_energy;
  data->chassis_mode=sentry_system.chassis_mode;
  
  if(chassis_com==com_nom)
  data->if_chassis_open=1;
  else data->if_chassis_open=0;
  data->chassis_vx=sentry_chassis.chassis_vx;
  data->chassis_vy=sentry_chassis.chassis_vy;
  data->up_yaw=INS.Yaw;
}
int gggg;
uint8_t TX_buff[24]={0};
void Transmit_data_send(USART_Tx_data_t *data)
{
  
   u8_to_float data_wz;
	u8_to_float data_vx;
	u8_to_float data_vy;
	u8_to_float data_yaw;
  u8_to_float data_ecd;
  u8_to_u16 data_buff; 
//  u8_to_u16 data_buff; 
  data_vx.data=data->chassis_vx;
  data_vy.data=data->chassis_vy;
  data_wz.data=data->chassis_wz;
  
  data_yaw.data   =power_heat_data.chassis_power;
  data_ecd.data   =yaw_motor.Position;
  data_buff.data=power_heat_data.buffer_energy;
 
  
  TX_buff[0]=0XA5;
  TX_buff[1]=data->if_chassis_open;
  TX_buff[2]=data->chassis_mode;
//  TX_buff[2]=no_move;
  TX_buff[3]=data_vx.d[0];
  TX_buff[4]=data_vx.d[1];
  TX_buff[5]=data_vx.d[2];
  TX_buff[6]=data_vx.d[3];
  TX_buff[7]=data_vy.d[0];
  TX_buff[8]=data_vy.d[1];
  TX_buff[9]=data_vy.d[2];
  TX_buff[10]=data_vy.d[3];
  TX_buff[11]=data_ecd.d[0];
  TX_buff[12]=data_ecd.d[1];
  TX_buff[13]=data_ecd.d[2];
  TX_buff[14]=data_ecd.d[3];
  TX_buff[15]=game_state.game_progress;
  TX_buff[16]=USART_TX_data.if_outpost_des;
  
  TX_buff[17]=data_yaw.d[0];
  TX_buff[18]=data_yaw.d[1];
  TX_buff[19]=data_yaw.d[2];
  TX_buff[20]=data_yaw.d[3];
  TX_buff[21]=data_buff.d[0];
  TX_buff[22]=data_buff.d[1];
  TX_buff[23]=0XAA;

  	gggg=HAL_UART_Transmit_DMA(&huart1,TX_buff,DATA_COUNT_TX);
}

void send_gimbal_mode_1_gimbal(CAN_HandleTypeDef *hcan,uint32_t id_range,small_gimbal_send_t *data)
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
  data->small_control_data.robot_color=robot_data.robot_color;
  data->small_control_data.shoot_mode=sentry_system.shoot_mode;
  data->small_control_data.vision_mode=sentry_system.vision_mode;
  
//  tx_data[0]=data->gimbal_mode;
//  tx_data[1]=data_yaw.d[0];
//  tx_data[2]=data_yaw.d[1];
//  tx_data[3]=data_pitch.d[0];
//  tx_data[4]=data_pitch.d[1];
//  tx_data[5]=data_wheel.d[0];
//  tx_data[6]=data_wheel.d[1];
//  tx_data[7]=data->if_pitch_can;

tx_data[0]=*((uint8_t *)&data->small_control_data);
  tx_data[1]=data_yaw.d[0];
  tx_data[2]=data_yaw.d[1];
  tx_data[3]=data_pitch.d[0];
  tx_data[4]=data_pitch.d[1];
  tx_data[5]=data_wheel.d[0];
  tx_data[6]=data_wheel.d[1];
  
  HAL_CAN_AddTxMessage( hcan, &tx_header,tx_data,(uint32_t*)CAN_TX_MAILBOX0);
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
  data->small_control_data.robot_color=robot_data.robot_color;
  data->small_control_data.shoot_mode=sentry_system.shoot_mode;
  data->small_control_data.vision_mode=sentry_system.vision_mode;
  
//  tx_data[0]=data->gimbal_mode;
//  tx_data[1]=data_yaw.d[0];
//  tx_data[2]=data_yaw.d[1];
//  tx_data[3]=data_pitch.d[0];
//  tx_data[4]=data_pitch.d[1];
//  tx_data[5]=data_wheel.d[0];
//  tx_data[6]=data_wheel.d[1];
//  tx_data[7]=data->if_pitch_can;

tx_data[0]=*((uint8_t *)&data->small_control_data);
  tx_data[1]=data_yaw.d[0];
  tx_data[2]=data_yaw.d[1];
  tx_data[3]=data_pitch.d[0];
  tx_data[4]=data_pitch.d[1];
  tx_data[5]=data_wheel.d[0];
  tx_data[6]=data_wheel.d[1];
  
  HAL_CAN_AddTxMessage( hcan, &tx_header,tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}



void send_gimbal_mode_1_shoot(CAN_HandleTypeDef *hcan,uint32_t id_range,small_gimbal_send_t*data)
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
  data_power.data=data->shooter_17mm_1_barrel_heat;
  
 
//  tx_data[0]=data->robot_color;
//  tx_data[1]=data_num.d[0];
//  tx_data[2]=data_num.d[1];
//  tx_data[3]=data_speed.d[0];
//  tx_data[4]=data_speed.d[1];
//  tx_data[5]=data_power.d[0];
//  tx_data[6]=data_power.d[1];
//  tx_data[7]=sentry_system.shoot_mode;

tx_data[0]=*((uint8_t *)&decision.Vision_ByteBits);
  tx_data[1]=data_num.d[0];
  tx_data[2]=data_num.d[1];
  tx_data[3]=data_speed.d[0];
  tx_data[4]=data_speed.d[1];
  tx_data[5]=data_power.d[0];
  tx_data[6]=data_power.d[1];
  
  HAL_CAN_AddTxMessage( hcan, &tx_header,tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}

//void send_gimbal_mode_2_gimbal(CAN_HandleTypeDef *hcan,uint32_t id_range,small_gimbal_send_t *data)
//{
// CAN_TxHeaderTypeDef tx_header;
//  uint8_t             tx_data[8];
//	
//	tx_header.StdId = id_range;
//  tx_header.IDE   = CAN_ID_STD;
//  tx_header.RTR   = CAN_RTR_DATA;
//  tx_header.DLC   = 8;
//  
//  
//  int16_to_8 data_yaw,data_pitch,data_wheel;
//  data_yaw.data=rc_ctrl.rc.ch2;
//  data_pitch.data=rc_ctrl.rc.ch3;
//  data_wheel.data=rc_ctrl.rc.wheel;
////  data->gimbal_mode=;

///**
//*@note small_control_data是打包数据，按照每个数据的占位拼成一个数据包
//*@note 这里是更新数据包的数据
//*/
//  data->small_control_data.gimbal_mode=sentry_system.small_gimbal_mode;
//  data->small_control_data.pitch_can=small_gimbal.pitch_can;
//  data->small_control_data.robot_color=robot_data.robot_color;
//  data->small_control_data.shoot_mode=sentry_system.shoot_mode;
//  data->small_control_data.vision_mode=sentry_system.vision_mode;
//  
////  tx_data[0]=data->gimbal_mode;
////  tx_data[1]=data_yaw.d[0];
////  tx_data[2]=data_yaw.d[1];
////  tx_data[3]=data_pitch.d[0];
////  tx_data[4]=data_pitch.d[1];
////  tx_data[5]=data_wheel.d[0];
////  tx_data[6]=data_wheel.d[1];
////  tx_data[7]=data->if_pitch_can;

//tx_data[0]=*((uint8_t *)&data->small_control_data);
//  tx_data[1]=data_yaw.d[0];
//  tx_data[2]=data_yaw.d[1];
//  tx_data[3]=data_pitch.d[0];
//  tx_data[4]=data_pitch.d[1];
//  tx_data[5]=data_wheel.d[0];
//  tx_data[6]=data_wheel.d[1];
//  
//  HAL_CAN_AddTxMessage( hcan, &tx_header,tx_data,(uint32_t*)CAN_TX_MAILBOX0);
//}






void send_gimbal_mode_2_shoot(CAN_HandleTypeDef *hcan,uint32_t id_range,small_gimbal_send_t *data)
{
 CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
	
	tx_header.StdId = id_range;
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = 8;
  
 u8_to_u16 data_num,data_speed,data_power;
  data_num.data=SHOOT_NUM_2;//后续考虑只将速度发过去，在小头端可以直接判断速度有没有变化而算出弹丸数量变换，这一位可以用来发送发弹标志位
  data_speed.data=speed_gun_2*10;//将小数转换成整数节省资源
  data_power.data=data->shooter_17mm_2_barrel_heat;
  
 
//  tx_data[0]=data->robot_color;
//  tx_data[1]=data_num.d[0];
//  tx_data[2]=data_num.d[1];
//  tx_data[3]=data_speed.d[0];
//  tx_data[4]=data_speed.d[1];
//  tx_data[5]=data_power.d[0];
//  tx_data[6]=data_power.d[1];
//  tx_data[7]=sentry_system.shoot_mode;

tx_data[0]=*((uint8_t *)&decision.Vision_ByteBits);
  tx_data[1]=data_num.d[0];
  tx_data[2]=data_num.d[1];
  tx_data[3]=data_speed.d[0];
  tx_data[4]=data_speed.d[1];
  tx_data[5]=data_power.d[0];
  tx_data[6]=data_power.d[1];
  
  HAL_CAN_AddTxMessage( hcan, &tx_header,tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}
//void send_gimbal_mode_2_shoot(CAN_HandleTypeDef *hcan,uint32_t id_range,small_gimbal_send_t*data)
//{
// CAN_TxHeaderTypeDef tx_header;
//  uint8_t             tx_data[8];
//	
//	tx_header.StdId = id_range;
//  tx_header.IDE   = CAN_ID_STD;
//  tx_header.RTR   = CAN_RTR_DATA;
//  tx_header.DLC   = 8;
//  
// u8_to_u16 data_num,data_speed,data_power;
//  data_num.data=data->shoot_num_2;//后续考虑只将速度发过去，在小头端可以直接判断速度有没有变化而算出弹丸数量变换，这一位可以用来发送发弹标志位
//  data_speed.data=speed_gun_2*10;//将小数转换成整数节省资源
//  data_power.data=data->shooter_17mm_1_barrel_heat;
//  
// 
////  tx_data[0]=data->robot_color;
////  tx_data[1]=data_num.d[0];
////  tx_data[2]=data_num.d[1];
////  tx_data[3]=data_speed.d[0];
////  tx_data[4]=data_speed.d[1];
////  tx_data[5]=data_power.d[0];
////  tx_data[6]=data_power.d[1];
////  tx_data[7]=sentry_system.shoot_mode;

//tx_data[0]=*((uint8_t *)&decision.Vision_ByteBits);
//  tx_data[1]=data_num.d[0];
//  tx_data[2]=data_num.d[1];
//  tx_data[3]=data_speed.d[0];
//  tx_data[4]=data_speed.d[1];
//  tx_data[5]=data_power.d[0];
//  tx_data[6]=data_power.d[1];
//  HAL_CAN_AddTxMessage( hcan, &tx_header,tx_data,(uint32_t*)CAN_TX_MAILBOX0);
//}