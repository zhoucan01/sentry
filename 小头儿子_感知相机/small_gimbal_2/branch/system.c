#include "system.h"
#include "CAN_receive.h"
#include "ins_task.h"
#include "cmsis_os.h"
#include "struct_typedef.h"
#include "stm32f4xx_hal.h"
#include "remote_control.h"
#include "Nautilus_Vision.h"
#include "bsp_dwt.h"
gimbal_system_t gimbal_system;

/**
*@note读取主控控制信息
*@note因为主控写了模式处理，下位机不用再继续进行模式判断,只判断是否通信离线
*@note加上检测通信（detect_test）是否离线，离线则直接无法控制，电流全发0
*@note
*/
Wheel_State_t Wheel_State;
toe_offline_t toe_offline[ERROR_LIST_LENGHT] = {0};
//float diff_time;
//uint32_t time;
void system_run()
{
 
 for(;;)
 {
  
  
  get_control_mode(&control_data);
  Tj_Send_Data(&TJ_Vision_Tx);
  communicate_pin_state(&control_data);
 
  vision_reset(&TJ_Vision_Rx);
        vTaskDelay(1);  
 }
}
/**
*@note检测是否通信离线，离线则全部下电进入保护状态
*/
int oooiii;
void get_control_mode(control_data_t *mode)
{
  if(toe_offline[BOARD_TOE].communication_state==COMMUNICATION_NONE)
  {
  oooiii++;
   gimbal_system.control_com=com_err;
   com_data_reset(&control_data);
   
  }
  else gimbal_system.control_com=com_nom;
  
  
  gimbal_system.last_control_com=gimbal_system.control_com;
}
int ooooo;
int ppppp;
void communicate_pin_state(control_data_t *mode)
{
  if(gimbal_system.control_com==com_nom)
  {
  
  ooooo++;
     HAL_GPIO_WritePin(GPIOH,GPIO_PIN_10,GPIO_PIN_SET);;
  }
  else if(gimbal_system.control_com==com_err)
  {
  ppppp++;
    HAL_GPIO_WritePin(GPIOH,GPIO_PIN_10,GPIO_PIN_RESET);
  }
}
int oooool;
//void vision_reset(Vision_rx_Data_t *data)
//{
//  if(toe_offline[VISION_TOE].communication_state==COMMUNICATION_NONE)
//  {
//    oooool++;
//    data->Flag_Found=0;
//    data->IF_stay=0;
//    data->fire=0;
//    data->Fire_Mode=0;
//    data->yaw_obj=INS.YawTotalAngle;
//    data->pitch_obj=INS.Roll;
//    
//  }
//  else 
//  {
//   NULL;
//  }
//}

void vision_reset(TJ_Vision_Rx_t *data)
{
  if(toe_offline[VISION_TOE].communication_state==COMMUNICATION_NONE)
  {
    memset(&data,0,sizeof(data));
  }
  else 
  {
   NULL;
  }
}
void com_data_reset(control_data_t *mode)
{ 
  mode->data_wheel=0;
  mode->gimbal_mode= small_gimbal_off;
  mode->gimbal_pitch_in=0;
  mode->gimbal_yaw_in=0;
  mode->if_gimbal_can=0;
  mode->robot_color=NO_CONTACT;
  mode->shoot_mode=shoot_no;
  mode->other_data.wheel_state=0;
  
}




#include "cmsis_os.h"

#include "stdio.h"

/**
  * @brief          init
  * @param[in]      none
  * @retval         none
  */
static void detect_init(void);





#if INCLUDE_uxTaskGetStackHighWaterMark       
	uint32_t detect_task_stack;
#endif



/**
  * @brief          init
  * @param[in]      none
  * @retval         none
  */
static void detect_init(void)
{
  uint8_t i = 0;
  uint16_t max_offline_frame_rate[ERROR_LIST_LENGHT] =
      {
          
          FRIC_R_MAX_OFFLINE_FRAME_RATE,          // fric_r
          FRIC_L_MAX_OFFLINE_FRAME_RATE,          // fric_l
          TRIG_MAX_OFFLINE_FRAME_RATE,            // trig
          YAW_MAX_OFFLINE_FRAME_RATE,             // yaw
          PITCH_MAX_OFFLINE_FRAME_RATE,           // pitch
				  RC_RECIVE_MAX_OFFLINE_FRAME_RATE,
          
          VISION_MAX_OFFLINE_FRAME_RATE,
				  BOARD_MAX_OFFLINE_FRAME_RATE
          
      };
        
  for (i = 0; i < ERROR_LIST_LENGHT; i++)
  {
    toe_offline[i].offline_frame_rate = 0;
    toe_offline[i].max_offline_frame_rate = max_offline_frame_rate[i];
    toe_offline[i].communication_state = COMMUNICATION_MORMAL;
    toe_offline[i].toe_offline_data_handle_f = NULL;
    toe_offline[i].toe_unable_f = NULL;
    toe_offline[i].toe_connect_soft_restart_f = NULL;
  }

  // toe_offline[DBUS_TOE].toe_offline_data_handle_f = RC_data_is_error;
  // toe_offline[DBUS_TOE].toe_unable_f = slove_RC_lost;
  // toe_offline[DBUS_TOE].toe_connect_soft_restart_f = slove_data_error;
}




/**
  * @brief          detect task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */

void DETECT_task(void const *argument)
{
	detect_init();
	while(1)
	{
  
//  WHEEL_STATE_Ctrl();
  uint8_t i = 0;
  for (i = 0; i < ERROR_LIST_LENGHT; i++)
  {
    toe_offline[i].offline_frame_rate++;
    if (toe_offline[i].offline_frame_rate > toe_offline[i].max_offline_frame_rate)
    {
      toe_offline[i].communication_state = COMMUNICATION_NONE;
      toe_offline[i].offline_frame_rate = toe_offline[i].max_offline_frame_rate;
      if (toe_offline[i].toe_offline_data_handle_f != NULL)
      {
        toe_offline[i].toe_offline_data_handle_f();
      }
    }
    else
    {
      toe_offline[i].communication_state = COMMUNICATION_MORMAL;
    }
  }
  vTaskDelay(DETECT_CONTROL_TIME);
  #if INCLUDE_uxTaskGetStackHighWaterMark
    detect_task_stack = uxTaskGetStackHighWaterMark(NULL);
  #endif
}
}


/**
  * @brief          get toe error status
  * @param[in]      toe: table of equipment
  * @retval         true (eror) or false (no error)
  */
bool toe_is_error(uint8_t toe)
{
  if (COMMUNICATION_NONE == toe_offline[toe].communication_state)
  {
    return true;
  }
  else
  {
    return false;
  }
}

//toe_is_error(DBUS_TOE);

/**
  * @brief          communication_frame_rate_update
  * @param[in]      toe: table of equipment
  * @retval         none
  */
/**
  * @brief          通信帧率更新
  * @param[in]      toe:设备序号
  * @retval         none
  */
void communication_frame_rate_update(uint8_t toe)
{
  toe_offline[toe].offline_frame_rate = 0;
}




int WHEEL_UP_TIME;
int WHEEL_DOWN_TIME;
int WHEEL_LAST;
int pppppppp;
int receive_data;
void WHEEL_STATE_Ctrl(void)
{

receive_data=control_data.data_wheel;
	if ( receive_data<-600 )
	{
		WHEEL_UP_TIME ++;
	}
	else if ( receive_data>600)
	{
		WHEEL_DOWN_TIME ++;
	}
	
	//判断为非动作位
	if ( receive_data>=-600 && receive_data<=600 && WHEEL_LAST>=-600 && WHEEL_LAST<=600 )
	{
		Wheel_State = ZERO_rc;
		WHEEL_DOWN_TIME = 0;
		WHEEL_UP_TIME = 0;
	}
	
	//UP--松开时进行判断SHORT or LONG
	if ( receive_data>=-600 && receive_data<=600 && WHEEL_LAST<-600 && WHEEL_UP_TIME<30 )
		Wheel_State = UP_SHORT_rc;
	else if(receive_data<-600 && WHEEL_UP_TIME>=20 )
		Wheel_State = UP_LONG_rc;
	
	//DOWN--松开时进行判断SHORT or LONG
	if ( receive_data>=-600 && receive_data<=600 && WHEEL_LAST>600 && WHEEL_DOWN_TIME<300 )
  {
  Wheel_State = DOWN_SHORT_rc;
  pppppppp++;
  }
		
	else if( receive_data>600 && WHEEL_DOWN_TIME>=300 )
		Wheel_State = DOWN_LONG_rc;
	
	WHEEL_LAST = receive_data;
}