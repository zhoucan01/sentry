/*
 *                        .::::.
 *                      .::::::::.
 *                     :::::::::::
 *                  ..:::::::::::'
 *               '::::::::::::'
 *                 .::::::::::
 *            '::::::::::::::..
 *                 ..::::::::::::.
 *               ``::::::::::::::::
 *                ::::``:::::::::'        .:::.
 *               ::::'   ':::::'       .::::::::.
 *             .::::'      ::::     .:::::::'::::.
 *            .:::'       :::::  .:::::::::' ':::::.
 *           .::'        :::::.:::::::::'      ':::::.
 *          .::'         ::::::::::::::'         ``::::.
 *      ...:::           ::::::::::::'              ``::.
 *     ````':.          ':::::::::'                  ::::..
 *                        '.:::::'                    ':'````..
 */

/*
使用：使用者只需在头文件的设备列表中添加对应的设备，在函数detect_init中添加对应的数据，在需要使用的设备中使用函数communication_frame_rate_update
更新通信帧率即可

注意：此代码不兼容while1死循环，函数detect_init中的max_offline_frame_rate为最大失联帧率，需要自己测试
*/





#include "detect_task.h"
#include "cmsis_os.h"

#include "stdio.h"

/**
  * @brief          init
  * @param[in]      none
  * @retval         none
  */
static void detect_init(void);


toe_offline_t toe_offline[ERROR_LIST_LENGHT] = {0};


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
          DBUS_MAX_OFFLINE_FRAME_RATE,
          CHASSIS_MOTOR_1_MAX_OFFLINE_FRAME_RATE,
          CHASSIS_MOTOR_2_MAX_OFFLINE_FRAME_RATE, // chassis_motor2
          CHASSIS_MOTOR_3_MAX_OFFLINE_FRAME_RATE, // chassis_motor3
          CHASSIS_MOTOR_4_MAX_OFFLINE_FRAME_RATE, // chassis_motor4
          FRIC_R_MAX_OFFLINE_FRAME_RATE,          // fric_r
          FRIC_L_MAX_OFFLINE_FRAME_RATE,          // fric_l
          TRIG_MAX_OFFLINE_FRAME_RATE,            // trig
          YAW_MAX_OFFLINE_FRAME_RATE,             // yaw
          PITCH_MAX_OFFLINE_FRAME_RATE,           // pitch
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


