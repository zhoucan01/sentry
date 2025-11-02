#ifndef __SYSTEM_H
#define __SYSTEM_H


#include "CAN_receive.h"
#include "Nautilus_Vision.h"
extern int oooiii;
typedef enum
{
	com_err=0,
	com_nom,
}com_mode_t;


typedef enum
{
 vision_off=0,
 vision_on,
}vision_mode_t;
typedef struct
{
 float gimbal_yaw_in;
 float gimbal_pitch_in;
 gimbal_mode_t gimbal_mode;
 com_mode_t last_control_com;
 com_mode_t control_com;
 vision_mode_t vision_mode;
}gimbal_system_t;

void get_control_mode(control_data_t *mode);


typedef enum 
{
	ZERO_rc = 0,
	UP_LONG_rc = 1,
	DOWN_LONG_rc  = 2,
	UP_SHORT_rc = 3,
	DOWN_SHORT_rc,
}Wheel_State_t;
void WHEEL_STATE_Ctrl(void);

extern Wheel_State_t Wheel_State;



void communicate_pin_state(control_data_t *mode);




#include "struct_typedef.h"

#include "stdbool.h"

#define DETECT_TASK_INIT_TIME 57
#define DETECT_CONTROL_TIME 5

#define COMMUNICATION_MORMAL   0
#define COMMUNICATION_NONE     1

#define USE_DETECT


#define DBUS_MAX_OFFLINE_FRAME_RATE               40
#define CHASSIS_MOTOR_1_MAX_OFFLINE_FRAME_RATE    10
#define CHASSIS_MOTOR_2_MAX_OFFLINE_FRAME_RATE    10
#define CHASSIS_MOTOR_3_MAX_OFFLINE_FRAME_RATE    10
#define CHASSIS_MOTOR_4_MAX_OFFLINE_FRAME_RATE    10
#define FRIC_R_MAX_OFFLINE_FRAME_RATE             30
#define FRIC_L_MAX_OFFLINE_FRAME_RATE             80
#define TRIG_MAX_OFFLINE_FRAME_RATE               10
#define YAW_MAX_OFFLINE_FRAME_RATE                50
#define PITCH_MAX_OFFLINE_FRAME_RATE              50
#define RC_RECIVE_MAX_OFFLINE_FRAME_RATE          80
#define BOARD_MAX_OFFLINE_FRAME_RATE              90
#define VISION_MAX_OFFLINE_FRAME_RATE             150

//设备列表
enum TOEList
{
  
  
  FRICR_MOTOR_R_TOE,
  FRICR_MOTOR_L_TOE,
  TRIGGER_MOTOR_TOE,
  YAW_GIMBAL_MOTOR_TOE,
  PITCH_GIMBAL_MOTOR_TOE,
	RC_RECIVE_TOE,
	BOARD_TOE,
  VISION_TOE,
  
  
  ERROR_LIST_LENGHT,
};



typedef struct 
{
  /* data */
  uint32_t offline_frame_rate;                  //设备失联帧率
  uint32_t max_offline_frame_rate;              //设备最大失联帧率
  uint8_t communication_state;                  //通信状态
  bool_t (*toe_offline_data_handle_f)(void);    //设备失联数据处理
  void (*toe_unable_f)(void);                      //设备失联后失能
  void (*toe_connect_soft_restart_f)(void);          //设备失联后重新使能
}toe_offline_t;

//extern toe_offline_t toe_offline[ERROR_LIST_LENGHT] = {0};


extern gimbal_system_t gimbal_system;
/**
  * @brief          get toe error status
  * @param[in]      toe: table of equipment
  * @retval         true (eror) or false (no error)
  */
bool toe_is_error(uint8_t err);

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
void communication_frame_rate_update(uint8_t toe);
void get_control_mode(control_data_t *mode);
void DETECT_task(void const *argument);
void vision_reset(TJ_Vision_Rx_t *data);
#endif