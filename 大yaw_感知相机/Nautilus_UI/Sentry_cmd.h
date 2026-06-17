/**
  ************************************* Copyright ******************************   
    * FileName   : Sentry_cmd.h   
  * Version    : v1.0		
  * Author     : 王子佩			
  * Date       : 2024-05-12         
  * Description:    
  * Function List:  
  	1. ....
  	   <version>: 		
  <modify staff>:
  		  <data>:
   <description>:  
  	2. ...
  ******************************************************************************
 */



#ifndef __SENTRY_CMD_H_
#define __SENTRY_CMD_H_


#include "Nautilus_UI.h"
#include "decision.h"
#include "referee.h"

typedef struct 
{
	uint8_t IF_Revive;
	uint8_t IF_Immediately_Revive;
	uint16_t Exchange_Projectile_Num;
	uint8_t IF_Remote_Exchange_HP;
	
}Sentry_cmd_Fill_t;


typedef __packed struct
{
	UI_Packhead_t UI_Packhead;
	// cmd_id
	map_data_t data;
	// 内容数据段
	uint16_t frame_tail;
} Path_Data_t;

#define CONST__PATH_SEND_CMD_ID        0x0307

#define Map_Data_byte    114 //9+105
typedef struct
{
    /* 控制模式 */
//    uint8_t point_mode;      // 点位模式
//    toward_mode_e toward;    // 朝向模式
//    move_mode_e move;        // 移动模式
//    cruise_mode_e cruise;    // 巡航模式

//    /* 目标选择 */
//    ext_target_switch_t IF_Strike;        // 击打目标
//    ext_target_switch_t Follow_target;    // 跟随目标

//    /* 云台巡航 */
//    float yaw_mid_obj;       // yaw轴循环中值

    /* 机器人目标位置 */
    robot_pos_t robot_pos_set;            // 机器人目标位置
    /* 机器人路径 */
    Path_Data_t path_data;   // 机器人路径信息
	
}Map_data_t;


extern Sentry_cmd_Fill_t Sentry_cmd_Fill;
extern Sentry_cmd_t Sentry_cmd;
extern Map_data_t Map_data;
//void Sentry_cmd_Ctl( Sentry_cmd_Fill_t * Sentry_cmd_Fill,Decision_t * decision);

#endif


 

