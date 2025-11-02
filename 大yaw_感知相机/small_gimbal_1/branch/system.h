#ifndef __SYSTEM_H
#define __SYSTEM_H

#include "struct_typedef.h"
#include "stdbool.h"
typedef enum
{
no_move=0,//底盘不动
follow_move,//底盘舵机跟随头动
spine_move,//陀螺模式
lock_move,//底盘跟随，底盘与大yaw夹角固定

}chassis_mode_t;
#define WHEEL_PERIMETER 			 376.99f	//车轮周长 (轮子直径 * PI 再转换成mm)
#define M3508_RATIO 	 				 19.2032				//电机减速比
#define Radius 								 62				//轮径mm  半径
#define distance_x             180/1000      //舵轮中心到底盘中心的距离
#define distance_y             180/1000     //


#define WHEEL_FACTOR 0.00034   // 电机转速单位rpm转换为真实速度m/s 的常数

#define big_yaw_kp 0.0002
typedef enum
{
shoot_no=0,
shoot_on,//手打

}shoot_mode_t;

typedef struct
{
 float set_vx;
  float set_vy;
  float set_wz;

}chassis_set_t;
typedef enum
{
 vision_off=0,
 vision_on,
}vision_mode_t;


typedef enum
{
  rc_mode=0,
  auto_mode,
}control_mode_t;
//typedef enum
//{
//  NO_CONTACT	 = 0,   
//    RED          = 1,   
//    BLUE	       = 2,   
//}robot_color_t;

typedef enum
{
  small_gimbal_off=0,
  small_gimbal_rc,
  small_gimbal_pc,
  
}small_gimbal_mode_t;
typedef enum
{
	com_err=0,
	com_nom,
}com_mode_t;
typedef struct
{
  float set_yaw_in;
  float speed_kp;
  
  chassis_set_t chassis_set;
  shoot_mode_t shoot_mode;
  vision_mode_t vision_mode;
  chassis_mode_t chassis_mode; 
 control_mode_t control_mode;
 small_gimbal_mode_t small_gimbal_mode;
 
 int if_small_pitch_can;
}sentry_system_t;

typedef __packed struct 
{  
	uint8_t shoot_all      : 1;  
	uint8_t shoot_Hero     : 1;  
	uint8_t shoot_Engineer : 1;  
	uint8_t shoot_infantr3  : 1;  
	uint8_t shoot_infantr4 : 1;  
	uint8_t shoot_Sentry   : 1;  
	uint8_t shoot_outpost  : 1; 
	uint8_t shoot_base     : 1;  

}Vision_ByteBits_t;


//typedef __packed struct
//{
// uint8_t gimbal_mode :2;
// uint8_t pitch_can  :1;
// uint8_t shoot_mode :1;
// uint8_t vision_mode :1;
// uint8_t robot_color :2;
//}small_control_data;

typedef struct
{
  Vision_ByteBits_t Vision_ByteBits;
}decision_t;

extern decision_t decision;
typedef enum
{
  
  NO_CONTACT=0,
  red,
  blue,
  
}robot_color_t ;
typedef struct
{
 robot_color_t robot_color; 
}robot_data_t;
void AGV_big_yaw(sentry_system_t *mode);
void AGV_mode_chose(sentry_system_t *mode);
void remote_offline_set(sentry_system_t *mode);
void shoot_mode_chose(sentry_system_t *mode);

void choose_control_mode(sentry_system_t *mode);
void auto_mode_set(sentry_system_t *mode);
//void chassis_move_limit(sentry_system_t *mode);
extern sentry_system_t sentry_system;
extern robot_data_t robot_data;
void judge_if_shoot(decision_t *mode);
void small_gimbal_mode_chose(sentry_system_t *mode);
void sentry_decision(decision_t *mode);
static bool judge_if_shoot_sentry(int16_t robot_hp,int16_t outpost_HP);
static bool judge_if_shoot_infantr4(int16_t robot_hp);
static bool judge_if_shoot_hero(int16_t robot_hp);
static bool judge_if_shoot_Engineer(int16_t robot_hp);
static bool judge_if_shoot_infantr3(int16_t robot_hp);
bool judge_if_shoot_base(int16_t outpost_hp);
#endif