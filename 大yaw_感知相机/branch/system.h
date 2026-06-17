#ifndef __SYSTEM_H
#define __SYSTEM_H

#include "struct_typedef.h"
#include "stdbool.h"
#include "decision.h"
typedef enum
{
no_move=0,//底盘不动
normol_move,
navigation_move,//
}chassis_mode_t;
#define WHEEL_PERIMETER 			 376.99f	//车轮周长 (轮子直径 * PI 再转换成mm)
#define M3508_RATIO 	 				 19.2032				//电机减速比
#define Radius 								 62				//轮径mm  半径
#define distance_x             180/1000      //舵轮中心到底盘中心的距离
#define distance_y             180/1000     //

//#define normal_test

#define WHEEL_FACTOR 0.00034   // 电机转速单位rpm转换为真实速度m/s 的常数

#define big_yaw_kp 0.0003
typedef enum
{
shoot_no=0,
shoot_on,//手打

}shoot_mode_t;


typedef enum
{
  no_spine=0,
  low_spine,
  mid_spine,
  high_spine,
  lock_spine,
}Vz_state_t;

typedef struct
{
 float set_vx;
  float set_vy;
  Vz_state_t Vz_state; 

}chassis_set_t;
typedef enum
{
 vision_off=0,
 vision_on,
// vision_navi,
}vision_mode_t;


typedef enum
{
  rc_mode=0,
  auto_mode,
}control_mode_t;


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

typedef enum
{
  trig_off=0,
  trig_on=1,
}trig_mode_t;

typedef enum
{
  big_yaw_off=0,
  big_yaw_rc,
  big_yaw_pc,
}big_yaw_mode_t;


typedef struct
{
  float set_yaw_in;
  float speed_kp;
  chassis_set_t chassis_yaw_t;
  chassis_set_t chassis_set;
  shoot_mode_t shoot_mode;
  vision_mode_t vision_mode;
  chassis_mode_t chassis_mode; 
 control_mode_t control_mode;
 trig_mode_t trig_mode;
 small_gimbal_mode_t small_gimbal_mode;
 big_yaw_mode_t big_yaw_mode;
 
 int if_small_pitch_can;
 
}sentry_system_t;

extern sentry_system_t sentry_system;



void AGV_big_yaw(sentry_system_t *mode);
void AGV_mode_chose(sentry_system_t *mode);
void remote_offline_set(sentry_system_t *mode);
void shoot_mode_chose(sentry_system_t *mode);

void choose_control_mode(sentry_system_t *mode);
uint8_t judg_if_imu_error(void);
//void chassis_move_limit(sentry_system_t *mode);
extern sentry_system_t sentry_system;

//void judge_if_shoot(decision_t *mode);
void small_gimbal_mode_chose(sentry_system_t *mode);
//void sentry_shoot_decision(decision_t *mode);
void Auto_shoot_mode_chose(sentry_system_t *mode);
void AUTO_big_yaw(sentry_system_t *mode);
void Auto_small_gimbal_mode(sentry_system_t *mode);
//void AGV_auto_chassis(decision_t *mode,sentry_system_t *move);
void judge_if_heat_full(sentry_system_t *mode);
void chassis_speed_set(sentry_system_t *mode);

#endif