#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "struct_typedef.h"
void get_gimbal_com();
float pitch_limit(float data);

typedef enum
{

 no_lock=0,
  left_lock,
  right_lock,
}lock_state_t;
typedef enum
{
  vision_frount=0,
  vision_back,
  vision_lost,
}vision_state_t;

//typedef enum
//{
//  vision_frount=0,
//  vision_back,
//}vision_decision_state_t;

typedef struct
{
  uint16_t front_found_lost_cnt;
  uint8_t if_front_update;
  uint16_t wait_to_update_cnt;
  uint16_t lost_cnt_cnt;
  
}small_gimbal_judge_t;

typedef struct
{
   float current_transform_ecd;
  float current_transform_angle;
  
  float ANTI_current_transform_ecd;
  float ANTI_current_transform_angle;
}transform_t;

typedef struct
{
  float yaw_set;
  float pitch_set;
 
  
  float yaw_vision_set;
  float pitch_vision_set;
  
  float yaw_outpost_set;
  
  float yaw_speed_set;
  float pitch_speed_set;
  int if_pitch_can;
  int yaw_cruise_dirc;
  int pitch_cruise_dire;
  int vision_found;
  int vision_on;
  lock_state_t lock_state;
  int big_yaw_curise_dire;
  vision_state_t vision_state;
  uint8_t right_angle;
  uint8_t left_angle;
  small_gimbal_judge_t small_gimbal_judge;
transform_t transform;
  
//  vision_decision_state_t vision_decision_state;
}gimbal_t;


extern gimbal_t gimbal;
extern int get_target;
void gimbal_pitch_cruise(gimbal_t *mode);
void gimbal_yaw_cruise(gimbal_t *mode);
void choose_gimbal_mode(gimbal_t *mode);
void gimbal_rc_set(gimbal_t *mode);
void gimbal_pc_set(gimbal_t *mode);
void gimbal_cruise_set(gimbal_t *mode);
uint16_t gimbal_left_limit(int16_t motor_ecd);
uint16_t gimbal_right_limit(int16_t motor_ecd);
float pitch_limit(float data);
void gimbal_off_set(gimbal_t *mode);
void gimbal_vision_set(gimbal_t *mode);
void motor_Send();
void gimbal_pid_init();
void gimbal_init(gimbal_t *mode);
void get_small_lock_state(gimbal_t *mode);
void gimbal_vision_state(gimbal_t *mode);
void gimbal_stay_Set(gimbal_t *mode);
void gimbal_back_set(gimbal_t *mode);
void gimbal_vision_back_set(gimbal_t *mode);
void get_base_link_angle(gimbal_t *mode);
extern float get_yaw_diff;
extern void gimbal_vision_outpost_set(gimbal_t *mode);
extern void gimbal_vision_car(gimbal_t *mode);
/**
*@note 巡航相关参数
//*/
//#define yaw_left_ecd 1000
//#define yaw_right_ecd 5600
//#define min_diff_ecd 300
//#define yaw_cruise_diff 0.0
//#define pitch_cruise_diff 0.25

//#define big_yaw_cruise_diff 0.195


#define yaw_left_ecd 6300
#define yaw_right_ecd 3086
#define yaw_mid_ecd 4674
#define min_diff_ecd 300
#define yaw_cruise_diff 0.05
#define pitch_cruise_diff 0.15

#define big_yaw_cruise_diff 0.035
#endif