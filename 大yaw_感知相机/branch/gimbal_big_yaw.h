#ifndef __GIMBAL_BIG_YAW_H
#define __GIMBAL_BIG_YAW_H


#include "stdbool.h"

#define yaw_cusise_diff 0.03
typedef enum
{ 
  Responded=0,
  Not_responding,
}response_state_t;


typedef enum
{
  gimbal_rc_mode=0,
  gimbal_curise_mode,
  gimbal_vision_mode,
}gimbal_mode_t;

//typedef enum
//{
// curise_stop=0,
// curise_add,
// curise_minus,
// 
//}curise_mode_t;
typedef struct
{
float yaw_add;
 float yaw_set;
 int if_big_yaw_can;
 float big_yaw_speed_set;
 gimbal_mode_t gimbal_mode;
 float yaw_vision_set;
 float get_current_yaw;
 int   if_update;
 int   curise_direction;
}gimbal_t;



#define direc -1
extern gimbal_t gimbal;
extern int big_yaw_lost_count;
void get_big_gimbal_com(void);
void gimbal_mode_set(gimbal_t *mode);
extern int big_yaw_count;
void gimbal_vision_set(gimbal_t *mode);
void gimbal_vision_on_mode(gimbal_t *mode);
void gimbal_vision_no_mode();
void gimbal_navi_set(gimbal_t *mode);
void gimbal_current_clac(gimbal_t *mode);
#endif