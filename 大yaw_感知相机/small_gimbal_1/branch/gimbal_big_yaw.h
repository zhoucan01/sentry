#ifndef __GIMBAL_BIG_YAW_H
#define __GIMBAL_BIG_YAW_H

typedef struct
{
 float yaw_set;
 int if_big_yaw_can;
 float big_yaw_speed_set;
}gimbal_t;


extern gimbal_t gimbal;

void get_big_gimbal_com(void);
void gimbal_mode_set(void);
void gimbal_vision_no_mode();
void gimbal_vision_on_mode();
void gimbal_current_clac();
#endif