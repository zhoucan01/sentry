#ifndef __SHOOT_H

#define __SHOOT_H



#include "stdbool.h"
#define YES  1
#define NO   0


typedef struct
{
	float shoot_speed_set;
	float shoot_speed_fix;
	float shoot_tem_fix;
	bool if_shoot_speed_yes;
	float get_now_speed;
	
	
}shoot_t;
void shoot_pid_init();
void shoot_pid_clac(void);
void if_shoot_com(void);
bool Report_IF_Fric3508_SetSpeed(void);
void SpeedAdapt(float real_S , float min_S, float max_S,float *fix , float up_num , float down_num);
extern shoot_t stander_shoot;
float Report_RealShootSpeed(void);
void shoot_speed_set(shoot_t *mode);
void Temp_Fix_30S(void);
//#define FRICTION_L3_SPEED 6300 //7150
bool Report_IF_Fric3508_SetSpeed(void);
extern float over_speed;

#define PID_shoot_KP 5
#define PID_shoot_KI 0
#define PID_shoot_IMAX 100
#define PID_shoot_MAX  16384





#define MIN_SPEED 22.2
#define MAX_SPEED 23.0
#define UP_NUM   10
#define DOWN_NUM  10

//³õÊ¼µ¯ËÙ
#define FRICTION_L3_SPEED 6500//7150

//#define FRICTION_L3_SPEED 5500//7150
#endif