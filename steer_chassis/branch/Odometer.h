#ifndef __ODOMETER_H
#define __ODOMETER_H
#include "system.h"

void Odometer_calculate(chassis_t *mode);
typedef struct
{
 float Sx;
 float Sy;
 float Sx_set;
 float Sy_set;
 float steer_real_angle[4];
 float steer_init_ecd[4];
 float vx,vy;
 float vx_all[4];
 
 float init_yaw;
 float diff_yaw;
 float diff_angle;
 
 
 
 float relative_init_angle;  // 初始化时yaw的角度
 float relative_angle;//云台与底盘的相对角
 
}location_t;
extern location_t location;
void loacation_init();
#endif