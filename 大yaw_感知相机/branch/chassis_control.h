#ifndef __CHASSIS_CONTROL_H
#define __CHASSIS_CONTROL_H

#include "system.h"
typedef struct
{
  float chassis_vx;
  float chassis_vy;
  float chassis_wz;
  float speed_kp;
  float speed_vx;
  float speed_vy;
  float speed_max;
  float max_wz;
}sentry_chassis_t;

/*******¹¦ÂÊ********/
#define POWER_40_W    40
#define POWER_45_W    45
#define POWER_50_W    50
#define POWER_60_W    60
#define POWER_70_W    70
#define POWER_80_W    80
#define POWER_100_W   100
#define POWER_120_W   120

#define power40_wzspeed 2200
#define power45_wzspeed 2400
#define power50_wzspeed 2700
#define power55_wzspeed 2900
#define power60_wzspeed 3200
#define power65_wzspeed 3500
#define power70_wzspeed 3900
#define power75_wzspeed 4100
#define power80_wzspeed 4350
#define power85_wzspeed 4650
#define power90_wzspeed 4850
#define power95_wzspeed 5150
#define power100_wzspeed 5150
#define power105_wzspeed 5300
#define power110_wzspeed 5600
#define power120_wzspeed 5800


void get_chassis_com();
int chassis_speed_limit(int speed_init,int speed_max);
void chassis_task_run();
extern com_mode_t chassis_com;
extern sentry_chassis_t sentry_chassis;;
void chassis_normol_set(sentry_chassis_t *mode);
void chassis_no_move_set(sentry_chassis_t *mode);
void chassis_normol_move_set(sentry_chassis_t *mode);
void get_chassis_mode_set(sentry_chassis_t *mode);
void chassis_spine_move_set(sentry_chassis_t *mode);
void chassis_lock_move(sentry_chassis_t *mode);
#endif