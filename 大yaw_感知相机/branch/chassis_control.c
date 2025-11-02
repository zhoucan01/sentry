#include  "chassis_control.h"
#include "cmsis_os.h"
#include "referee.h"
#include "gimbal_big_yaw.h"
#include "system.h"
#include "remote_control.h"
#include "navigation.h"
sentry_chassis_t sentry_chassis;

com_mode_t chassis_com;
void chassis_task_run()
{
  for(;;)
  {
 sentry_chassis.speed_max=700;
 sentry_chassis.max_wz=2000;
    get_chassis_com();
    get_chassis_mode_set(&sentry_chassis);
    
    vTaskDelay(1);
  }
  
}

//void choose_chassis_mode()
//{
//  switch(chassis_com)
//  {
//   case com_err:
//   {
//     
//     break;
//   }
//   case com_nom:
//   { 
//     
//     break;
//   }
//  }
//  
//}

void get_chassis_com()
{
  if(sentry_system.chassis_mode==no_move||get_if_communite_broke()==1||
	robot_status.power_management_chassis_output==0)
	{
		chassis_com=com_err;
	}
	else chassis_com=com_nom;
}



void get_chassis_mode_set(sentry_chassis_t *mode)
{
  switch(chassis_com)
  {
    case com_err:
    {
      mode->chassis_vx=0;
      mode->chassis_vy=0;
      mode->chassis_wz=0;
      break;
    }
    case com_nom:
    {
    
      chassis_normol_set(mode);
      break;
    }
  }
}

void chassis_normol_set(sentry_chassis_t *mode)
{



  switch(sentry_system.chassis_mode)
  {
  

    case no_move:
    {
    chassis_normol_move_set(mode);
    break;
    }
    case normol_move:
    {
    chassis_normol_move_set(mode);
      
      break;
    }

    
    case navigation_move:
    {
      chassis_spine_move_set(mode);
      break;
    }
    


  }
 
  
}


void chassis_no_move_set(sentry_chassis_t *mode)
{
    mode->chassis_vx=0;
    mode->chassis_vy=0;
    mode->chassis_wz=0;
}

void chassis_normol_move_set(sentry_chassis_t *mode)
{
      mode->chassis_vx=sentry_system.chassis_set.set_vx;
      mode->chassis_vy=sentry_system.chassis_set.set_vy;
      mode->chassis_wz=0;
}


void chassis_spine_move_set(sentry_chassis_t *mode)
{
      mode->chassis_vx=sentry_system.chassis_set.set_vx*0.25;
      mode->chassis_vy=sentry_system.chassis_set.set_vy*0.25;
      mode->chassis_wz=1200;
}

//在有导航后开启跟随，即底盘与云台夹角不变，为了更好的跨越各种地形
void chassis_lock_move(sentry_chassis_t *mode)
{
      mode->chassis_vx=sentry_system.chassis_set.set_vx;
      mode->chassis_vy=sentry_system.chassis_set.set_vy;
      mode->chassis_wz=0;
}




int chassis_speed_limit(int speed_init,int speed_max)
{
 if(speed_init>speed_max)
 {
  speed_init=speed_max;
 }
 else if(speed_init<-speed_max)
 {
  speed_init=-speed_max;
 }
 
 
return speed_init;
  
}