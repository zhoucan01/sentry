#include "gimbal_big_yaw.h"
#include "system.h"
#include "cmsis_os.h"
//#include "vision.h"
#include "referee.h"
#include "remote_control.h"
#include "can_receive.h"
#include "ins_task.h"
//#include "vision.h"
#include "bsp_pid.h"
com_mode_t gim_com;
com_mode_t last_gim;
gimbal_t gimbal;
pid_struct_t pid_yaw_angle;
pid_struct_t pid_yaw_speed;
pid_struct_t pid_vision_yaw_angle;
pid_struct_t pid_vision_yaw_speed;
void gimbal_pid_init()
{
 pid_init(&pid_yaw_angle,2.3,0,0,0,8);
 pid_init(&pid_yaw_speed,2.0,0,0,0,10);
 
 pid_init(&pid_vision_yaw_angle,1,0,0,0,10);
 pid_init(&pid_vision_yaw_speed,1,0,0,0,10);
}
void big_yaw_run()
{

gimbal_pid_init();
  for(;;)
  {
  get_big_gimbal_com();
  switch(gim_com)
  {
    case com_err:
    {
    gimbal.yaw_set=INS.YawTotalAngle;
    gimbal.if_big_yaw_can=0;
    break;
    }
    case com_nom:
    {
      gimbal.if_big_yaw_can=1;
      gimbal_mode_set();
      gimbal_current_clac();
      break;
    }
  }
    vTaskDelay(1);
  }
}


//ÅÐ¶ÏÔÆÌ¨ÊÇ·ñ¿ªÆô
void get_big_gimbal_com(void)
{
	if(sentry_system.chassis_mode==no_move||get_if_communite_broke()==1||
	robot_status.power_management_gimbal_output==0)
	{
		gim_com=com_err;
	}
	else gim_com=com_nom;
	
	if(robot_status.power_management_gimbal_output==0)
	{
	yaw_motor.Err=0;
	}
	
	
	last_gim=gim_com;
}


void gimbal_mode_set(void)
{
  if(sentry_system.vision_mode==vision_off)
  {
    gimbal_vision_no_mode();
  }
  else if(sentry_system.vision_mode==vision_on)
  {
    gimbal_vision_no_mode();
  }

}
void gimbal_vision_no_mode()
{
  gimbal.yaw_set-=sentry_system.set_yaw_in; 
}


void gimbal_vision_on_mode()
{
  

}

void gimbal_current_clac()
{
 if(gim_com==1)
 {
//   if(sentry_system.vision_mode==vision_off||(sentry_system.vision_mode==vision_on&&FlagFound==1))
//   {
//   gimbal.big_yaw_speed_set=pid_calc(&pid_yaw_angle,INS.YawTotalAngle,gimbal.yaw_set);
//   yaw_motor.Torque_SET  =pid_calc(&pid_yaw_speed,INS.Gyro[2],gimbal.big_yaw_speed_set);
//   }
//   else 
//   {
     gimbal.big_yaw_speed_set=pid_calc(&pid_yaw_angle,INS.YawTotalAngle,gimbal.yaw_set);
     yaw_motor.Torque_SET  =pid_calc(&pid_yaw_speed,INS.Gyro[2],gimbal.big_yaw_speed_set);
//   }
 }
 else 
 {
 yaw_motor.Torque_SET=0;
 }
 
}