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
#include "navigation.h"
#include "stdbool.h"
com_mode_t gim_com;
com_mode_t last_gim;
gimbal_t gimbal;
pid_struct_t pid_yaw_angle;
pid_struct_t pid_yaw_speed;
pid_struct_t pid_vision_yaw_angle;
pid_struct_t pid_vision_yaw_speed;
pid_struct_t pid_navi_yaw_speed;
void gimbal_pid_init()
{

 
 pid_init(&pid_yaw_angle,1.0,0,0,0,10);
 pid_init(&pid_yaw_speed,1,0,0,0,10);
 pid_init(&pid_navi_yaw_speed,2.5,0,0,0,5);
 

 pid_init(&pid_vision_yaw_angle,1.0,0,0,0,10);
 pid_init(&pid_vision_yaw_speed,1.0,0,0,0,10);
}

int big_yaw_count;
void big_yaw_run()
{
gimbal.curise_direction=1;
gimbal_pid_init();
  for(;;)
  {
  get_big_gimbal_com();
  switch(gim_com)
  {
    case com_err:
    {
    gimbal.yaw_set=INS.YawTotalAngle;
    gimbal.yaw_vision_set=INS.YawTotalAngle;
    gimbal.if_big_yaw_can=0;
    break;
    }
    case com_nom:
    {
      gimbal.if_big_yaw_can=1;
      gimbal_mode_set(&gimbal);
      break;
    }
  }
    vTaskDelay(1);
  }
}



int big_yaw_lost_count=0;//记录大yaw断电情况,计数小于200时认为以断电

//判断云台是否开启
void get_big_gimbal_com(void)
{    
   
   big_yaw_lost_count++;
   
	if(sentry_system.chassis_mode==no_move||get_if_communite_broke()==1||
	robot_status.power_management_gimbal_output==0||big_yaw_lost_count>200)
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




int lokos;
int kjkhd;
int jdgsj;
void gimbal_mode_set(gimbal_t *mode)
{
 

 
 get_gimbal_com_count++;
  switch (sentry_system.big_yaw_mode)
  {
    case big_yaw_off: 
     {
         lokos++;
         mode->yaw_set=INS.YawTotalAngle;
         mode->yaw_vision_set=INS.YawTotalAngle;
         break;
     }    
    case big_yaw_rc:
    {
       jdgsj++;
       gimbal_vision_no_mode();
       break;
    }
    case big_yaw_pc:
    {
    
//    gimbal_vision_no_mode();
      gimbal_vision_on_mode(mode);
      break;
    }
  }
  
}




void gimbal_vision_no_mode()
{
  gimbal.yaw_set-=sentry_system.set_yaw_in; 
  kjkhd++;
  gimbal.yaw_vision_set=INS.YawTotalAngle;
  gimbal.if_update=1;
  
  gimbal.big_yaw_speed_set=pid_calc(&pid_yaw_angle,INS.YawTotalAngle,gimbal.yaw_set);
  yaw_motor.Torque_SET  =pid_calc(&pid_yaw_speed,INS.Gyro[2],gimbal.big_yaw_speed_set);
}


float compare_yaw_add(float yaw_in)
{
  if(fabs(yaw_in)<50)
  { 
    yaw_in=50;
  }
  else yaw_in=fabs(yaw_in);
  return yaw_in;
}
lock_state_t Last_lock_state;
Vision_look_state_t Last_look_state;
bool get_gimbal_response_state(gimbal_t *mode)
{
//如果检测到卡限位了就让大yaw转，目标值累加到不卡了就停下来再进入检测模式,且此时增加的值为大yaw卡着的瞬时应该增加的值
   if(fabs(mode->yaw_vision_set-mode->get_current_yaw)<compare_yaw_add(mode->yaw_add)&&mode->if_update==0)
   { 
     if(mode->curise_direction==-1)
     { 
       mode->yaw_vision_set-=0.2;
     }
     else 
     {
       mode->yaw_vision_set+=0.2;
     }
     mode->if_update=0;
   }
   else 
   {
     mode->if_update=1;
   }
}


void gimbal_curise_set(gimbal_t *mode)
{
 mode->yaw_vision_set+=mode->curise_direction*yaw_cusise_diff;
 mode->if_update=1;
 mode->get_current_yaw=INS.YawTotalAngle;
 gimbal.yaw_set=INS.YawTotalAngle;
 
 gimbal.big_yaw_speed_set=pid_calc(&pid_vision_yaw_angle,INS.YawTotalAngle,gimbal.yaw_vision_set);
  yaw_motor.Torque_SET  =pid_calc(&pid_vision_yaw_speed,INS.Gyro[2],gimbal.big_yaw_speed_set);
}

void gimbal_vision_set(gimbal_t *mode)
{
lokos++;

gimbal.yaw_set=INS.YawTotalAngle;

  get_gimbal_response_state(mode);
  if(receive_gimbal_data.vision_state!=vision_lost&&receive_gimbal_data.lock_state==left_lock&&mode->if_update==1)
   { 
   mode->yaw_add=receive_gimbal_data.small_yaw_add;
   mode->get_current_yaw=INS.YawTotalAngle;
   mode->curise_direction=-1*direc;
     mode->if_update=0;
   }
   else if(receive_gimbal_data.vision_state!=vision_lost&&receive_gimbal_data.lock_state==right_lock&&mode->if_update==1)
   {
      mode->yaw_add=receive_gimbal_data.small_yaw_add;
   mode->get_current_yaw=INS.YawTotalAngle;
   mode->curise_direction=1*direc;
     mode->if_update=0;
   }
  else if(receive_gimbal_data.vision_state!=vision_lost&&receive_gimbal_data.lock_state==no_block&&mode->if_update==1)
  {
   mode->yaw_vision_set=mode->yaw_vision_set;
  }
   
   
  gimbal.big_yaw_speed_set=pid_calc(&pid_vision_yaw_angle,INS.YawTotalAngle,gimbal.yaw_vision_set);
  yaw_motor.Torque_SET  =pid_calc(&pid_vision_yaw_speed,INS.Gyro[2],gimbal.big_yaw_speed_set);
}

uint8_t last_vision_mode;
void gimbal_vision_on_mode(gimbal_t *mode)
{
  

  
  
//  if(decision.Judge_condition.IF_Arrived==1)
//  {
     if(get_gimbal_com_count>100)
     {
       receive_gimbal_data.vision_state=vision_lost;
     }
     
       if(decision.point==WE_FORTRESS_POINT&&decision.Judge_condition.IF_Arrived==1)
  {
  
     mode->gimbal_mode=gimbal_vision_mode;
        gimbal_vision_set(mode);
//        gimbal_vision_no_mode();
        mode->if_update=1;
  }
     
  else if(receive_gimbal_data.vision_state==vision_lost)
      {
      
        mode->gimbal_mode=gimbal_curise_mode;
        gimbal_curise_set(mode);
//        gimbal_vision_no_mode();
        mode->if_update=1;
      }
      else 
      {
        mode->gimbal_mode=gimbal_vision_mode;
//        gimbal_vision_no_mode();
        gimbal_vision_set(mode);
      }
      
//  }
//  else 
//  {
////  gimbal_vision_no_mode();

//gimbal_curise_set(mode);
////    gimbal_navi_set(mode);
//  }
  
  
  
}


float yaw_kkp=1.9;
float yaw_speed_kp_1=1.0;

void gimbal_navi_set(gimbal_t *mode)
{
 yaw_motor.Torque_SET=pid_calc(&pid_navi_yaw_speed,INS.Gyro[2]*yaw_speed_kp_1,navigation_rx.navi_wz*yaw_kkp);
 gimbal.yaw_set=INS.YawTotalAngle;
 gimbal.yaw_vision_set=INS.YawTotalAngle;
}





