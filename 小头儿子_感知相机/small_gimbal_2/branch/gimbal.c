#include "gimbal.h"
#include "CAN_receive.h"
#include "system.h"
#include "cmsis_os.h"
#include "struct_typedef.h"
#include "stm32f4xx_hal.h"
#include "ins_task.h"
#include "bsp_pid.h"
#include "Nautilus_Vision.h"
#include "user_lib.h"
#include "shoot.h"
#include "trig.h"
#include "kalman.h"
#include "bsp_dwt.h"
#include "vofa.h"
#include "controller.h"
#include "CAN_transmit.h"
com_mode_t gimbal_com;
gimbal_t gimbal;

//后续准备写一个函数来维护小yaw与大yaw之间的真实旋转角度用来视觉追击，目前没有时间写,


pid_struct_t pid_yaw_position;
pid_struct_t pid_pitch_position;

pid_struct_t pid_yaw_speed;
pid_struct_t pid_pitch_speed;



feedforward_control_t yaw_feedforward;
feedforward_control_t pitch_feedforward;
void gimbal_run()
{

vTaskDelay(2);
gimbal_pid_init();
gimbal.yaw_cruise_dirc=1;
gimbal_init(&gimbal);
  gimbal.pitch_cruise_dire=1;
  gimbal.small_gimbal_judge.if_front_update=1;
   gimbal.vision_state=vision_lost;
  for(;;)
  {
  
  get_gimbal_com();
  get_base_link_angle(&gimbal);
  
  switch(gimbal_com)
  {
    
    case com_err:
    {
      gimbal.vision_state=vision_lost;
      gimbal.if_pitch_can=0;
      gimbal_off_set(&gimbal);
       break;
    }
    case com_nom:
    {
      gimbal.if_pitch_can=1;
      choose_gimbal_mode(&gimbal);
      break;
    }
  }
   vTaskDelay(1);
  }
  
}

void gimbal_pid_init()
{
  pid_init(&pid_yaw_position,2.0,0,0,0,70);
  pid_init(&pid_yaw_speed,400,0,0,0,30000);
  pid_init(&pid_pitch_position,0.28,0,0,0,5);
  pid_init(&pid_pitch_speed,0.3,0,0,0,8);
  //目前视觉击打车体时用的是前馈，只能打高速平移和静止车体
  feedforward_control_init(&yaw_feedforward,15,0,100);
  
  
  feedforward_control_init(&pitch_feedforward,2,0,100);
  Vision_Gimbal_Init();
}



void gimbal_init(gimbal_t *mode)
{

//云台目标值初始化
mode->yaw_set=INS.YawTotalAngle;
mode->yaw_vision_set=INS.YawTotalAngle;
mode->yaw_outpost_set=INS.YawTotalAngle;
mode->pitch_vision_set=INS.Roll;
mode->pitch_set=INS.Roll;
mode->yaw_speed_set=0;
mode->pitch_speed_set=0;

//云台相关动作状态初始化
gimbal.lock_state=no_lock;
gimbal.big_yaw_curise_dire=1;
mode->small_gimbal_judge.if_front_update=1;  
mode->small_gimbal_judge.lost_cnt_cnt=2000;
  
  
  
  
}
void get_gimbal_com()
{

  if(control_data.gimbal_mode!=small_gimbal_off&&gimbal_system.control_com==com_nom&&control_data.if_gimbal_can==1)
  { 
    gimbal_com=com_nom;
  }
  else 
  {
   
   gimbal_com=com_err;
  }
  
}
int last_get=0;
int lost_time;
int get_target=0;
//int kmjfj;  
void choose_gimbal_mode(gimbal_t *mode)
{
//kmjfj++;

//get_small_lock_state(mode);
 switch(control_data.gimbal_mode)
 {
  case small_gimbal_off:
  {
   gimbal_off_set(mode);
   mode->small_gimbal_judge.lost_cnt_cnt=2000;
   mode->vision_state=vision_lost;
   break;
  }
  case small_gimbal_rc:
  {
   gimbal_rc_set(mode);
   mode->small_gimbal_judge.lost_cnt_cnt=2000;
    mode->vision_state=vision_lost;
   break;
  }
  case small_gimbal_pc:
  {
   gimbal_pc_set(mode);
   break;
  }
  
 }
}

void gimbal_off_set(gimbal_t *mode)
{
  mode->yaw_set=INS.YawTotalAngle;
  mode->yaw_outpost_set=INS.YawTotalAngle;
  mode->yaw_vision_set=INS.YawTotalAngle;
  mode->pitch_set=INS.Roll;
  mode->pitch_vision_set=INS.Roll;
  
  mode->yaw_speed_set=0;
  mode->pitch_speed_set=0;
  
  //云台相关动作状态初始化
  yaw_motor.motor_tar.set_current=0;
   pitch_motor.Torque_SET=0;
   mode->vision_on=0;
   
   
   //云台自瞄状态
   
   
}
int lllll;




void gimbal_pc_set(gimbal_t *mode)
{


    gimbal_vision_state (mode);


    switch(mode->vision_state)
    {
      case vision_frount:
      {
        gimbal_vision_set(mode);
//        gimbal_cruise_set(mode);
        get_small_lock_state(mode);
        break;
      }
      case vision_back:
      {
        gimbal_vision_back_set(mode);
        get_small_lock_state(mode);
        break;
      }

      
      case vision_lost:
      {
        mode->small_gimbal_judge.if_front_update=1;
        gimbal_cruise_set(mode);
        mode->lock_state=no_lock;
//      gimbal_rc_set(mode);
       break;
      }
    }
      mode->vision_on=1;
   
}



#define use_yaw_ecd


 float get_yaw_diff=0;
  
int lock_cnt;

float get_yaw_diff_ecd=0;
  float yaw_motor_obj=0;
void get_small_lock_state(gimbal_t *mode)
{




 get_yaw_diff=TJ_Vision_Rx.total_yaw-INS.YawTotalAngle;
 
 if(get_yaw_diff>180)
 {
  get_yaw_diff-=360;
 }
 else if(get_yaw_diff<-180)
 {
   get_yaw_diff+=360;
 }
 
 get_yaw_diff_ecd=get_yaw_diff*8192/(360);
 
 
 
 
 #ifdef use_yaw_ecd
 
 
 yaw_motor_obj=yaw_motor.motor_measure.ecd+get_yaw_diff_ecd;
 
 if(yaw_motor_obj<0)
 {
   yaw_motor_obj+=8192;
 }
 else if(yaw_motor_obj>8192)
 {
  yaw_motor_obj-=8192;
 }
 
#else 
yaw_motor_obj = yaw_mid_ecd;

#endif 

  if(fabs(yaw_motor.motor_measure.ecd-yaw_left_ecd)<500||fabs(yaw_motor_obj-yaw_left_ecd)<500)
  { 
  
  lock_cnt++;
   mode->lock_state=left_lock;
//   mode->big_yaw_curise_dire=1;
  }
  else if(fabs(yaw_motor.motor_measure.ecd-yaw_right_ecd)<500||fabs(yaw_motor_obj-yaw_right_ecd)<500)
  {
  
  lock_cnt++;
   mode->lock_state=right_lock;
//   mode->big_yaw_curise_dire=-1;
  }
  else mode->lock_state=no_lock;
}






void gimbal_vision_chose(gimbal_t *mode)
{
  
}
int kmjfj;

void gimbal_vision_state(gimbal_t *mode)
{
  
  float yaw_vision_diff=0;
  

  yaw_vision_diff=mode->yaw_vision_set-INS.YawTotalAngle;
//  if(yaw_vision_diff>180)
//  {
//    yaw_vision_diff-=360.0f;
//  }
//  else if(yaw_vision_diff<-180)
//  {
//    yaw_vision_diff+=360;
//  }
  
//  if(mode->vision_state==vision_back&&fabs(yaw_vision_diff)<10)
//  {
//      mode->small_gimbal_judge.if_front_update=1;
//  }

//  if(TJ_Vision_Rx.Vision_gimbal_mode==no_control&&Rx_Vision.IF_stay==0)
//  {
//     mode->small_gimbal_judge.lost_cnt_cnt++;
//  }
//  else mode->small_gimbal_judge.lost_cnt_cnt=0;
//  
//  
//  
//  if(Rx_Vision.Flag_Found==0&&Rx_Vision.IF_stay==0&&mode->small_gimbal_judge.if_front_update==1)
//  {
//    mode->small_gimbal_judge.front_found_lost_cnt++;
//  }
//  
//  

//  if((Rx_Vision.Flag_Found||Rx_Vision.IF_stay)||mode->small_gimbal_judge.lost_cnt_cnt<900)
//  {
//    kmjfj++;
//    mode->small_gimbal_judge.front_found_lost_cnt=0;
//    mode->small_gimbal_judge.if_front_update=1;
//    mode->vision_state=vision_frount;
//  }
//  else if(Rx_Vision.Back_Found==1&&mode->small_gimbal_judge.front_found_lost_cnt>1000&&control_data.if_arrvied==1&&mode->small_gimbal_judge.if_front_update)
//  {
//    mode->vision_state=vision_back;
//  }
//  else if(mode->small_gimbal_judge.if_front_update==1)
//  {
//     mode->vision_state=vision_lost;
//  }
  if(TJ_Vision_Rx.Vision_gimbal_mode==no_control)
  {
    mode->vision_state=vision_lost;
  }
  else 
  {
    mode->vision_state=vision_frount;
  }
  
  

//  if(mode->small_gimbal_judge.if_front_update==0)
//  
//  if(Rx_Vision.Flag_Found||Rx_Vision.IF_stay||Rx_Vision.Back_Found)
//  {
//   mode->vision_state=vision_get;
//    lost_time=0;
//  }
//  else 
//  {
//    lost_time++;
//  }
//  
//  if(lost_time>=500)
//  {
//    mode->vision_state=vision_lost;
//  }
//  else mode->vision_state=mode->vision_state;
}



void gimbal_rc_set(gimbal_t *mode)
{


  mode->yaw_set-=control_data.gimbal_yaw_in*0.0003;
  mode->pitch_set-=control_data.gimbal_pitch_in*0.0001;
  
  mode->pitch_set=pitch_limit(mode->pitch_set);
  

  
 
  
  mode->yaw_speed_set=pid_calc(&pid_yaw_position,INS.YawTotalAngle,mode->yaw_set);
  yaw_motor.motor_tar.set_current=pid_calc(&pid_yaw_speed,INS.Gyro[2]*10,mode->yaw_speed_set);
  
  mode->pitch_speed_set=pid_calc(&pid_pitch_position,INS.Roll,mode->pitch_set);
  pitch_motor.Torque_SET=pid_calc(&pid_pitch_speed,INS.Gyro[1],mode->pitch_speed_set);
  
//   mode->yaw_vision_set=INS.YawTotalAngle;
  mode->yaw_outpost_set=INS.YawTotalAngle;
  mode->yaw_vision_set=INS.YawTotalAngle;
  mode->pitch_vision_set=INS.Roll;
  mode->vision_on=0;
}


void gimbal_cruise_set(gimbal_t *mode)
{


  gimbal_yaw_cruise(mode);
  gimbal_pitch_cruise(mode);
  
  
  
  
  mode->yaw_speed_set=pid_calc(&pid_yaw_position,INS.YawTotalAngle,mode->yaw_set);
  yaw_motor.motor_tar.set_current=pid_calc(&pid_yaw_speed,INS.Gyro[2]*10,mode->yaw_speed_set);
  
  mode->pitch_speed_set=pid_calc(&pid_pitch_position,INS.Roll,mode->pitch_set);
  pitch_motor.Torque_SET=pid_calc(&pid_pitch_speed,INS.Gyro[1],mode->pitch_speed_set);
  
  mode->yaw_outpost_set=INS.YawTotalAngle;
  mode->yaw_vision_set=INS.YawTotalAngle;
//  mode->yaw_vision_set=INS.Yaw;
  mode->pitch_vision_set=INS.Roll;
  }





//
/*
  1前面的头巡航 瞄到了那就立刻调用视觉的目标值
  2如果前面的瞄到了后五秒没有瞄到此时后面的头瞄到了
那就立刻将小yaw的目标值加或者减180直至小头转到目标值(如果过程中小头瞄到了那么立刻调用视觉的目标值)
  3如果小yaw转到了目标值后巡航5s再响应感知相机的目标
*/


int vision_count=0;

void gimbal_vision_set(gimbal_t *mode)
{

vision_count++;
//  if(Rx_Vision.armor_id==ARMOR_OUTPOST)
//  {
//     gimbal_vision_outpost_set(mode);
//  }
//  else 
gimbal_vision_car(mode);
   

}

int16_t oioi;
//打前哨站模式
void gimbal_vision_outpost_set(gimbal_t *mode)
{ 
oioi++;
   //Rx_Vision.yaw_obj
  mode->yaw_outpost_set=KalmanFilter(&test_yaw,Rx_Vision.total_yaw);
  mode->pitch_vision_set=Rx_Vision.pitch_obj;
  
  mode->pitch_vision_set=pitch_limit(mode->pitch_vision_set);
  

//  if(mode->yaw_vision_set-INS.Yaw>180)
//  {
//   mode->yaw_vision_set-=360.0f;
//  }
//  else if(mode->yaw_vision_set-INS.Yaw<-180)
//  {
//   mode->yaw_vision_set+=360.0f;
//  }
  
  
  
  mode->yaw_speed_set=pid_calc(&pid_outpost_angle,INS.YawTotalAngle,mode->yaw_outpost_set)
  ;
//                      +feedforward_control_calc(&yaw_feedforward,mode->yaw_vision_set)
  yaw_motor.motor_tar.set_current=pid_calc(&pid_outpost_speed,INS.Gyro[2]*10,mode->yaw_speed_set)
  ;
  
  mode->pitch_speed_set=pid_calc(&pid_Vision_Pitch_angle,INS.Roll,mode->pitch_vision_set)
  +feedforward_control_calc(&pitch_feedforward,mode->pitch_vision_set);
//                         
  pitch_motor.Torque_SET=pid_calc(&pid_Vision_Pitch_speed,INS.Gyro[1],pid_Vision_Pitch_angle.output);
  
  mode->yaw_vision_set=INS.YawTotalAngle;
  mode->yaw_set=INS.YawTotalAngle;
  mode->pitch_set=INS.Roll;
}

void gimbal_vision_car(gimbal_t *mode)
{


   //Rx_Vision.yaw_obj
  mode->yaw_vision_set=TJ_Vision_Rx.total_yaw;
  mode->pitch_vision_set=TJ_Vision_Rx.pitch;
  
  mode->pitch_vision_set=pitch_limit(mode->pitch_vision_set);
  

//  if(mode->yaw_vision_set-INS.Yaw>180)
//  {
//   mode->yaw_vision_set-=360.0f;
//  }
//  else if(mode->yaw_vision_set-INS.Yaw<-180)
//  {
//   mode->yaw_vision_set+=360.0f;
//  }
  
  
  
  mode->yaw_speed_set=pid_calc(&pid_Vision_Yaw_angle,INS.YawTotalAngle,mode->yaw_vision_set)
  +feedforward_control_calc(&yaw_feedforward,mode->yaw_vision_set);
//                      
  yaw_motor.motor_tar.set_current=pid_calc(&pid_Vision_Yaw_speed,INS.Gyro[2]*10,mode->yaw_speed_set)
  ;
  
  mode->pitch_speed_set=pid_calc(&pid_Vision_Pitch_angle,INS.Roll,mode->pitch_vision_set)
  +feedforward_control_calc(&pitch_feedforward,mode->pitch_vision_set);
//                         
  pitch_motor.Torque_SET=pid_calc(&pid_Vision_Pitch_speed,INS.Gyro[1],pid_Vision_Pitch_angle.output);
  
  mode->yaw_outpost_set=INS.YawTotalAngle;
  mode->yaw_set=INS.YawTotalAngle;
  mode->pitch_set=INS.Roll;
}



float limit_addspeed(float speed_set,float speed_ref,float addspeed_limit)
{
	if(fabs(speed_set-speed_ref)>addspeed_limit)
	{
		if(speed_set>speed_ref)
		{
			speed_set=speed_ref+addspeed_limit;
		}
		else if(speed_set<speed_ref)
		{
			speed_set=speed_ref-addspeed_limit;
		}
	}
	return speed_set;
}

//控制云台向后边运动，整体朝向调后
//调后后瞄到敌人就将目标值转换成视觉传过来的目标值
float yaw_fliter_set;
void gimbal_vision_back_set(gimbal_t *mode)
{
  if(mode->vision_state==vision_back&&mode->small_gimbal_judge.if_front_update==1)
  {
    gimbal_back_set(mode);
    mode->small_gimbal_judge.if_front_update=0;
  }
  
   yaw_fliter_set=limit_addspeed(mode->yaw_vision_set,yaw_fliter_set,0.5);
  
  
  gimbal_pitch_cruise(mode);
  
  mode->yaw_speed_set=pid_calc(&pid_yaw_position,INS.YawTotalAngle,mode->yaw_vision_set);
  yaw_motor.motor_tar.set_current=pid_calc(&pid_yaw_speed,INS.Gyro[2]*10,mode->yaw_speed_set);
  
  mode->pitch_speed_set=pid_calc(&pid_pitch_speed,INS.Roll,mode->pitch_vision_set);
  pitch_motor.Torque_SET=pid_calc(&pid_pitch_speed,INS.Gyro[1],mode->pitch_speed_set);
  
  mode->yaw_outpost_set=INS.YawTotalAngle;
  mode->yaw_set=INS.YawTotalAngle;
  mode->pitch_set=INS.Roll;
  
  
}



//获取当前大小yaw之间的相对角
//作用：
/*

    知道了小yaw运动的范围就可以知道小yaw到底在哪
    1可以用作巡航
    2大小yaw协作控制
    3背后大yaw感知相机也可以用这个角度来调整朝向
*/
float current_transform_ecd,current_transform_angle;
void get_base_link_angle(gimbal_t *mode)
{
   
//   mode->transform.
   mode->transform.current_transform_ecd=yaw_motor.motor_measure.ecd-yaw_mid_ecd;
   
   if(mode->transform.current_transform_ecd>4096)
   {
     mode->transform.current_transform_ecd-=8192;
   }
   else if(mode->transform.current_transform_ecd<-4096)
   {
     mode->transform.current_transform_ecd+=8192;
   }
   
   
   mode->transform.current_transform_angle=mode->transform.current_transform_ecd/8192*360;


   mode->transform.ANTI_current_transform_ecd=yaw_mid_ecd-yaw_motor.motor_measure.ecd;
   
   if(mode->transform.ANTI_current_transform_ecd>4096)
   {
     mode->transform.ANTI_current_transform_ecd-=8192;
   }
   else if(mode->transform.ANTI_current_transform_ecd<-4096)
   {
     mode->transform.ANTI_current_transform_ecd+=8192;
   }
   
   
   mode->transform.ANTI_current_transform_angle=mode->transform.ANTI_current_transform_ecd/8192*360;
}




void gimbal_back_set(gimbal_t *mode)
{
  if(yaw_motor.motor_measure.ecd>3305&&yaw_motor.motor_measure.ecd<4744)
  {
   gimbal.yaw_vision_set-=170.0f;
  }
  else 
  {
    gimbal.yaw_vision_set+=170.0f;
  }
}

void gimbal_pitch_cruise(gimbal_t *mode)
{ 
//巡航的上下限位要根据你的巡航速度和巡航pid来给定.简单来来说打印看一下是否真实角度是否到达限位
  if(mode->pitch_cruise_dire==1)
  {
//     if(mode->pitch_set>16)
     if(mode->pitch_set>29)//机械限位26
    {
      mode->pitch_cruise_dire=-1;
    }
  }
  
  else if(mode->pitch_cruise_dire==-1)
  {
//      if(mode->pitch_set<-10)
    if(mode->pitch_set<-29)//机械限位-22
    {
    mode->pitch_cruise_dire=1;
    }
  }
 
  mode->pitch_set+=(mode->pitch_cruise_dire*pitch_cruise_diff);
  
}

int okokol;
int ppppp0;
int uuuy;
void gimbal_yaw_cruise(gimbal_t *mode)
{


    


    if(mode->yaw_cruise_dirc==1)
    {
     if(gimbal_left_limit(yaw_motor.motor_measure.ecd)<min_diff_ecd)
      {
        okokol++;
        
        
        mode->yaw_cruise_dirc=-1;
      }
    }
      else if(mode->yaw_cruise_dirc==-1)
      {
        if(gimbal_right_limit(yaw_motor.motor_measure.ecd)<min_diff_ecd)
       {
        mode->yaw_cruise_dirc=1;
       }
      }
   
   
   //将大yaw的旋转速度叠加在小yaw上
  if(control_data.curise_mode==curise_stop)
  {
   mode->big_yaw_curise_dire=0;
  }
  else if(control_data.curise_mode==curise_minus)
  {
   mode->big_yaw_curise_dire=-1;
  }else if(control_data.curise_mode==curise_add)
  {
   mode->big_yaw_curise_dire=1;
  }
  
   
   mode->yaw_set+=((yaw_cruise_diff)*mode->yaw_cruise_dirc+big_yaw_cruise_diff*mode->big_yaw_curise_dire);
  
  
  //
//  mode->pitch_set=pitch_limit(mode->pitch_set);
}




/**
*@note pitch角度限幅
*/
float pitch_limit(float data)
{

  if(data>22)
  {
   data=22;
  }
  else if(data<-25.5)
  {
    data=-25.5;
  }
  
  return data;
}


/**
*@note yaw轴巡航限幅
*/
uint16_t gimbal_right_limit(int16_t motor_ecd)
{

  uint16_t diff_ecd;

diff_ecd=abs(motor_ecd-yaw_right_ecd);

  return diff_ecd;
}

uint16_t gimbal_left_limit(int16_t motor_ecd)
{

  uint16_t diff_ecd;

diff_ecd=abs(motor_ecd-yaw_left_ecd);

  return diff_ecd;
}




int tttt;
//float diff_time;
//uint32_t time;
void printf_task(void const * argument)
{
	while(1)
	{
  tttt++;

//Vofa_Send_Data4(trig_motor.motor_tar.set_current,trig_motor.motor_measure.speed_rpm,trig_motor.motor_measure.total_ecd,trig_motor.motor_measure.feedback_current);
Vofa_Send_Data4(INS.YawTotalAngle,INS.Pitch,INS.Roll,(float)pid_Vision_Yaw_speed.set);
//Vofa_Send_Data4(INS.Yaw,Rx_Vision.yaw_obj,Rx_Vision.Flag_Found,Rx_Vision.Back_Found);
////Vofa_Send_Data4(INS.Roll,gimbal.pitch_set,INS.Gyro[1],gimbal.pitch_speed_set);
//  Vofa_Send_Data4(INS.Roll,Rx_Vision.pitch_obj,INS.Gyro[1],gimbal.pitch_speed_set);
//Vofa_Send_Data4((float)shoot_motor[0].motor_measure.speed_rpm,(float)-shoot_motor[1].motor_measure.speed_rpm,over_speed,control_data.shoot_speed);
//Vofa_Send_Data4(Rx_Vision.Flag_Found,gggggg,oooiii,control_data.curise_mode);
 osDelay(7);
  
  
  
	}
}
