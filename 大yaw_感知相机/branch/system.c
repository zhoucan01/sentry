#include "bsp_transmit.h"
#include "cmsis_os.h"

#include "system.h"

#include "remote_control.h"
#include <stdlib.h>
#include "referee.h"
#include "stdbool.h"
#include "Nautilus_Vision.h"
#include "CAN_receive.h"
#include "decision.h"
#include "navigation.h"
#include "ins_task.h"
#include "bsp_pid.h"

#define Navigation

//#define test//在system.h0

/*
1如果定义在比赛过程中的话 则在比赛过程中直接自动化运行，不管
 遥控器，不在比赛过程中则根据是否自动化运行，如果不自动化运行
 则根据遥控器来选择
2如果不定义在比赛过程中的话，则依靠遥控器来控制它

*/ 

//#define Game_prograss   
sentry_system_t sentry_system;
robot_data_t robot_data;
pid_struct_t pid_chaais_Sx;
pid_struct_t pid_chaais_Sy;
void system_run(void const *argument)
{
    (void)argument;
  pid_init(&pid_chaais_Sx,0.1,0,0,0,1);
  pid_init(&pid_chaais_Sy,0.1,0,0,0,1);
  sentry_system.control_mode=rc_mode;
  robot_status.power_management_chassis_output=1;
  robot_status.power_management_gimbal_output=1;
  robot_status.power_management_shooter_output=1;
  for(;;)
  {
  
  
  robot_status.power_management_chassis_output=1;
  robot_status.power_management_gimbal_output=1;
  robot_status.power_management_shooter_output=1;
  
  #ifdef Game_prograss

  #else 
//  WHEEL_STATE_Ctrl();
  if(get_if_communite_broke()==1||decision.keyboard_disable==1||judg_if_imu_error())
  {
    remote_offline_set(&sentry_system);
  }
  else
  {
    choose_control_mode(&sentry_system);
    
    switch (sentry_system.control_mode)
    {
      case rc_mode:
      {
        AGV_mode_chose(&sentry_system);
      AGV_big_yaw(&sentry_system);
      shoot_mode_chose(&sentry_system);
      small_gimbal_mode_chose(&sentry_system);
        break;
      }
      case auto_mode:
      {
        AGV_auto_chassis(&decision);
        AUTO_big_yaw(&sentry_system);
        Auto_shoot_mode_chose(&sentry_system);
        Auto_small_gimbal_mode(&sentry_system);
        break;
      }
      
    }
//    sentry_system.chassis_mode=no_move;
    
  }
  #endif
  
//   Judge_Continuous_Handle(&decision.Judge_condition);
//   sentry_shoot_decision(&decision);
   
   
    vTaskDelay(1);
  }
}




uint8_t judg_if_imu_error()
{
  uint8_t if_imu_error;
  if(fabs(INS.Pitch)>65||fabs(INS.Roll)>65)
  {
     if_imu_error=1;  
  }
  else if_imu_error=0;
  
  return if_imu_error;
}

//mode->shoot_mode=shoot_on;
//mode->trig_mode=trig_on;

void Auto_shoot_mode_chose(sentry_system_t *mode)
{
  if(game_state.game_progress==4)
  {
    if(rc_ctrl.rc.s_l!=2)
    {
      mode->shoot_mode=shoot_on;
      mode->trig_mode=trig_on;
    }
    else 
    {
      mode->shoot_mode=shoot_no;
      mode->trig_mode=trig_off;
    }
  }
  else 
  {
      mode->shoot_mode=shoot_no;
      mode->trig_mode=trig_off;
  }
}







//void AGV_auto_mode(decision_t *mode)
//{
//  if(game_state.game_progress==4)
//  {
//       if(rc_ctrl.rc.s_l==3&&rc_ctrl.rc.s_r==3)
//      {
//        mode->decision_mode=protect;
//      }
//      else if(rc_ctrl.rc.s_l==1&&rc_ctrl.rc.s_r==3)
//      {
//        mode->decision_mode=att_outpost;
//      }
//      else if(rc_ctrl.rc.s_l==3&&rc_ctrl.rc.s_r==1)
//      {  
//        mode->decision_mode=attack;
//      }
//      else if(rc_ctrl.rc.s_l==1&&rc_ctrl.rc.s_r==1)
//      {  
//        mode->decision_mode=infringe;
//      }
//      else if(rc_ctrl.rc.s_l==2||rc_ctrl.rc.s_r==2)
//      {
//        mode->decision_mode=protect;
//      }      
//  }
//  else 
//  {
//      mode->decision_mode=protect;
//  }
//}








uint8_t last_navi_seq;

void AGV_mode_chose(sentry_system_t *mode)
{
  switch(rc_ctrl.rc.s_l)
  {
    case 2:
    {
    mode->chassis_mode=no_move;
    mode->chassis_set.Vz_state=no_spine;
    break;
    }
    case 3:
    {
    
    
//    decision.Judge_condition.IF_Arrived=0;
//    navigation_rx.if_arrived=0;
      mode->chassis_mode=normol_move;
      mode->chassis_set.Vz_state=no_spine;
      break;
    }
    case 1:
    {
    
        if(rc_ctrl.rc.s_r==1)
        {
//        (game_state.game_progress==4&&)
          if(decision.Judge_condition.IF_Arrived==0)
          {
             mode->chassis_mode=navigation_move;
             mode->chassis_set.Vz_state=no_spine;
          }
          else 
          {
            mode->chassis_mode=normol_move;
            mode->chassis_set.Vz_state=low_spine;
          }
          
        }
        else 
        {
          mode->chassis_set.Vz_state=high_spine;
          mode->chassis_mode=normol_move;
        }
        
        
        break;
    }
  }
  chassis_speed_set(mode);
 
}


uint8_t if_lost_navi;

int16_t navi_weak_state;
void chassis_speed_set(sentry_system_t *mode)
{
  /**
  *@note 这里将遥控器输入速度转换成m/s的单位与导航单位一致
  */
  switch(mode->chassis_mode)
  {
    case no_move:
    {
      mode->chassis_set.set_vx=0;
      mode->chassis_set.set_vy=0;

      break;
    }
    case normol_move:
    {
     mode->chassis_set.set_vx=-rc_ctrl.rc.ch1*4.5*WHEEL_FACTOR;
     mode->chassis_set.set_vy=-rc_ctrl.rc.ch0*4.5*WHEEL_FACTOR;
     break; 
    }
    case navigation_move:
    {
       mode->chassis_set.set_vx=-navigation_rx.navi_vx;
       mode->chassis_set.set_vy=navigation_rx.navi_vy;
     break;
    }

  }
  
  navi_tx_count++;
  
  if(fabs(navigation_rx.navi_vx)<0.15&&fabs(navigation_rx.navi_vy)<0.15)
  {
    navi_weak_state++;
  }
  else navi_weak_state=0;
  
  
  
  if(navi_tx_count>500||navi_weak_state >100)
  {
      navigation_rx.if_lost_navi=1;

  }
  else 
  {
   navigation_rx.if_lost_navi=0;
  }
  
  //根据底盘虚弱情况来给不同的速度
  if(decision.Judge_condition.If_chassis_weak==1)
  {
    mode->chassis_set.set_vx*=0.33;
    mode->chassis_set.set_vy*=0.33;
  }
  else {
    mode->chassis_set.set_vx=mode->chassis_set.set_vx;
    mode->chassis_set.set_vy=mode->chassis_set.set_vy;
  }

     
    
    
    
}



void AGV_big_yaw(sentry_system_t *mode)
{
  if(rc_ctrl.rc.s_l!=2)
  {
      switch(rc_ctrl.rc.s_r)
    {
      case 1:
      {
        if(rc_ctrl.rc.s_l==1)
        {
          mode->big_yaw_mode=big_yaw_pc;
        }
        else 
        {
          mode->big_yaw_mode=big_yaw_rc;
        }
      
        
        break;
      }
      case 3:
      {
       mode->big_yaw_mode=big_yaw_rc;
       break;
       
      }
      case 2:
      {
       mode->big_yaw_mode=big_yaw_rc;
       break;      
      }
      
    }
  mode->set_yaw_in=rc_ctrl.rc.ch2*big_yaw_kp;
 }
 else 
 {
   mode->big_yaw_mode=big_yaw_off;
   mode->set_yaw_in=0;
 }
 
 
}


void AUTO_big_yaw(sentry_system_t *mode)
{
  mode->set_yaw_in=rc_ctrl.rc.ch2*big_yaw_kp;
  
  
  //在比赛过程中完全不依靠遥控器操控

    switch (rc_ctrl.rc.s_l)
    {
      case 1:
      {
        mode->big_yaw_mode=big_yaw_pc;
        break;
      }
      case 3:
      {
        mode->big_yaw_mode=big_yaw_pc;
        break;
      }
      case 2:
      {
        mode->big_yaw_mode=big_yaw_off;
        break;
      }
    }

}



void shoot_mode_chose(sentry_system_t *mode)
{
  switch(rc_ctrl.rc.s_r)
  {
    case 1:
    {
    
    
      if(mode->vision_mode==vision_off)
      {
        mode->shoot_mode=shoot_on;
        mode->trig_mode=trig_on;
      }
      else if(mode->vision_mode==vision_on)
      {
          #ifndef normal_test 
         if(game_state.game_progress==4)
        {
        
        if(decision.Judge_condition.IF_Arrived==1)
           {
             mode->shoot_mode=shoot_on;
             mode->trig_mode=trig_on; 
           }
           else 
           {
             mode->shoot_mode=shoot_on;
             mode->trig_mode=trig_on;
           }
          
        }
         else 
         {  
           mode->shoot_mode=shoot_on;
           mode->trig_mode=trig_on;
         }
           
        #else 
        mode->shoot_mode=shoot_on;
          mode->trig_mode=trig_on;
        #endif 
      }
      
          

//      judge_if_heat_full(mode);
      break;
      
    }
    case 3:
    {
      mode->shoot_mode=shoot_no;
      mode->trig_mode=trig_off;
      break;
    }
    case 2:
    {
     mode->shoot_mode=shoot_no;
      mode->trig_mode=trig_off; 
     break;
    }
   }
}




void remote_offline_set(sentry_system_t *mode)
{
      mode->chassis_set.set_vx=0;
      mode->chassis_set.set_vy=0;

      mode->set_yaw_in=0;
      mode->chassis_mode=no_move;
      mode->set_yaw_in=0;
      mode->shoot_mode=shoot_no;
}

int last_rc_wheel=0;

void choose_control_mode(sentry_system_t *mode)
{
   if(rc_ctrl.rc.wheel<-400&&game_state.stage_remain_time<420)
   {
     mode->control_mode=auto_mode;
   }
   else if(rc_ctrl.rc.wheel<200&&last_rc_wheel>200)
   {
    mode->control_mode=rc_mode;
   }
   last_rc_wheel=rc_ctrl.rc.wheel;
}



//后期开始写决策，前期的只用判断模式



void Auto_small_gimbal_mode(sentry_system_t *mode)
{
 
  if(robot_status.power_management_gimbal_output==0)
  {
   mode->small_gimbal_mode=small_gimbal_off;
         mode->if_small_pitch_can=0;
         mode->vision_mode=vision_off;
  }
  else 
  {
    if(rc_ctrl.rc.s_l!=2)
    {
     mode->small_gimbal_mode=small_gimbal_pc;
     mode->if_small_pitch_can=1;
     mode->vision_mode=vision_on;
    }
    else 
    {
     mode->small_gimbal_mode=small_gimbal_off;
     mode->if_small_pitch_can=0;
     mode->vision_mode=vision_off;
    }
    
    
  }
}


void small_gimbal_mode_chose(sentry_system_t *mode)
{

  if(rc_ctrl.rc.s_l==1)
  {
    switch(rc_ctrl.rc.s_r)
      {
       case 1:
       {
         
         
        if(game_state.game_progress==4)
        {
          if(decision.Judge_condition.IF_Arrived==1)
         {
           mode->small_gimbal_mode=small_gimbal_pc;
           mode->if_small_pitch_can=1;
           mode->vision_mode=vision_on;
         }
         else
         {
           mode->small_gimbal_mode=small_gimbal_pc;
           mode->if_small_pitch_can=1;
           mode->vision_mode=vision_on;
          
         }
        }
        else
        {
          mode->small_gimbal_mode=small_gimbal_pc;
           mode->if_small_pitch_can=1;
           mode->vision_mode=vision_on;
         
        }
         break;
       }
       case 3:
       {
         mode->small_gimbal_mode=small_gimbal_rc;
         mode->if_small_pitch_can=1;
         mode->vision_mode=vision_off;
         break;
       }
       case 2:
       {
         mode->small_gimbal_mode=small_gimbal_off;
         mode->if_small_pitch_can=0;
         mode->vision_mode=vision_off;
         break;
       }
      }
      
  }
  else if(rc_ctrl.rc.s_l==3)
  {
   switch(rc_ctrl.rc.s_r)
      {
       case 1:
       {
         mode->small_gimbal_mode=small_gimbal_rc;
         mode->if_small_pitch_can=1;
         break;
       }
       case 3:
       {
         mode->small_gimbal_mode=small_gimbal_rc;
         mode->if_small_pitch_can=1;
         break;
       }
       case 2:
       {
         mode->small_gimbal_mode=small_gimbal_off;
         mode->if_small_pitch_can=0;
         break;
       }
      }
      
      mode->vision_mode=vision_off;
  }
  else 
  {
  mode->small_gimbal_mode=small_gimbal_off;
         mode->if_small_pitch_can=0;
         mode->vision_mode=vision_off;
  }
  if(robot_status.power_management_gimbal_output==0)
  {
   mode->small_gimbal_mode=small_gimbal_off;
         mode->if_small_pitch_can=0;
         mode->vision_mode=vision_off;
  }
  
  
}