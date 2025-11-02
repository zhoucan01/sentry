#include "bsp_transmit.h"
#include "cmsis_os.h"
#include "system.h"
#include "remote_control.h"
#include <stdlib.h>
#include "referee.h"
#include "stdbool.h"
sentry_system_t sentry_system;
robot_data_t robot_data;
decision_t decision;
void system_run()
{
  for(;;)
  {
  
  
  if(get_if_communite_broke()==1)
  {
  remote_offline_set(&sentry_system);
  }
  else
  {
    choose_control_mode(&sentry_system);
    if(sentry_system.control_mode==rc_mode)
    {
      AGV_mode_chose(&sentry_system);
      AGV_big_yaw(&sentry_system);
      shoot_mode_chose(&sentry_system);
      small_gimbal_mode_chose(&sentry_system);
    }
    else if(sentry_system.control_mode==auto_mode)
    {
     AGV_mode_chose(&sentry_system);
      AGV_big_yaw(&sentry_system);
      shoot_mode_chose(&sentry_system);
      small_gimbal_mode_chose(&sentry_system);
    }
  
    
  }
   
   sentry_decision(&decision);
   
   
    vTaskDelay(1);
  }
}





void AGV_mode_chose(sentry_system_t *mode)
{
  switch(rc_ctrl.rc.s_l)
  {
    case 2:
    {
    mode->chassis_mode=no_move;
    break;
    }
    case 3:
    {
    mode->chassis_mode=follow_move;
    break;
    }
    case 1:
    {
    if(game_state.game_progress==4)
    {
     if(USART_TX_data.if_outpost_des==0)
     {
       mode->chassis_mode=lock_move;
     }
     else 
     {
      mode->chassis_mode=spine_move;
     }
    }
    else mode->chassis_mode=spine_move;
    
    break;
    }
  }
  /**
  *@note 这里将遥控器输入速度转换成m/s的单位与导航单位一致
  */
  switch(mode->chassis_mode)
  {
    case no_move:
    {
      mode->chassis_set.set_vx=0;
      mode->chassis_set.set_vy=0;
      mode->chassis_set.set_wz=0;
      break;
    }
    case follow_move:
    {
     mode->chassis_set.set_vx=rc_ctrl.rc.ch1*2.5*WHEEL_FACTOR;
     mode->chassis_set.set_vy=rc_ctrl.rc.ch0*2.5*WHEEL_FACTOR;
     
//     if(mode->chassis_set.set_vx!=0&&mode->chassis_set.set_vy!=0)
//     {
//      mode->chassis_set.set_vx*=(1/2);
//      mode->chassis_set.set_vy*=(1/2);
//     }
     break;
     
    }
    case spine_move:
    {
     mode->chassis_set.set_vx=rc_ctrl.rc.ch1*2.5*WHEEL_FACTOR;
     mode->chassis_set.set_vy=rc_ctrl.rc.ch0*2.5*WHEEL_FACTOR;
     
//     if(mode->chassis_set.set_vx!=0&&mode->chassis_set.set_vy!=0)
//     {
//      mode->chassis_set.set_vx*=(1/2);
//      mode->chassis_set.set_vy*=(1/2);
//     }
     break;
    }
    case lock_move:
    {
      mode->chassis_set.set_vx=rc_ctrl.rc.ch1*2.5*WHEEL_FACTOR;
     mode->chassis_set.set_vy=rc_ctrl.rc.ch0*2.5*WHEEL_FACTOR;
     break;
    }
  }
//  chassis_move_limit(mode);
}

void AGV_big_yaw(sentry_system_t *mode)
{
  switch(rc_ctrl.rc.s_r)
  {
    case 1:
    {
      
    
      //导航控制或者视觉控制
      mode->set_yaw_in=rc_ctrl.rc.ch2*big_yaw_kp;
      break;
    }
    case 3:
    {
      mode->set_yaw_in=rc_ctrl.rc.ch2*big_yaw_kp;
      break;
    }
    case 2:
    {
      mode->set_yaw_in=rc_ctrl.rc.ch2*big_yaw_kp;
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
    mode->shoot_mode=shoot_on;
    break;
      
    }
    case 3:
    {
      mode->shoot_mode=shoot_no;
      break;
    }
    case 2:
    {
     mode->shoot_mode=shoot_no; 
     break;
    }
   }
}


void remote_offline_set(sentry_system_t *mode)
{
      mode->chassis_set.set_vx=0;
      mode->chassis_set.set_vy=0;
      mode->chassis_set.set_wz=0;
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
void auto_mode_set(sentry_system_t *mode)
{
  mode->chassis_mode=spine_move;
  mode->vision_mode=vision_on;
  
  
}


//void chassis_move_limit(sentry_system_t *mode)
//{
//  
//  if(mode->chassis_set.set_vx>-50&&mode->chassis_set.set_vx<50)
//  { 
//    mode->chassis_set.set_vx=0;
//  }
//  
//  if(mode->chassis_set.set_vy>-50&&mode->chassis_set.set_vy<50)
//  { 
//    mode->chassis_set.set_vy=0;
//  }
//}

void small_gimbal_mode_chose(sentry_system_t *mode)
{




  if(rc_ctrl.rc.s_l==1)
  {
    switch(rc_ctrl.rc.s_r)
      {
       case 1:
       {
         mode->small_gimbal_mode=small_gimbal_pc;
         mode->if_small_pitch_can=1;
         mode->vision_mode=vision_on;
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



//static bool Judge_Vision_IF_Shoot_Balance( int16_t Robot_HP )
//{
//  bool res;
//	
//	 static  uint16_t robot_tim3  = 0;      
//	 static  uint16_t robot_tim10 = 0;     
//	 static  uint16_t Last_HP;
//	
//	  if( Robot_HP < 60 && Last_HP == 0 && Robot_HP > 0)      //所有车体的上限血量的10%均小于60
//			robot_tim10  = game_state.stage_remain_time;
//		
//		if( Robot_HP >= 150 && Last_HP == 0 )            				//所有车体的上限血量均大于等于150
//			robot_tim3 = game_state.stage_remain_time;

//		if( ((robot_tim10 - game_state.stage_remain_time < 10) && ( robot_tim10 - game_state.stage_remain_time > 0 ))
//			|| (robot_tim3  - game_state.stage_remain_time < 3   && ( robot_tim3  - game_state.stage_remain_time > 0 ) ) || Robot_HP == 0 || game_state.game_progress != 4 )//
//			res = 0; 
//    else 	
//      res = 1;
// 
//		
//	Last_HP = Robot_HP;
//		return res;

//}




void sentry_decision(decision_t *mode)
{
  judge_if_shoot(mode);
}


void judge_if_shoot(decision_t *mode)
{

  if(robot_data.robot_color==red)
  {

    mode->Vision_ByteBits.shoot_Hero=judge_if_shoot_hero(game_robot_HP.blue_1_robot_HP);
    mode->Vision_ByteBits.shoot_Engineer=judge_if_shoot_Engineer(game_robot_HP.blue_2_robot_HP);
    mode->Vision_ByteBits.shoot_infantr3=judge_if_shoot_infantr3(game_robot_HP.blue_3_robot_HP);
    mode->Vision_ByteBits.shoot_infantr4=judge_if_shoot_infantr4(game_robot_HP.blue_4_robot_HP);
    mode->Vision_ByteBits.shoot_Sentry= judge_if_shoot_sentry(game_robot_HP.blue_7_robot_HP,game_robot_HP.blue_outpost_HP);
    mode->Vision_ByteBits.shoot_base=judge_if_shoot_base(game_robot_HP.blue_outpost_HP);
  }
  else 
  {
    mode->Vision_ByteBits.shoot_Hero=judge_if_shoot_hero(game_robot_HP.red_1_robot_HP);
    mode->Vision_ByteBits.shoot_Engineer=judge_if_shoot_Engineer(game_robot_HP.red_2_robot_HP);
    mode->Vision_ByteBits.shoot_infantr3=judge_if_shoot_infantr3(game_robot_HP.red_3_robot_HP);
    mode->Vision_ByteBits.shoot_infantr4=judge_if_shoot_infantr4(game_robot_HP.red_4_robot_HP);
    mode->Vision_ByteBits.shoot_Sentry= judge_if_shoot_sentry(game_robot_HP.red_7_robot_HP,game_robot_HP.red_outpost_HP);
    mode->Vision_ByteBits.shoot_base=judge_if_shoot_base(game_robot_HP.red_outpost_HP);
    
  }
}


static bool judge_if_shoot_hero(int16_t robot_hp)
{
  bool res;
  
  static uint16_t wait_tim3 = 0;
  static uint16_t wait_tim10 = 0;
  static uint16_t last_hp;
  
  if(robot_hp<60 && last_hp==0)
  {
   wait_tim10 = game_state.stage_remain_time;
  }
  
  if(robot_hp>=150 && last_hp == 0)
  {
    wait_tim3 = game_state.stage_remain_time;   
   }
   
   if(((wait_tim10-game_state.stage_remain_time<10)&&(wait_tim10-game_state.stage_remain_time>0))
      |((wait_tim3-game_state.stage_remain_time<3)&&(wait_tim3-game_state.stage_remain_time>0))
      |(robot_hp==0)
      |(game_state.game_progress!=4))
      {
       res=0;
      }
    else 
      res=1;
   
   last_hp = robot_hp;
   
   return res;
}


static bool judge_if_shoot_Engineer(int16_t robot_hp)
{
  bool res;
  
  static uint16_t wait_tim3 = 0;
  static uint16_t wait_tim10 = 0;
  static uint16_t last_hp;
  
  if(robot_hp<60 && last_hp==0)
  {
   wait_tim10 = game_state.stage_remain_time;
  }
  
  if(robot_hp>=150 && last_hp == 0)
  {
    wait_tim3 = game_state.stage_remain_time;   
   }
   
   if(((wait_tim10-game_state.stage_remain_time<10)&&(wait_tim10-game_state.stage_remain_time>0))
      ||((wait_tim3-game_state.stage_remain_time<3)&&(wait_tim3-game_state.stage_remain_time>0))
      ||(robot_hp==0)
      ||(game_state.game_progress!=4)||(game_state.stage_remain_time>180))
      {
       res=0;
      }
    else 
      res=1;
   
   last_hp = robot_hp;
   
   return res;
  
  
}

static bool judge_if_shoot_infantr3(int16_t robot_hp)
{
  bool res;
  
  static uint16_t wait_tim3 = 0;
  static uint16_t wait_tim10 = 0;
  static uint16_t last_hp;
  
  if(robot_hp<60 && last_hp==0)
  {
   wait_tim10 = game_state.stage_remain_time;
  }
  
  if(robot_hp>=150 && last_hp == 0)
  {
    wait_tim3 = game_state.stage_remain_time;   
   }
   
   if(((wait_tim10-game_state.stage_remain_time<10)&&(wait_tim10-game_state.stage_remain_time>0))
      ||((wait_tim3-game_state.stage_remain_time<3)&&(wait_tim3-game_state.stage_remain_time>0))
      ||(robot_hp==0)
      ||(game_state.game_progress!=4))
      {
       res=0;
      }
    else 
      res=1;
   
   last_hp = robot_hp;
   
   return res;
  
  
}

static bool judge_if_shoot_infantr4(int16_t robot_hp)
{
  bool res;
  
  static uint16_t wait_tim3 = 0;
  static uint16_t wait_tim10 = 0;
  static uint16_t last_hp;
  
  if(robot_hp<60 && last_hp==0)
  {
   wait_tim10 = game_state.stage_remain_time;
  }
  
  if(robot_hp>=150 && last_hp == 0)
  {
    wait_tim3 = game_state.stage_remain_time;   
   }
   
   if(((wait_tim10-game_state.stage_remain_time<10)&&(wait_tim10-game_state.stage_remain_time>0))
      ||((wait_tim3-game_state.stage_remain_time<3)&&(wait_tim3-game_state.stage_remain_time>0))
      ||(robot_hp==0)
      ||(game_state.game_progress!=4))
      {
       res=0;
      }
    else 
      res=1;
   
   last_hp = robot_hp;
   
   return res;
  
  
}


static bool judge_if_shoot_sentry(int16_t robot_hp,int16_t outpost_HP)
{
  bool res;
  
  static uint16_t wait_tim3 = 0;
  static uint16_t wait_tim10 = 0;
  static uint16_t last_hp;
  
  if(robot_hp<60 && last_hp==0)
  {
   wait_tim10 = game_state.stage_remain_time;
  }
  
  if(robot_hp>=150 && last_hp == 0)
  {
    wait_tim3 = game_state.stage_remain_time;   
   }
   
   if(((wait_tim10-game_state.stage_remain_time<10)&&(wait_tim10-game_state.stage_remain_time>0))
      ||((wait_tim3-game_state.stage_remain_time<3)&&(wait_tim3-game_state.stage_remain_time>0))
      ||(robot_hp==0)
      ||(game_state.game_progress!=4)
      ||(outpost_HP>0) )
     
      {
       res=0;
      }
    else 
      res=1;
   
   last_hp = robot_hp;
   
   return res;
  
  
}

bool judge_if_shoot_base(int16_t outpost_hp)
{
  bool res=0;
  if(outpost_hp>0)
  {
    res=0;
  }
  else res=1;
  
  return res;
}