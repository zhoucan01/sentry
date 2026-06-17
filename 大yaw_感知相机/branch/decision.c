
/**
  ************************************* Copyright ****************************** 
  * FileName   : decision.c   
  * Version    : v2.1	
  * Author     : 周灿
  * Number     : 15271187610 	
  * Date       : 2025-8-1   
  * Description:    哨兵电控决策，后续改到上位机上，调试较为困难且复杂，上限太低了
  * Function List:  
  	1. ....
  	   <version>: 		
  <modify staff>:
  		  <data>:
   <description>:  
  	2. ...
  ******************************************************************************
 *根据建图的起始点来改代码  下列是所涉及到的协议，但是目前坐标系很不统一，比较麻烦
 1云台手标点坐标需要修改map_control_fill
 2云台手小地图哨兵路径提示需要改Path_display
 3导航点推送至导航端Navigation_Tx_Send(&navigation_tx);
 4所有涉及到目前哨兵位置的也就是导航发送过来的当前位置都需要看,因为当前导航发过来的坐标是里程计坐标，也就是从程序运行起来相对于你程序起始点的坐标而不是相对于零点的坐标，后续有可能更换，目前不确定
*******************************************************************/

#include "decision.h"
#include "bsp_transmit.h"
#include "cmsis_os.h"
#include "system.h"
#include "remote_control.h"
#include <stdlib.h>
#include "referee.h"
#include "stdbool.h"
#include "Nautilus_Vision.h"
#include "CAN_receive.h"
#include "navigation.h"
#include "struct_typedef.h"
#include "stdbool.h"
#include "Nautilus_UI.h"
#include "string.h"
#include "math.h"
#include "navigation.h"
#include "Nautilus_UI.h"
#include "ins_task.h"
#include "Sentry_cmd.h"
/*导航点决策*/

decision_t decision;
Sentry_cmd_t Sentry_cmd_send;

float Red_Navi_position[DECISION_POSITION_NUM][2];



//每场场前确定打不打工程
//#define if_shoot_Engineer
//建图后确定到底是哪方建图的
#define RED_START_NAVIGATION // 确定红方开始建图
//提前写好蓝方建图和红方建图的代码在家里测试好了之后，去了比赛就可以快速修改
//总共14个点位

//float Red_Navi_position[DECISION_POSITION_NUM][2]  = { {  3.79   , 7.99 },//9.96 3.69
//                                                       {  2.42  , 2.35 },
//                                                       {  10.35 , 14.27 },//10.75 
//                                                       {  8.5 , 7.5 },
//                                                       {  8.5  , 7.5 },
//                                                       {  6.65 , 9.1 },
//                                                       {  6.28 , 6.84 },
//                                                       {  15.75,9.39},
//                                                       {  21.18, 5.67},
//                                                       {  23.80, 7.59},
//                                                       {  11.43, 4.36},
//                                                       {  21.18, 5.67},
//                                                       {  13.48, 8.98},
//                                                       {  8.9 , 7.5}};
float Red_Navi_position[DECISION_POSITION_NUM][2]  = { {  0   , 0 },//9.96 3.69
                                                       {  2.42  , -2.35 },//训练
                                                       {  10.35 , 14.27 },//10.75 
                                                       {  8.5 , 7.5 },
                                                       {  8.5  , 7.5 },
                                                       {  6.65 , 9.1 },
                                                       {  6.28 , 6.84 },
                                                       {  15.75,9.39},
                                                       {  21.18, 5.67},
                                                       {  23.80, 7.59},
                                                       {  11.43, 4.36},
                                                       {  21.18, 5.67},
                                                       {  4,-3},//训练
                                                       {  8.9 , 7.5}};



int iiipp;
int huug;
 int hurt_time=0;
/* ====== 自主决策主任务（FreeRTOS入口） ====== */
void Auto_run(void const * argument)
{
 Decision_Init(&decision);
 hurt_time=420;

  
 decision.If_point_change=0;
 memset(&decision,0,sizeof(decision_t));
 decision.Cmd_condition.if_update=1;
 memset(&Sentry_cmd_send,0,sizeof(Sentry_cmd_t));

//decision.decision_mode=guard;
 huug++;
// decision.Judge_condition.IF_Arrived=

 #ifdef if_shoot_Engineer
 Red_Navi_position[ENEMY_OUTPOST_PROTECT_POINT][0]=12.09;
 Red_Navi_position[ENEMY_OUTPOST_PROTECT_POINT][1]=8.9;
 
 #endif
   for(;;)
   {
   
     //导航决策处理
     Decison_State_Ctl(&decision);

     //击打决策处理
     sentry_shoot_decision(&decision);
     //解析裁判系统信息
     get_referr_data();
     
     
     //哨兵自主决策相关指令处理
     Sentry_cmd_decision(&decision);
     
     //导航点填充
     decision_point_fill();
     
     //决策点推送至导航
     Navigation_Tx_Send(&navigation_tx);
     vTaskDelay(1);
   }
}
int mnmnk;
/* ====== 自动模式选择（遥控器开关组合 -> 决策模式） ====== */
void AGV_auto_mode(decision_t *mode)
{

//mnmnk++;
  if(game_state.game_progress==4)
  {
  
    if(rc_ctrl.rc.s_l==1&&rc_ctrl.rc.s_r==1)
    {
      mode->decision_mode=extreme;
    }
     else if(rc_ctrl.rc.s_l==3&&rc_ctrl.rc.s_r==3)
    {
      mode->decision_mode=conservative;
    }
    else if(rc_ctrl.rc.s_l==3&&rc_ctrl.rc.s_r==2)
    {
      mode->decision_mode=flying;
    }   
    else if(rc_ctrl.rc.s_l==3&&rc_ctrl.rc.s_r==1)
    {
      mode->decision_mode=patrol;
    }    
    else if(rc_ctrl.rc.s_l==1&&rc_ctrl.rc.s_r==3)
    {
      mode->decision_mode=protect;
    }
    
  }
  else 
  {
  
//mnmnk++;
      mode->decision_mode=extreme;
  }
}

/* ====== 获取裁判系统数据（机器人颜色） ====== */
void get_referr_data()
{
 if( robot_status.robot_id <= 9 && robot_status.robot_id >0 )
		 decision.robot_data.robot_color = red ;
		 else if( robot_status.robot_id >= 101)
			decision.robot_data.robot_color = blue ;
		 else
			decision.robot_data.robot_color = NO_CONTACT ;
      
      
      if(decision.robot_data.robot_color==red)
      {
        Robot_ID=UI_Data_RobotID_RSentry;
      }
      else Robot_ID = UI_Data_RobotID_BSentry;
}



//哨兵自主决策相关指令
uint16_t Last_HP;
uint16_t Last_projectile_allowance_17mm;
/* ====== 哨兵指令决策（复活/兑换弹丸/兑换血量） ====== */
void Sentry_cmd_decision(decision_t *mode)
{

  if(game_state.game_progress==4)
  {
      mode->Cmd_condition.Exchange_Projectile_Num=0;
      mode->Cmd_condition.If_remote_exchange_HP=0;
      mode->Cmd_condition.If_Immediately_Revive=0;
      
      if(robot_status.current_HP==0&&Last_HP>0)
      {
        mode->Cmd_condition.Die_cnt++;
      }
      

      mode->Cmd_condition.If_revive=1;
      //裁判系统响应了哨兵的自主买弹

      mode->Cmd_condition.Exchange_Projectile_Num=0;
      
      Sentry_Cmd_Fill(&Sentry_cmd_send,mode->Cmd_condition.If_revive,mode->Cmd_condition.If_Immediately_Revive,
                        mode->Cmd_condition.Exchange_Projectile_Num,mode->Cmd_condition.If_remote_exchange_HP);
      
      Last_HP=robot_status.current_HP;
      Last_projectile_allowance_17mm=projectile_allowance.projectile_allowance_17mm;
      
      
  }
  else 
  {
    memset(&mode->Cmd_condition,0,sizeof(Cmd_condition_t));
    Sentry_Cmd_Fill(&Sentry_cmd_send,mode->Cmd_condition.If_revive,mode->Cmd_condition.If_Immediately_Revive,
                        mode->Cmd_condition.Exchange_Projectile_Num,mode->Cmd_condition.If_remote_exchange_HP);
  }
  
  

}


int kmnjnk;
/* ====== 决策初始化 ====== */
void Decision_Init(decision_t *mode)
{
kmnjnk++;

  mode->point=INIT_PACK_POINT;
}

//
//目前是分区赛决策
/* ====== 决策状态主控制器 ====== */
void Decison_State_Ctl(decision_t *mode)
{

  if(game_state.game_progress==4)
  {
  //决策判断条件处理
     Judge_Continuous_Handle(&decision.Judge_condition);
     
     
     //决策的模式处理，根据遥控器来提前确认决策逻辑
     AGV_auto_mode(mode);
     
     
   //决策模式处理（自主决策加云台手决策）
     decision_point_chose(mode);
     

  }
  else 
  {
  
  
  
  //决策条件清零
  mode->decision_mode=extreme;
//  mode->decision_mode=guard;
  
  If_Point_arrived();
  judge_if_location_over(&decision.Judge_condition);
//  mode->Judge_condition.IF_Arrived = judge_if_location_over();
  mode->Judge_condition.IF_10s_NotHurted = 1;
  mode->Judge_condition.IF_3s_NotFound = 1;
  mode->Judge_condition.IF_5s_NotFound = 1;
  mode->Judge_condition.IF_10s_NotFound = 1;
  mode->Judge_condition.IF_HP_Less_50 = 0;
  mode->Judge_condition.IF_HP_Less_100 = 0;
  mode->Judge_condition.IF_outpost_destroyed = 0;
  mode->Judge_condition.IF_fire_lock = 0;
  mode->Judge_condition.IF_allowance_less_50 = 0;
  mode->Judge_condition.IF_HP_recover = 1;
  mode->Judge_condition.If_enemy_outpost_lock=0;
//  mode->Judge_condition.If_on_toss=
//  mode->Judge_condition.If_moving_v=
  judge_if_on_toss(&mode->Judge_condition);
  judge_if_moving_v(&mode->Judge_condition);
  mode->Judge_condition.If_chassis_weak=0;
  mode->Judge_condition.IF_need_to_protect=0;
  judge_if_need_to_protect(&decision.Judge_condition);
  mode->Judge_condition.IF_base_armor_spred=0;
  mode->Judge_condition.If_fortress_free=1;
  mode->Judge_condition.If_get_allow_17=0;
  mode->Judge_condition.If_chip_base=0;
  mode->Judge_condition.IF_3s_NotHurted=1;
  mode->Judge_condition.IF_5s_NotHurted=1;
  
  mode->Judge_condition.If_close_to_enemy_out=0;
  judge_if_moving_v(&mode->Judge_condition);
  

  lock_dart_count=0;
  if_random_dart=0;
  last_dart_time=0;
  if_update=0;
  hurt_time=430;
  Last_base_hurt_time=430;
  decision.keyboard_disable=0;
      memset(&map_command,0,sizeof(map_command));
//   sentry_extreme_decision(mode);
  //导航至启动区
// mode->point= ENEMY_OUTPOST_POINT;
  
  //决策的模式处理，根据遥控器来提前确认决策逻辑
//     AGV_auto_mode(mode);
     
     
   //决策模式处理（自主决策加云台手决策）
//     decision_point_chose(mode);
//  mode->point=WE_FORTRESS_POINT;

  
//     sentry_air_control_decision(mode);
  
  
  
mode->point=INIT_PACK_POINT;
////mode->point=WE_DEPOT_POINT;
  }
  
  
  
}
uint8_t last_decision_point;

/* 判断是否距离敌方前哨站较近 */
void judge_if_close_to_enemy_out(Judge_condition_t *mode)
{
  if((fabs(navigation_rx.current_x-navigation_tx.navi_set_x_pos)<1.5
   &&fabs(navigation_rx.current_y-navigation_tx.navi_set_y_pos)<1.5)
 ||mode->IF_Arrived==1)
   {
     mode->If_close_to_enemy_out=1;
   }
   else mode->If_close_to_enemy_out=0;
}

/* 判断键盘一键失能 */
void judge_if_keyboard_disable(decision_t *mode)//判断是否一键失能
{
   if(if_update==1&&map_command.cmd_keyboard=='D')
   {
     mode->keyboard_disable=1;     
   }
   else if(if_update==1&&map_command.cmd_keyboard=='W')
   {
     mode->keyboard_disable=0;
   }
}

void sentry_test_decision(decision_t *mode)
{
  if(mode->Judge_condition.IF_HP_Less_100)
  {
    mode->point=WE_DEPOT_POINT;
  }
  else 
  {
    mode->point=CENTRL_HIGH_POINT;  
  }
}

int get_clear_count=0;
//uint8_t navi_state_get;
uint8_t if_navi_receive=0;
int rx_point_lose;

/* ====== 决策点位选择（云台手标点 -> 模式路由） ====== */
void decision_point_chose(decision_t *mode)
{

//  static uint8_t last_point;
  if(if_update==1)//响应云台手点位
  {
    mode->decision_mode=air_control;
    
       // 判断是否一键失能
    judge_if_keyboard_disable(mode);
//    if_update=0;
  }
  
  if(if_update==1&&if_map_correct==1)
  {
    map_control_fill(mode);
  }
  
 //决策变换后将点位初始化
  if(mode->decision_mode!=mode->last_decision_mode)
  {
    mode->last_decision_mode=mode->decision_mode;
    mode->point=INIT_PACK_POINT;
  }
  
  sentry_test_decision(mode);
  
  //点位决策
  
}




void sentry_air_control_decision(decision_t *mode)
{
  switch (mode->point)
   {
      case MANUAL_POINT:
      {
         if(mode->Judge_condition.IF_HP_Less_100==1||mode->Judge_condition.If_get_allow_17==1)
        {
          mode->point=WE_DEPOT_POINT;
        }
        break;
      }
      case WE_DEPOT_POINT:
      {
      
        if((mode->Judge_condition.IF_Arrived==1&&mode->Judge_condition.IF_HP_recover==1))
        {
           mode->point=MANUAL_POINT;
        }
        break;
      }
      default:
      {
      kmnjnk++;
        mode->point=MANUAL_POINT;
        break;
      }
   }
}


//后续主要维护决策
/* ====== 激进模式决策（主战模式） ====== */
void sentry_extreme_decision(decision_t *mode)
{
  switch(mode->point)
  {
    case INIT_PACK_POINT:
    mode->point=ENEMY_OUTPOST_POINT;
    break;
    case ENEMY_OUTPOST_POINT:
    {
      if(mode->Judge_condition.IF_HP_Less_50||mode->Judge_condition.If_get_allow_17)
      {
        mode->point=WE_DEPOT_POINT;
      }
      else if((!mode->Judge_condition.IF_3s_NotHurted&&mode->Judge_condition.If_hp_less_200))
      {
        mode->point=CENTRL_HIGH_POINT;
      }
      else if(mode->Judge_condition.IF_HP_Less_100&&!mode->Judge_condition.IF_outpost_destroyed)
      {
        mode->point=WE_OUTPOST_POINT;
      }
      else if(((mode->Judge_condition.IF_outpost_destroyed&&mode->Judge_condition.IF_need_to_protect)
              ||mode->Judge_condition.IF_base_armor_spred))
      {
        mode->point=WE_PATROL_POINT;
      }
      else if(mode->Judge_condition.IF_enemy_outpost_destroyed&&!mode->Judge_condition.IF_allowance_less_50
              &&!mode->Judge_condition.If_chassis_weak)
      {
        mode->point=ENEMY_OUTPOST_PROTECT_POINT;
      }
      else if(mode->Judge_condition.IF_enemy_outpost_destroyed&&mode->Judge_condition.IF_allowance_less_50)
      {
        mode->point=WE_PATROL_POINT;
      }
      break;
    }
    case WE_DEPOT_POINT:
    {
      if((mode->Judge_condition.IF_Arrived==1&&mode->Judge_condition.IF_HP_recover==1))
      { 
        if(mode->Judge_condition.If_chassis_weak)
        {
         mode->point=WE_PATROL_POINT;
        }
        else if(((mode->Judge_condition.IF_outpost_destroyed&&mode->Judge_condition.IF_need_to_protect)
              ||mode->Judge_condition.IF_base_armor_spred)
              &&!mode->Judge_condition.If_chassis_weak)
        {
          mode->point=WE_FORTRESS_POINT;
        }
        else if(!mode->Judge_condition.IF_enemy_outpost_destroyed&&!mode->Judge_condition.IF_allowance_less_100)
        {
          mode->point=ENEMY_OUTPOST_POINT;
        }
        else if(mode->Judge_condition.IF_allowance_less_100)
        {
          mode->point=WE_PATROL_POINT;
        }
        
        else mode->point=WE_PATROL_POINT;
      }
      break;
    }
    case WE_PATROL_POINT:
    {
      if(mode->Judge_condition.IF_HP_Less_100||mode->Judge_condition.If_get_allow_17)
      {
        mode->point=WE_DEPOT_POINT;
      }
      else if(!mode->Judge_condition.If_chassis_weak)
      {
         if(((mode->Judge_condition.IF_outpost_destroyed&&mode->Judge_condition.IF_need_to_protect)
                ||mode->Judge_condition.IF_base_armor_spred)
                &&!mode->Judge_condition.If_chassis_weak)
        {
          mode->point=WE_FORTRESS_POINT;
        }
        else if(!mode->Judge_condition.IF_allowance_less_100)
        {
          mode->point=ENEMY_OUTPOST_POINT;
        }
      }
      break;
    }
    case CENTRL_HIGH_POINT:
    {
       if(mode->Judge_condition.IF_3s_NotHurted)
       {
         mode->point=ENEMY_OUTPOST_POINT;
       }
       break;
    }
    case WE_FORTRESS_POINT:
    {
       if(mode->Judge_condition.IF_HP_Less_100)
      {
        mode->point=WE_DEPOT_POINT;
      }
      else if(!mode->Judge_condition.IF_need_to_protect&&!mode->Judge_condition.IF_base_armor_spred)//2025/6/28 添加下堡垒的条件!mode->Judge_condition.IF_base_armor_spred
      {
        mode->point=WE_PATROL_POINT;
      }
      
      break;
    }
    case WE_OUTPOST_POINT:
    {
       if(mode->Judge_condition.IF_HP_Less_50||mode->Judge_condition.If_get_allow_17)
       {
         mode->point=WE_DEPOT_POINT;
       }
       else if(mode->Judge_condition.IF_HP_recover)
       {
         mode->point=ENEMY_OUTPOST_POINT;
       }

       break;
    }
    case ENEMY_OUTPOST_PROTECT_POINT:
    {
      if(mode->Judge_condition.IF_HP_Less_50||mode->Judge_condition.If_get_allow_17)
      {
        mode->point=WE_DEPOT_POINT;
      }
      else if(mode->Judge_condition.IF_HP_Less_100&&!mode->Judge_condition.IF_outpost_destroyed)
      {
        mode->point=WE_OUTPOST_POINT;
      }
      else if(((mode->Judge_condition.IF_outpost_destroyed&&mode->Judge_condition.IF_need_to_protect)
              ||mode->Judge_condition.IF_base_armor_spred)
              &&!mode->Judge_condition.If_chassis_weak)// 7/8修改从敌方英雄打前哨站的位置到堡垒
      {
        mode->point=WE_PATROL_POINT;
      }
      else if(mode->Judge_condition.If_need_to_enemy_fortress)
      {
        mode->point=ENEMY_FORTRESS_POINT;
      }

      break;
    }
    case ENEMY_FORTRESS_POINT:
    {
      if(mode->Judge_condition.IF_HP_Less_50||mode->Judge_condition.If_get_allow_17)
      {
        mode->point=WE_DEPOT_POINT;
      }
      else if(mode->Judge_condition.IF_HP_Less_100&&!mode->Judge_condition.IF_outpost_destroyed)
      {
        mode->point=WE_OUTPOST_POINT;
      }
      else if(!mode->Judge_condition.If_need_to_enemy_fortress)
      {
        mode->point=ENEMY_OUTPOST_PROTECT_POINT;
      }
      break;
      
    }
    
    default:
    {
      mode->point=INIT_PACK_POINT;
        break;
    }

  
  }
}
int kofjfs;
//稍微没有那么激进的决策 考虑是否添加打前哨 WE_PATROL_POINT WE_FORTRESS_POINT WE_FLYING_POINT
/* ====== 保守模式决策 ====== */
void sentry_conservative_decision(decision_t *mode)
{
  
  
  kofjfs++;
  switch(mode->point)
  {
    case INIT_PACK_POINT:
    {
      mode->point=WE_FLYING_POINT;
      break;
    }
    case WE_FLYING_POINT:
    {
      if(mode->Judge_condition.IF_HP_Less_100||mode->Judge_condition.If_get_allow_17)
      {
        mode->point=WE_DEPOT_POINT;
      }
      else if(mode->Judge_condition.IF_enemy_outpost_destroyed)
      {
        mode->point=WE_PATROL_POINT;
      }
     break;
    }
    case WE_PATROL_POINT:
    {
      if(mode->Judge_condition.IF_HP_Less_100||mode->Judge_condition.If_get_allow_17)
      {
        mode->point=WE_DEPOT_POINT;
      }
      else if(mode->Judge_condition.IF_outpost_destroyed)
      {
        mode->point=WE_FORTRESS_POINT;
      }
      break;
    }
    case WE_DEPOT_POINT:
    {
      if(mode->Judge_condition.IF_Arrived&&mode->Judge_condition.IF_HP_recover)
      {
        if(!mode->Judge_condition.IF_enemy_outpost_destroyed)
        {
          mode->point=WE_FLYING_POINT;
        }
        else if(!mode->Judge_condition.IF_outpost_destroyed)
        {
          mode->point=WE_PATROL_POINT;
        }
        else if(mode->Judge_condition.IF_outpost_destroyed)
        {
          mode->point=WE_FORTRESS_POINT;
        }
      }

      
      break;
    }
    case WE_FORTRESS_POINT:
    {
      if(mode->Judge_condition.IF_HP_Less_50)
      {
        mode->point=WE_DEPOT_POINT;
      }
      break;
    }
    default:
    {
      mode->point=INIT_PACK_POINT;
    }
    
    
  }
}

//只打前哨站不去敌方家里面不上堡垒 留一版保证能实现的所有功能 ENEMY_OUTPOST_POINT CENTRL_HIGH_POINT WE_PATROL_POINT
/* ====== 巡逻模式决策（只打前哨站） ====== */
void sentry_patrol_decision(decision_t *mode)
{
  
  switch(mode->point)
  {
    case INIT_PACK_POINT:
    {
      mode->point=ENEMY_OUTPOST_POINT;
      break;
    }
    case ENEMY_OUTPOST_POINT:
    {
      if(mode->Judge_condition.IF_HP_Less_50||mode->Judge_condition.If_get_allow_17)
      {
        mode->point=WE_DEPOT_POINT;
      }
      else if(!mode->Judge_condition.IF_5s_NotHurted&&mode->Judge_condition.If_hp_less_200)
      {
        mode->point=CENTRL_HIGH_POINT;
      }
      else if(mode->Judge_condition.IF_enemy_outpost_destroyed)
      {
        mode->point=WE_PATROL_POINT;
      }
      break;
    }

    case WE_DEPOT_POINT:
    {
      if(mode->Judge_condition.IF_Arrived&&mode->Judge_condition.IF_HP_recover)
      {
        if(mode->Judge_condition.IF_allowance_less_50)
        {  
          mode->point=WE_PATROL_POINT;
        }
        else if(!mode->Judge_condition.IF_outpost_destroyed)
        {
          mode->point=ENEMY_OUTPOST_POINT;
        }
        
      }
      break;
    }
    case WE_PATROL_POINT:
    { 
      if(mode->Judge_condition.IF_HP_Less_50||mode->Judge_condition.If_get_allow_17)
      {  
        mode->point=WE_DEPOT_POINT;
      }
      break;
    }
    case CENTRL_HIGH_POINT:
    {
    
      if(mode->Judge_condition.IF_HP_Less_50||mode->Judge_condition.If_get_allow_17)
      {  
        mode->point=WE_DEPOT_POINT;
      }
      else if(mode->Judge_condition.IF_5s_NotHurted)
      {
        mode->point=ENEMY_OUTPOST_POINT;
      }
      else if(mode->Judge_condition.IF_outpost_destroyed)
      { 
        mode->point=WE_PATROL_POINT;
      }
      
      break;
    }
  
    
    default:
    {
      mode->point=INIT_PACK_POINT;
      break;
    }
    
    
  }
}
//飞坡落点加打前哨 ENEMY_OUTPOST_POINT CENTRL_HIGH_POINT WE_FLYING_POINT
/* ====== 飞坡模式决策 ====== */
void sentry_flying_decision(decision_t *mode)
{
  
  switch(mode->point)
  {
    case INIT_PACK_POINT:
    {
      mode->point=ENEMY_OUTPOST_POINT;
      break;
    }
    case ENEMY_OUTPOST_POINT:
    {
      if(mode->Judge_condition.IF_HP_Less_50||mode->Judge_condition.If_get_allow_17)
      {
        mode->point=WE_DEPOT_POINT;
      }
      else if(!mode->Judge_condition.IF_5s_NotHurted&&mode->Judge_condition.If_hp_less_200)
      {
        mode->point=CENTRL_HIGH_POINT;
      }
      else if(mode->Judge_condition.IF_enemy_outpost_destroyed)
      {
        mode->point=WE_FLYING_POINT;
      }
      break;
    }
    case WE_DEPOT_POINT:
    {
      if(mode->Judge_condition.IF_Arrived&&mode->Judge_condition.IF_HP_recover)
      {
        if(mode->Judge_condition.IF_allowance_less_50)
        {  
          mode->point=WE_FLYING_POINT;
        }
        else if(!mode->Judge_condition.IF_outpost_destroyed)
        {
          mode->point=ENEMY_OUTPOST_POINT;
        }
        
      }
      break;
    }
    case WE_FLYING_POINT:
    { 
      if(mode->Judge_condition.IF_HP_Less_50||mode->Judge_condition.If_get_allow_17)
      {  
        mode->point=WE_DEPOT_POINT;
      }
      break;
    }
    case CENTRL_HIGH_POINT:
    {
    
      if(mode->Judge_condition.IF_HP_Less_50||mode->Judge_condition.If_get_allow_17)
      {  
        mode->point=WE_DEPOT_POINT;
      }
      else if(mode->Judge_condition.IF_5s_NotHurted)
      {
        mode->point=ENEMY_OUTPOST_POINT;
      }
      else if(mode->Judge_condition.IF_outpost_destroyed)
      { 
        mode->point=WE_FLYING_POINT;
      }
      
      break;
    }
  
    
    default:
    {
      mode->point=INIT_PACK_POINT;
      break;
    }
    
    
  }
}

//分区赛上场决策
//场地测试版本
/* ====== 保护模式决策 ====== */
void sentry_protect_decision(decision_t *mode)
{

    switch(mode->point)
    {
      case INIT_PACK_POINT:
      {
        mode->point=WE_FLYING_POINT;
        break;
      }
      case WE_DEPOT_POINT:
      {
        if((mode->Judge_condition.IF_Arrived==1&&mode->Judge_condition.IF_HP_recover==1))
        {
          if(mode->Judge_condition.IF_outpost_destroyed==1)
         {
           mode->point=WE_PATROL_POINT; 
         }
         else if(mode->Judge_condition.IF_HP_recover==1)
               mode->point=WE_FLYING_POINT;
        }
       break;
      }
      case WE_FLYING_POINT:
      {
       
       
       if(mode->Judge_condition.IF_HP_Less_100==1||mode->Judge_condition.If_get_allow_17==1)
        {
          mode->point=WE_DEPOT_POINT;
        }
       else if((mode->Judge_condition.IF_outpost_destroyed==1||mode->Judge_condition.IF_enemy_outpost_destroyed)&&mode->Judge_condition.IF_3s_NotFound)
       {
         mode->point=WE_PATROL_POINT;
       }
       break;
      } 
      case WE_PATROL_POINT:
      {   
        if(mode->Judge_condition.IF_HP_Less_100==1||mode->Judge_condition.If_get_allow_17==1)
        {
          mode->point=WE_DEPOT_POINT;
        }  
       break;
      }
            default:
      {
      
        mode->point=INIT_PACK_POINT;
        break;
      }
    }
}


//只有两个点  台阶前方
void sentry_two_point_decision(decision_t *mode)
{
  switch(mode->point)
  {
    case INIT_PACK_POINT:
    {
      mode->point=WE_PATROL_POINT;
      break;
    }
    case WE_PATROL_POINT:
    {
      if(mode->Judge_condition.IF_HP_Less_100||mode->Judge_condition.If_get_allow_17)
      {
        mode->point=WE_DEPOT_POINT;
      }
      break;
    }
    case WE_DEPOT_POINT:
    {
      if((mode->Judge_condition.IF_Arrived==1&&mode->Judge_condition.IF_HP_recover==1))
      {
        mode->point=WE_PATROL_POINT;
      }
      break;

    }
    default:
    {
      mode->point=INIT_PACK_POINT;
      break;
    }   
  }
}

void sentry_auto_decision(decision_t *mode)
{
  
  
}





void air_control_ctl(decision_t *mode)
{
  mode->point=MANUAL_POINT;
}

void map_control_fill(decision_t *mode)
{


  //将坐标系转换为导航坐标系
//  if(decision.robot_data.robot_color==red)
//  {


   #ifdef RED_START_NAVIGATION
// if(decision.robot_data.robot_color==red)
// {
     Red_Navi_position[MANUAL_POINT][0]=map_command.target_position_x;
     Red_Navi_position[MANUAL_POINT][1]=map_command.target_position_y;
//   }
//     else 
//  {
//    Red_Navi_position[MANUAL_POINT][0]=28-map_command.target_position_x;
//    Red_Navi_position[MANUAL_POINT][1]=15-map_command.target_position_y;
//  }
   
   #else 
   Red_Navi_position[MANUAL_POINT][0]=28-map_command.target_position_x;
   Red_Navi_position[MANUAL_POINT][1]=15-map_command.target_position_y;
   #endif

    
    
//  }
//  else 
//  {
//    Red_Navi_position[MANUAL_POINT][0]=28-map_command.target_position_x;
//    Red_Navi_position[MANUAL_POINT][1]=15-map_command.target_position_y;
//  }
}
float diff_yaw;
     float transform_x,transform_y;float dist_x,dist_y;
     float transform_angle;
/* ====== 决策点位填充（计算导航坐标） ====== */
void decision_point_fill()
{
    
  
  if(receive_gimbal_data.vision_state!=vision_frount)
  {
    #ifdef RED_START_NAVIGATION
  
  if(decision.point!=MANUAL_POINT)
  {
//      if(decision.robot_data.robot_color==red)
//    {
      navigation_tx.navi_set_x_pos=Red_Navi_position[decision.point][0]-Red_Navi_position[INIT_PACK_POINT][0];
      navigation_tx.navi_set_y_pos=Red_Navi_position[decision.point][1]-Red_Navi_position[INIT_PACK_POINT][1];
//    }
//    else 
//    {
////      navigation_tx.navi_set_x_pos=28-(Red_Navi_position[decision.point][0]-(Red_Navi_position[INIT_PACK_POINT][0]));
////      navigation_tx.navi_set_y_pos=15-(Red_Navi_position[decision.point][1]-(Red_Navi_position[INIT_PACK_POINT][1]));
//      navigation_tx.navi_set_x_pos=(28-Red_Navi_position[decision.point][0])-Red_Navi_position[INIT_PACK_POINT][0];
//      navigation_tx.navi_set_y_pos=15-Red_Navi_position[decision.point][1]-Red_Navi_position[INIT_PACK_POINT][1];
//    }
  }
  else 
  {
    navigation_tx.navi_set_x_pos=Red_Navi_position[decision.point][0]-Red_Navi_position[INIT_PACK_POINT][0];
    navigation_tx.navi_set_y_pos=Red_Navi_position[decision.point][1]-Red_Navi_position[INIT_PACK_POINT][1];
  }

  
  #else 
    if(decision.point!=MANUAL_POINT)
  {
  if(decision.robot_data.robot_color==red)
  {
    navigation_tx.navi_set_x_pos=(28-Red_Navi_position[decision.point][0])-(Red_Navi_position[0][0]);
    navigation_tx.navi_set_y_pos=(15-Red_Navi_position[decision.point][1])-(Red_Navi_position[0][1]);
    

  }
  else 
  { navigation_tx.navi_set_x_pos=Red_Navi_position[decision.point][0]-Red_Navi_position[INIT_PACK_POINT][0];
    navigation_tx.navi_set_y_pos=Red_Navi_position[decision.point][1]-Red_Navi_position[INIT_PACK_POINT][1];

  }
  }
  else 
  {
    navigation_tx.navi_set_x_pos=Red_Navi_position[decision.point][0]-Red_Navi_position[INIT_PACK_POINT][0];
    navigation_tx.navi_set_y_pos=Red_Navi_position[decision.point][1]-Red_Navi_position[INIT_PACK_POINT][1];
  }
  
  
 
  
  #endif
  }
  
  
  
else if(receive_gimbal_data.vision_state==vision_frount&&decision.point!=WE_DEPOT_POINT)
{
     transform_angle=receive_gimbal_data.current_transform_angle*2*3.14/360;
     dist_x=-receive_gimbal_data.armor_dist*cos(transform_angle);
     dist_y=-receive_gimbal_data.armor_dist*sin(transform_angle);
     
     
     
     diff_yaw=INS.Yaw-0;
     if(diff_yaw>180)
     {
       diff_yaw-=360;
     }
     else if(diff_yaw<-180)
     {
       diff_yaw+=360;
     }
     float diff_angle=diff_yaw*2*3.14/360;
     transform_x=dist_x*cos(diff_angle)+dist_y*sin(diff_angle);
     transform_y=-dist_x*sin(diff_angle)+dist_y*cos(diff_angle);
     
    navigation_tx.navi_set_x_pos=(navigation_rx.current_x)+transform_x;
    navigation_tx.navi_set_y_pos=(navigation_rx.current_y)+(-transform_y);
}
//  {
     
     

     
//}
}
uint8_t if_close_to_des;//是否靠近目标点
float navigation_x,navigation_y;
  float current_x,current_y;
void If_Point_arrived()
{
  
  
  
  if(decision.point!=MANUAL_POINT)
  {
    navigation_x=Red_Navi_position[decision.point][0]-Red_Navi_position[INIT_PACK_POINT][0];
    navigation_y=Red_Navi_position[decision.point][1]-Red_Navi_position[INIT_PACK_POINT][1];
  
    if(fabs(navigation_x-navigation_rx.current_x)<0.8
       &&fabs(navigation_y-navigation_rx.current_y)<0.8)
       {
         if_close_to_des=1;
       }
       else 
       {
         if_close_to_des=0;
       } 
  }
  else 
  {   
    //云台手坐标为全局坐标系坐标
    navigation_x=Red_Navi_position[decision.point][0];
    navigation_y=Red_Navi_position[decision.point][1];
  
    if(decision.robot_data.robot_color==red)
    {
    current_x=navigation_rx.current_x+Red_Navi_position[INIT_PACK_POINT][0];
    current_y=navigation_rx.current_y+Red_Navi_position[INIT_PACK_POINT][1];
    }
    else 
    {
    current_x=navigation_rx.current_x+(28-Red_Navi_position[INIT_PACK_POINT][0]);
    current_y=navigation_rx.current_y+(15-Red_Navi_position[INIT_PACK_POINT][1]);
    }
    if(fabs(navigation_x-current_x)<0.8
       &&fabs(navigation_y-current_x)<0.8)
       {
         if_close_to_des=1;
       }
    else if_close_to_des=0;
       
       
  }
   
//  if_close_to_des==1
  if(navigation_rx.If_get_path==0
      ||navigation_rx.if_lost_navi)

    {
      navigation_rx.if_arrived=1;
    }else
     navigation_rx.if_arrived=0;
}

/* 判断是否需要上敌方堡垒 */
void judge_if_need_to_enemy_fortress(Judge_condition_t *mode)
{
    if(decision.robot_data.robot_color==red)
  {
//  if(ground_robot_position.standard_3_x)
   if((ground_robot_position.standard_3_x>17&&ground_robot_position.standard_3_x<28)&&
    (ground_robot_position.standard_4_x>17&&ground_robot_position.standard_4_x<28))
  {
    mode->If_need_to_enemy_fortress=1;
  }
  else mode->If_need_to_enemy_fortress=0;
  }
  else if(decision.robot_data.robot_color==blue)
  {
     if((ground_robot_position.standard_3_x>0&&ground_robot_position.standard_3_x<11)&&
        (ground_robot_position.standard_4_x>0&&ground_robot_position.standard_4_x<11))
    {
      mode->If_need_to_enemy_fortress=1;
    }
    else mode->If_need_to_enemy_fortress=0;
  
  
    
  }
}



/* 判断决策点是否切换 */
void IF_decision_point_change(Judge_condition_t *mode)
{

}

// 判断基地是否扣血

uint16_t Last_base_hurt_time=420;
/* 判断基地是否在吊射状态 */
void judge_if_chip_base(uint16_t base_hp,Judge_condition_t *mode)
{

  static uint16_t Last_base_hp;
  
  if((Last_base_hp-base_hp>=40)&&mode->IF_need_to_protect==0)
  { 
    mode->If_chip_base=1;
  }
  else if(base_hp==5000||mode->IF_need_to_protect==1)
  { 
    mode->If_chip_base=0;
  }
}
  uint16_t last_allowance_17=300;
  uint16_t allow_to_get_17mm;//记录可以兑换的小弹丸
  uint16_t already_allowance_17;//记录已经拿取的小弹丸
  uint8_t  remain_time;//记录还剩多久可以领取小弹丸
// static uint16_t last_allowance_17;
// static uint16_t allow_to_get_17mm;//记录可以兑换的小弹丸
// static uint16_t already_allowance_17;//记录已经拿取的小弹丸
//检测是否需要回补给区补弹
/* 判断是否需要补给区补给17mm弹丸 */
void judge_if_need_allow_17(Judge_condition_t *mode)
{


 
 if(projectile_allowance.projectile_allowance_17mm>last_allowance_17)
 {
   already_allowance_17 += (projectile_allowance.projectile_allowance_17mm-last_allowance_17);
 }
 
 allow_to_get_17mm = ((420-game_state.stage_remain_time)/60)*100;
 

//剩余时间/60 < 20 回家补弹
remain_time = game_state.stage_remain_time % 60;
// if((projectile_allowance.projectile_allowance_17mm<50&&(allow_to_get_17mm-already_allowance_17>=200))
//    ||(projectile_allowance.projectile_allowance_17mm<20&&(allow_to_get_17mm-already_allowance_17>=100&&decision.point==WE_PATROL_POINT))
//    ||(projectile_allowance.projectile_allowance_17mm<5&&remain_time<5))
//    
//   {
//   mode->If_get_allow_17=1;
//    
//   }
//   else mode->If_get_allow_17=0;
 if(((projectile_allowance.projectile_allowance_17mm<50&&receive_gimbal_data.Vision_look_state==vision_lost)&&(allow_to_get_17mm-already_allowance_17>=200))
    ||(projectile_allowance.projectile_allowance_17mm<5&&remain_time<5&&remain_time>0))
    
   {
   mode->If_get_allow_17=1;
    
   }
   else mode->If_get_allow_17=0;
 
 
 
 last_allowance_17 = projectile_allowance.projectile_allowance_17mm;
 
 
}

/*检测是否10s 5s 3s未受击打*/
/* 判断哨兵是否未受击超过阈值 */
bool judge_if_nothurt(Judge_condition_t *mode)
{

  static uint16_t Last_HP;
  
  
  
  if(Last_HP-robot_status.current_HP>=10)
  {
   hurt_time=game_state.stage_remain_time;
  }

  
  
    if(hurt_time-game_state.stage_remain_time>10)
  { 
   mode->IF_10s_NotHurted=1;
  }
  else mode->IF_10s_NotHurted=0;
  
  if(hurt_time-game_state.stage_remain_time>5)
  { 
   mode->IF_5s_NotHurted=1;
  }
  else mode->IF_5s_NotHurted=0;
  
    if(hurt_time-game_state.stage_remain_time>3)
  { 
   mode->IF_3s_NotHurted=1;
  }
  else mode->IF_3s_NotHurted=0;
  
  Last_HP=robot_status.current_HP;
}

//void judge_if_hp_less_100(Judge_condition_t *mode)
//{
//  
//}





/*检测是否3,5,10s未发现敌人*/


/* 判断哨兵是否未发现敌人超过阈值 */
void judge_if_not_found(Judge_condition_t *mode)
{ 
  static int hurt_time=0;
  static int nfound_time=0;
  
  
  static uint8_t last_point;

  if(receive_gimbal_data.vision_state != vision_lost&&decision.Judge_condition.IF_Arrived==1)
  {
    nfound_time = 0;
  }
  else nfound_time ++;
  
  
  if(nfound_time > TIM_3S)
  {
    mode->IF_3s_NotFound = 1;
  }
  else mode->IF_3s_NotFound = 0;
  
    if(nfound_time > TIM_5S)
  {
    mode->IF_5s_NotFound = 1;
  }
  else mode->IF_5s_NotFound = 0;
  
      if(nfound_time > TIM_10S)
  {
    mode->IF_10s_NotFound = 1;
  }
  else mode->IF_10s_NotFound = 0;
  
  
  
  if(decision.point!=last_point)
  { 
    mode->IF_3s_NotFound = 0;
    mode->IF_5s_NotFound = 0;
    mode->IF_10s_NotFound = 0;
  } 
  
  last_point =decision.point;

}


/*检测是否到达导航点*/
/* 判断导航是否结束 */
void judge_if_location_over(Judge_condition_t *mode)
{ 

  if(navigation_rx.if_arrived==1)//已到达
  {
    mode->IF_Arrived=1;
  } 
  else 
  {
    mode->IF_Arrived=0;
  }
}

/* 判断允许发弹量是否小于50 */
void judge_if_allowance_less_50(Judge_condition_t *mode)
{
  if(projectile_allowance.projectile_allowance_17mm<30)
  {
    mode->IF_allowance_less_50=1;
  }
  else mode->IF_allowance_less_50=0;
}

/* 判断允许发弹量是否小于100 */
void judge_if_allowance_less_100(Judge_condition_t *mode)
{
  if(projectile_allowance.projectile_allowance_17mm<70)
  {
    mode->IF_allowance_less_100=1;
  }
  else mode->IF_allowance_less_100=0;
}

/* 判断哨兵血量是否小于200 */
void judge_if_HP_less_200(Judge_condition_t *mode)
{
   if(robot_status.current_HP<=150)
   {
     mode->If_hp_less_200=1;
   }
   else mode->If_hp_less_200=0;
}

//判断血量是否低于100
/* 判断哨兵血量是否小于100 */
void judge_if_HP_less_100(Judge_condition_t *mode)
{
   if(robot_status.current_HP<=110)
   {
     mode->IF_HP_Less_100=1;
   }
   else mode->IF_HP_Less_100=0;
}


//判断血量是否低于150
/* 判断哨兵血量是否小于50 */
void judge_if_HP_less_50(Judge_condition_t *mode)
{
   if(robot_status.current_HP<=70)
   {
     mode->IF_HP_Less_50=1;
   }
   else mode->IF_HP_Less_50=0;
}

/*判断己方基地护甲是否展开*/
uint16_t last_dart_time=0;
uint16_t lock_dart_count=0;
uint8_t if_random_dart=0;
/* 判断己方基地护甲是否展开 */
void judge_base_armor_spred(Judge_condition_t *mode, int16_t base_HP)
{

  if(last_dart_time!=event_data.thelast_dart_hit_time&&event_data.thelast_dart_hit_goal==3)
  {
    lock_dart_count++;
  }
  if(event_data.thelast_dart_hit_goal==4)
  {
    if_random_dart=1;
  }
  
  if(event_data.thelast_dart_hit_goal==3)
  {
    last_dart_time=event_data.thelast_dart_hit_time;
  }
  
  
  
  if(base_HP<=2200||if_random_dart==1||lock_dart_count==4)//后续加上飞镖打中的条件
  {
    mode->IF_base_armor_spred=1;
  }
  else mode->IF_base_armor_spred=0;
}
//小能量机关开启后持续45s
uint16_t enemy_small_energy_time=500;
/* 判断敌方是否开启小能量机关 */
bool judge_if_enemy_small_energy(Judge_condition_t *mode,int16_t outpost_HP )
{
  static uint16_t Last_outpost_HP;
   
  
  
  if(outpost_HP-Last_outpost_HP==5&&mode->If_enemy_small_energy==0&&game_state.stage_remain_time>120)
  {
    enemy_small_energy_time=game_state.stage_remain_time;
    mode->If_enemy_small_energy=0;
  }
  if(mode->If_enemy_small_energy==1&&(enemy_small_energy_time-game_state.stage_remain_time>=40))
  {
    mode->If_enemy_small_energy=0;
  }
  
  
  
}
//判断己方前哨站是否被击毁
/* 判断己方前哨站是否被击毁 */
void judge_if_outpost_destroyed(Judge_condition_t *mode,int16_t outpost_HP )
{
  if(outpost_HP<=250)
  {
    mode->IF_outpost_destroyed=1;
  }
  else mode->IF_outpost_destroyed=0;
  
}

//判断发射机构是否锁住
/* 判断发射机构是否锁住 */
void judge_if_fire_lock(Judge_condition_t *mode)
{
   
 static uint16_t lock_time; 
   if(robot_status.current_HP==0&&robot_status.power_management_shooter_output==0)
   {
     mode->IF_fire_lock=1;
   }
  if(robot_status.power_management_shooter_output==1)
  {
    mode->IF_fire_lock=0;
  }
}




/* 判断哨兵是否回血完成 */
void judge_if_HP_recover(Judge_condition_t *mode)
{

  if(robot_status.current_HP==400)
  {
    mode->IF_HP_recover=1;
  }
  else mode->IF_HP_recover=0;
}



/*根据导航与遇敌情况以及受击情况来自主选择底盘模式  联盟赛一直陀螺，根据速度输入来分配陀螺和移动的速度分配*/
/* ====== 自动底盘速度控制 ====== */
void AGV_auto_chassis(decision_t *mode)
{

    if(rc_ctrl.rc.s_l!=2)
    {
       if(mode->Judge_condition.IF_Arrived)
     {
          if(mode->Judge_condition.If_chassis_weak==0)
      { 
        if(mode->Judge_condition.IF_3s_NotHurted&&mode->point==ENEMY_OUTPOST_POINT)
        {
          sentry_system.chassis_mode=normol_move;
          sentry_system.chassis_set.Vz_state=no_spine;
        }
        else if(!mode->Judge_condition.IF_3s_NotHurted&&mode->point==ENEMY_OUTPOST_POINT)
        {
          sentry_system.chassis_mode=normol_move;
          sentry_system.chassis_set.Vz_state=low_spine;
        }
//        else if(!mode->Judge_condition.IF_3s_NotHurted&&mode->point)
//        {
//          sentry_system.chassis_mode=normol_move;
//          sentry_system.chassis_set.Vz_state=low_spine;
//        }        
        else  if(!mode->Judge_condition.IF_5s_NotHurted)
        {
        sentry_system.chassis_mode=normol_move;
        sentry_system.chassis_set.Vz_state=high_spine;          
        }
        else 
        { 
        sentry_system.chassis_mode=normol_move;
        sentry_system.chassis_set.Vz_state=low_spine;
        }
      }
      else 
      {
//        if(!mode->Judge_condition.IF_3s_NotHurted)
//        {
          sentry_system.chassis_mode=normol_move;
          sentry_system.chassis_set.Vz_state=low_spine;
//        }
//        else 
//        {
//          sentry_system.chassis_mode=normol_move;
//          sentry_system.chassis_set.Vz_state=no_spine;
//        }
        
      }
     }
     else 
     {
       if(mode->Judge_condition.If_on_toss
          ||(mode->Judge_condition.If_moving_v&&mode->Judge_condition.IF_3s_NotHurted))
       {
        sentry_system.chassis_mode=navigation_move; 
        sentry_system.chassis_set.Vz_state=no_spine;
       }
       else if(mode->Judge_condition.If_chassis_weak)
       { 
        sentry_system.chassis_mode=navigation_move; 
        sentry_system.chassis_set.Vz_state=no_spine;
       }
       else if(mode->Judge_condition.If_moving_v&&!mode->Judge_condition.IF_3s_NotHurted)
       {
         sentry_system.chassis_mode=navigation_move; 
        sentry_system.chassis_set.Vz_state=mid_spine;
       }
       else if(!mode->Judge_condition.IF_5s_NotHurted)
       {
         sentry_system.chassis_mode=navigation_move; 
        sentry_system.chassis_set.Vz_state=mid_spine;         
       }
       else 
       {
        sentry_system.chassis_mode=navigation_move; 
        sentry_system.chassis_set.Vz_state=no_spine;
       }
     
        
     }
      
    }
    else 
    {
      sentry_system.chassis_mode=no_move;
      sentry_system.chassis_set.Vz_state=no_spine;
    }
    
    //表演赛不开导航
//sentry_system.chassis_mode=normol_move;
//  if(rc_ctrl.rc.s_l!=2)
//  {sentry_system.chassis_mode=follow_move;
//  }
//  else sentry_system.chassis_mode=no_move;
  chassis_speed_set(&sentry_system);
}









//哨兵初始血量为400

//运动决策
/* 连续状态判断处理（周期性调用） */
void Judge_Continuous_Handle(Judge_condition_t *mode)
{
  if(game_state.game_progress==4)
  {
  
  //无关颜色的
       If_Point_arrived();
       judge_if_location_over(mode);
       judge_if_nothurt(mode);;
       judge_if_not_found(mode);
    
       judge_if_HP_less_100(mode);
       judge_if_fire_lock(mode);
       judge_if_HP_recover(mode);
       judge_if_stop_navi();
       judge_if_chassis_weak();
       judge_if_need_allow_17(mode);
       judge_if_fortress_free(mode);
       judge_if_enemy_outpost_lock(mode);
       judge_if_need_to_protect(mode);
       judge_if_moving_v(mode);
       judge_if_on_toss(mode);
       judge_if_allowance_less_50(mode);
       judge_if_allowance_less_100(mode);
       judge_if_HP_less_50(mode);
       judge_if_fortress_allow_less_50(mode);
       judge_if_close_to_enemy_out(mode);
  
  
    //有关颜色的
      if(decision.robot_data.robot_color==red)
    {
       judge_base_armor_spred(mode,game_robot_HP.red_base_HP);
       judge_if_outpost_destroyed(mode,game_robot_HP.red_outpost_HP );
       judge_if_chip_base(game_robot_HP.red_base_HP,mode);
       judge_if_enemy_outpost_destroyed(mode,game_robot_HP.blue_outpost_HP);
       judge_if_enemy_small_energy(mode,game_robot_HP.blue_outpost_HP);
    }
    else if(decision.robot_data.robot_color==blue)
    {
       judge_base_armor_spred(mode,game_robot_HP.blue_base_HP);
       judge_if_outpost_destroyed(mode,game_robot_HP.blue_outpost_HP );
       judge_if_chip_base(game_robot_HP.blue_base_HP,mode);
       judge_if_enemy_outpost_destroyed(mode,game_robot_HP.red_outpost_HP);
       judge_if_enemy_small_energy(mode,game_robot_HP.red_outpost_HP);
    }
    
       
       
   
  }
}

/* 判断堡垒增益点弹药是否少于50 */
void judge_if_fortress_allow_less_50(Judge_condition_t *mode)
{
  if(projectile_allowance.projectile_allowance_fortress<50)
  {
    mode->IF_fortress_allow_less_50=1;
  }
  else 
  {
    mode->IF_fortress_allow_less_50=0;
  }
}

/* 判断是否在中央荒地上 */
void judge_if_on_toss(Judge_condition_t *mode)
{
  float current_x,current_y;
 current_x=navigation_rx.current_x+Red_Navi_position[0][0];
 current_y=navigation_rx.current_y+Red_Navi_position[0][1];
 
 if((current_x>we_toss_start_x&&current_x<we_toss_end_x&&current_y>we_toss_start_y&&current_y<we_toss_end_y)
   ||(current_x>enemy_toss_start_x&&current_x<enemy_toss_end_x&&current_y>enemy_toss_start_y&&current_y<enemy_toss_end_y))
    {
      mode->If_on_toss=1;
    }
    else mode->If_on_toss=0;
 
}

/* 判断是否正在过U型弯 */
void judge_if_moving_v(Judge_condition_t *mode)
{

 float current_x,current_y;
 current_x=navigation_rx.current_x+Red_Navi_position[0][0];
          
 current_y=navigation_rx.current_y+Red_Navi_position[0][1];
            
 if((current_x>we_u_start_x&&current_x<we_u_end_x&&current_y>we_u_start_y&&current_y<we_u_end_y)
   ||(current_x>enemy_u_start_x&&current_x<enemy_u_end_x&&current_y>enemy_u_start_y&&current_y<enemy_u_end_y))
    {
      mode->If_moving_v=1;
    }
    else mode->If_moving_v=0;
 
}


/* 判断敌方前哨站是否被摧毁 */
void judge_if_enemy_outpost_destroyed(Judge_condition_t *mode,int16_t enemy_outpost_HP)
{
  if(enemy_outpost_HP==0)
  {
    mode->IF_enemy_outpost_destroyed=1;
  }
  else mode->IF_enemy_outpost_destroyed=0;
}

/* 判断堡垒增益区是否空闲 */
void judge_if_fortress_free(Judge_condition_t *mode)
{ 
  if(decision.robot_data.robot_color==red)
  {
    if(((fabs(ground_robot_position.standard_3_x-RED_FORTRESS_X)<0.5&&fabs(ground_robot_position.standard_3_y-RED_FORTRESS_Y)<0.5))
    ||((fabs(ground_robot_position.standard_3_x-RED_FORTRESS_X)<0.5&&fabs(ground_robot_position.standard_3_y-RED_FORTRESS_Y)<0.5)))
    {
      mode->If_fortress_free=0;
    }
    else mode->If_fortress_free=1;
  }
  if(decision.robot_data.robot_color==blue)
  {
    if(((fabs(ground_robot_position.standard_3_x-BLUE_FORTRESS_X)<0.5&&fabs(ground_robot_position.standard_3_y-BLUE_FORTRESS_Y)<0.5))
    ||((fabs(ground_robot_position.standard_3_x-BLUE_FORTRESS_X)<0.5&&fabs(ground_robot_position.standard_3_y-BLUE_FORTRESS_Y)<0.5)))
    {
      mode->If_fortress_free=0;
    }
    else mode->If_fortress_free=1;
  }
  
}


/* 判断敌方前哨站是否停转 */
void judge_if_enemy_outpost_lock(Judge_condition_t *mode)
{
  if(mode->IF_base_armor_spred==1||game_state.stage_remain_time<240)
  {
    mode->If_enemy_outpost_lock=1;
  }
  else mode->If_enemy_outpost_lock=0;
}

//todo


//||
//        (map_robot_data.sentry_position_x>0&&map_robot_data.sentry_position_x<11)||
//        (map_robot_data.infantry_3_position_x>0&&map_robot_data.infantry_3_position_x<11)||
//        (map_robot_data.infantry_4_position_x>0&&map_robot_data.infantry_4_position_x<11)

//)||
//        (map_robot_data.sentry_position_x>17&&map_robot_data.sentry_position_x<28)||
//        (map_robot_data.infantry_3_position_x>17&&map_robot_data.infantry_3_position_x<28)||
//        (map_robot_data.infantry_4_position_x>17&&map_robot_data.infantry_4_position_x<28)


/* 判断是否需要去保护基地 */
void judge_if_need_to_protect(Judge_condition_t *mode)
{  
  u8_to_u16 hero_x,hero_y,infantry_3_x,infantry_3_y,infantry_4_x,infantry_4_y;
  
  
  //雷达站回传数据
  hero_x.d[0]=robot_interaction_data.data[0];
  hero_x.d[1]=robot_interaction_data.data[1];
  hero_y.d[0]=robot_interaction_data.data[2];
  hero_y.d[1]=robot_interaction_data.data[3];
  infantry_3_x.d[0]=robot_interaction_data.data[4];
  infantry_3_x.d[1]=robot_interaction_data.data[5];
  infantry_3_y.d[0]=robot_interaction_data.data[6];
  infantry_3_y.d[1]=robot_interaction_data.data[7];
  infantry_4_x.d[0]=robot_interaction_data.data[8];
  infantry_4_x.d[1]=robot_interaction_data.data[9];
  infantry_4_y.d[0]=robot_interaction_data.data[10];
  infantry_4_y.d[1]=robot_interaction_data.data[11];
  
//  hero_x=robot_interaction_data.data[0];
//  hero_x=robot_interaction_data.data[1];
//  infantry_3_x=robot_interaction_data.data[2];
//  infantry_3_y=robot_interaction_data.data[3];
//  infantry_4_x=robot_interaction_data.data[4];
//  infantry_4_y=robot_interaction_data.data[5];
  
  
  if(decision.robot_data.robot_color==red)
  {
    if((hero_x.data>0&&hero_x.data<1100))
        {
          mode->IF_need_to_protect=1;
        }
        else mode->IF_need_to_protect=0;
  }
  else if(decision.robot_data.robot_color==blue)
  {
    if((hero_x.data>1700&&hero_x.data<2800))
        {
          mode->IF_need_to_protect=1;
        }
        else mode->IF_need_to_protect=0;
  }
  mode->IF_need_to_protect=0;
}


/* 判断底盘是否需要进入虚弱模式 */
void judge_if_chassis_weak()
{
  if(game_state.stage_remain_time<90)
  {
    decision.Judge_condition.If_chassis_weak=1;
  }
  else decision.Judge_condition.If_chassis_weak=0;
  
}


/* 判断是否需要停止导航击打敌人 */
void judge_if_stop_navi()
{
  if(receive_gimbal_data.vision_state!=vision_lost)
  {
    decision.Judge_condition.If_stop_navi=1;
  }
  else decision.Judge_condition.If_stop_navi=0;
}


/* ====== 哨兵射击决策 ====== */
void sentry_shoot_decision(decision_t *mode)
{
  judge_if_shoot(mode);//判断是否击打某些车体
  judge_shoot_top_senior_priority(mode);//自瞄最高优先级
}

void judge_shoot_top_senior_priority(decision_t *mode)
{
  
  uint16_t vision_robot_HP;
  
  if(decision.robot_data.robot_color==red)
  {
    if(receive_gimbal_data.vision_armor_id==ARMOR_HERO)
    {
      vision_robot_HP=game_robot_HP.blue_1_robot_HP;
    }
    else if(receive_gimbal_data.vision_armor_id==ARMOR_ENGINEER)
    {
      vision_robot_HP=game_robot_HP.blue_2_robot_HP;
    }
    else if(receive_gimbal_data.vision_armor_id==ARMOR_INFANTRY3)
    {
      vision_robot_HP=game_robot_HP.blue_3_robot_HP;
    }
    else if(receive_gimbal_data.vision_armor_id==ARMOR_INFANTRY4)
    {
      vision_robot_HP=game_robot_HP.blue_4_robot_HP;
    }
    else if(receive_gimbal_data.vision_armor_id==ARMOR_SENTRY)
    {
      vision_robot_HP=game_robot_HP.blue_7_robot_HP;
    }
    else if(receive_gimbal_data.vision_armor_id==ARMOR_OUTPOST)
    {
      vision_robot_HP=game_robot_HP.blue_outpost_HP;
    }
  }
  else if(decision.robot_data.robot_color==red)
  {
        if(receive_gimbal_data.vision_armor_id==ARMOR_HERO)
    {
      vision_robot_HP=game_robot_HP.red_1_robot_HP;
    }
    else if(receive_gimbal_data.vision_armor_id==ARMOR_ENGINEER)
    {
      vision_robot_HP=game_robot_HP.red_2_robot_HP;
    }
    else if(receive_gimbal_data.vision_armor_id==ARMOR_INFANTRY3)
    {
      vision_robot_HP=game_robot_HP.red_3_robot_HP;
    }
    else if(receive_gimbal_data.vision_armor_id==ARMOR_INFANTRY4)
    {
      vision_robot_HP=game_robot_HP.red_4_robot_HP;
    }
    else if(receive_gimbal_data.vision_armor_id==ARMOR_SENTRY)
    {
      vision_robot_HP=game_robot_HP.red_7_robot_HP;
    }
    else if(receive_gimbal_data.vision_armor_id==ARMOR_OUTPOST)
    {
      vision_robot_HP=game_robot_HP.red_outpost_HP;
    }
  }
  
  if((vision_robot_HP>5&&vision_robot_HP<=50)||receive_gimbal_data.vision_armor_id==ARMOR_HERO)
  { 
    mode->top_senior_priority=receive_gimbal_data.vision_armor_id;
  }
  else if(decision.point==CENTRL_HIGH_POINT)
  {
    mode->top_senior_priority=ARMOR_ENGINEER;
  }
  else if(decision.point==ENEMY_OUTPOST_PROTECT_POINT&&decision.point==ENEMY_FORTRESS_POINT)
  {
    mode->top_senior_priority=ARMOR_HERO;
  }
  else mode->top_senior_priority=ARMOR_HERO;
  
}
/* ====== 射击条件判断 ====== */
void judge_if_shoot(decision_t *mode)
{
   if(game_state.game_progress!=4)
   {
     mode->Vision_ByteBits.shoot_Hero=1;
     mode->Vision_ByteBits.shoot_Engineer=1;
     mode->Vision_ByteBits.shoot_infantr3=1;
     mode->Vision_ByteBits.shoot_infantr4=1;
     mode->Vision_ByteBits.shoot_Sentry=1;
     mode->Vision_ByteBits.shoot_outpost=1;
     mode->Vision_ByteBits.shoot_base=1;
     
   }
   else 
   {

    
     if(decision.robot_data.robot_color==red)
    {
      
      mode->Vision_ByteBits.shoot_Hero=judge_if_shoot_hero(game_robot_HP.blue_1_robot_HP);
      mode->Vision_ByteBits.shoot_Engineer=judge_if_shoot_Engineer(game_robot_HP.blue_2_robot_HP);
      mode->Vision_ByteBits.shoot_infantr3=judge_if_shoot_infantr3(game_robot_HP.blue_3_robot_HP);
      mode->Vision_ByteBits.shoot_infantr4=judge_if_shoot_infantr4(game_robot_HP.blue_4_robot_HP);
//      mode->Vision_ByteBits.shoot_infantr4=1;
      mode->Vision_ByteBits.shoot_Sentry= judge_if_shoot_sentry(game_robot_HP.blue_7_robot_HP,game_robot_HP.blue_outpost_HP);
      mode->Vision_ByteBits.shoot_outpost = judge_if_shoot_outpost(game_robot_HP.blue_outpost_HP);
      mode->Vision_ByteBits.shoot_base=judge_if_shoot_base(game_robot_HP.blue_outpost_HP);
    }

   else 
  {
      mode->Vision_ByteBits.shoot_Hero=judge_if_shoot_hero(game_robot_HP.red_1_robot_HP);
	  mode->Vision_ByteBits.shoot_Engineer=judge_if_shoot_Engineer(game_robot_HP.red_2_robot_HP);
	  mode->Vision_ByteBits.shoot_infantr3=judge_if_shoot_infantr3(game_robot_HP.red_3_robot_HP);
	  mode->Vision_ByteBits.shoot_infantr4=judge_if_shoot_infantr4(game_robot_HP.red_4_robot_HP);
	  mode->Vision_ByteBits.shoot_Sentry= judge_if_shoot_sentry(game_robot_HP.red_7_robot_HP,game_robot_HP.red_outpost_HP);
	  mode->Vision_ByteBits.shoot_outpost = judge_if_shoot_outpost(game_robot_HP.red_outpost_HP);
	  mode->Vision_ByteBits.shoot_base=judge_if_shoot_base(game_robot_HP.red_outpost_HP);

   
   }
//   if(mode->point==ENEMY_OUTPOST_POINT||mode->point==CENTRL_HIGH_POINT)
//   {
//      mode->Vision_ByteBits.shoot_Engineer=0;
//      mode->Vision_ByteBits.shoot_infantr3=0;
//      mode->Vision_ByteBits.shoot_infantr4=0;
//      mode->Vision_ByteBits.shoot_Sentry= 0;
////      mode->Vision_ByteBits.shoot_outpost = 1;
//      mode->Vision_ByteBits.shoot_base=0;
//   }
   
//   if((!mode->Judge_condition.IF_Arrived&&mode->point!=MANUAL_POINT)
//       ||mode->Judge_condition.If_moving_v||mode->Judge_condition.If_on_toss)
//   {
////      mode->Vision_ByteBits.shoot_Hero=0;
//      mode->Vision_ByteBits.shoot_Engineer=0;
//      mode->Vision_ByteBits.shoot_infantr3=0;
//      mode->Vision_ByteBits.shoot_infantr4=0;
//      mode->Vision_ByteBits.shoot_Sentry= 0;
//      mode->Vision_ByteBits.shoot_outpost = 0;
//      mode->Vision_ByteBits.shoot_base=0;
//   }
//	else 
//	{
//	  
//	  
//	}
   }
   
   
  
}

//  static uint16_t wait_tim3 = 0;
//  static uint16_t wait_tim10 = 0;
//  static uint16_t last_hp;

//   uint16_t wait_tim3 = 0;
//   uint16_t wait_tim10 = 0;
//   uint16_t last_hp;

int kjkjk;
static bool judge_if_shoot_hero(int16_t robot_hp)
{
  bool res;
  static uint16_t wait_tim3 = 0;
  static uint16_t wait_tim10 = 0;
  static uint16_t last_hp;
  
kjkjk++;
  
  if(robot_hp<=60 &&robot_hp>0 && last_hp==0)
  {
   wait_tim10 = game_state.stage_remain_time;
  }
  
  if(robot_hp>=200 && last_hp==0)
  {
   wait_tim3 = game_state.stage_remain_time;
  }
   if(((wait_tim10-game_state.stage_remain_time<9)&&(wait_tim10-game_state.stage_remain_time>=0))
      |((wait_tim3-game_state.stage_remain_time<2)&&(wait_tim3-game_state.stage_remain_time>=0))
      )
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
  
  if(robot_hp<=60 && robot_hp>0 && last_hp==0)
  {
   wait_tim10 = game_state.stage_remain_time;
  }
  
  if(robot_hp>=150 && last_hp == 0)
  {
    wait_tim3 = game_state.stage_remain_time;   
   }
   
   if(((wait_tim10-game_state.stage_remain_time<9)&&(wait_tim10-game_state.stage_remain_time>=0))
      ||((wait_tim3-game_state.stage_remain_time<3)&&(wait_tim3-game_state.stage_remain_time>=0))   
      )
//      ||(game_state.stage_remain_time>360)//考虑直接干掉工程
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
  
  if(robot_hp<=40 &&robot_hp>0 && last_hp==0)
  {
   wait_tim10 = game_state.stage_remain_time;
  }
  
  if(robot_hp>=150 && last_hp == 0)
  {
    wait_tim3 = game_state.stage_remain_time;   
  }
   
   if(((wait_tim10-game_state.stage_remain_time<9)&&(wait_tim10-game_state.stage_remain_time>=0))
      ||((wait_tim3-game_state.stage_remain_time<3)&&(wait_tim3-game_state.stage_remain_time>=0))
      )
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
  
  if(robot_hp<=40 &&robot_hp>0 && last_hp==0)
  {
   wait_tim10 = game_state.stage_remain_time;
  }
  
  if(robot_hp>=150 && last_hp == 0)
  {
    wait_tim3 = game_state.stage_remain_time;   
   }
   
   if(((wait_tim10-game_state.stage_remain_time<9)&&(wait_tim10-game_state.stage_remain_time>=0))
      ||((wait_tim3-game_state.stage_remain_time<3)&&(wait_tim3-game_state.stage_remain_time>=0))
     )
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
  static uint8_t if_die=0;  
  if(robot_hp<=80 &&robot_hp>0 && last_hp==0)
  {
    // 记录死亡
   if_die=1;
   wait_tim10 = game_state.stage_remain_time;
  }
 //检测到哨兵补完血
 if(robot_hp>80)
 {
   if_die=0;
 }
  
 if(robot_hp>=150 && last_hp == 0)
 {
  wait_tim3 = game_state.stage_remain_time;   
 }
//   ||(game_state.stage_remain_time>300)
   if(if_die
      ||((wait_tim3-game_state.stage_remain_time<2)&&(wait_tim3-game_state.stage_remain_time>=0))
      )
     
      {
       res=0;
      }
    else 
      res=1;
   
   last_hp = robot_hp;
   
   return res;
}

bool judge_if_shoot_outpost(uint16_t outpost_hp)
{ 
  bool res=0;
  
   if((decision.point==ENEMY_OUTPOST_POINT||decision.point==CENTRL_HIGH_POINT))
  {
    res=1;
  }
  else
  {
     res=0;
  }

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





