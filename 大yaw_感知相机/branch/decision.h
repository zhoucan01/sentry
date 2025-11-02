#ifndef __DECISION_H
#define __DECISION_H

#include "struct_typedef.h"
#include "stdbool.h"
#include "system.h"
#include "Nautilus_UI.h"
//DECISION_POSITION_NUM 





//we_fortress_gain_points
#define INIT_PACK_POINT                   0       // 初始化放置点 3.79, 7.99
#define WE_DEPOT_POINT                    1       // 己方补给区1.75  , 2.47
#define WE_FLYING_POINT                   2       // 己方飞坡落点 10.35 , 14.27
#define WE_FORTRESS_POINT                 3       // 己方堡垒6.71 , 7.41*
#define WE_PATROL_POINT                   4       // 己方防守点 8.9 , 7.5
#define WE_FORTRESS_BEHIND_FRONT_POINT    5       // 己方堡垒前方6.65 , 9.1
#define WE_FORTRESS_BEHIND_RIGHT_POINT    6       // 己方堡垒右 5.28 6.84 
#define ENEMY_OUTPOST_POINT               7       // 敌方前哨站 15.75,9.39  

#define ENEMY_OUTPOST_PROTECT_POINT       8     // 防止敌方英雄击打前哨站 21.18 5.67
#define ENEMY_BASE_PROTECT_POINT          9     // 防止敌方英雄吊射       23.80 7.59

#define WE_OUTPOST_POINT                  10       //己方前哨站11.43, 4.36
#define ENEMY_FORTRESS_POINT              11       //敌方堡垒21.26, 7.50
#define CENTRL_HIGH_POINT                 12       //高地背敌处12.48, 9.16
#define MANUAL_POINT                      13       // 云台手点位13.48, 8.98
#define DECISION_POSITION_NUM             14       // 总总共十四个点位


#define TIM_3S   500
#define TIM_5S   1500
#define TIM_10S  2500


//#define CENTRAL_GAIN_LEFT_RANGE          1
//#define CENTRAL_GAIN_RIGHT_RANGE         1
//#define CENTRAL_GAIN_UP_RANGE            1
//#define CENTRAL_GAIN_DOWN_RANGE          1


//堡垒
#define RED_FORTRESS_X                   10.5
#define RED_FORTRESS_Y                   7.5

#define BLUE_FORTRESS_X                  21- 10.5
#define BLUE_FORTRESS_Y                   7.5


//u型弯
#define we_u_start_x 3.91  
#define we_u_end_x   5.05
#define we_u_start_y -1
#define we_u_end_y    3.52

#define enemy_u_start_x 22.66  
#define enemy_u_end_x   24.5
#define enemy_u_start_y 11.60
#define enemy_u_end_y    16


//中央荒地
#define we_toss_start_x 11.29
#define we_toss_end_x   15.95
#define we_toss_start_y  2.62
#define we_toss_end_y    5

#define enemy_toss_start_x 11.9
#define enemy_toss_end_x   16.7
#define enemy_toss_start_y 10.12
#define enemy_toss_end_y   12.38


typedef enum
{
  
  NO_CONTACT=0,
  red,
  blue,
  
}robot_color_t ;


typedef __packed struct 
{  
	uint8_t shoot_all      : 1;  
	uint8_t shoot_Hero     : 1;  
	uint8_t shoot_Engineer : 1;  
	uint8_t shoot_infantr3 : 1;  
	uint8_t shoot_infantr4 : 1;  
	uint8_t shoot_Sentry   : 1;  
	uint8_t shoot_outpost  : 1; 
	uint8_t shoot_base     : 1;  

}Vision_ByteBits_t;


//typedef __packed struct
//{
// uint8_t gimbal_mode :2;
// uint8_t pitch_can  :1;
// uint8_t shoot_mode :1;
// uint8_t vision_mode :1;
// uint8_t robot_color :2;
//}small_control_data;


typedef enum
{ 
  protect=0,        //保护状态保持不动  用分区赛最简单的模式 protect
  extreme,        //激进模式  sentry_extreme_decision
  conservative,   // 保守模式 sentry_patrol_decision 分区赛决策加堡垒
  air_control,     // 云台手控制
  flying,         // 打前哨加飞坡落点 sentry_flying_decision
  patrol,         //打前哨加保护
}decision_mode_t;




extern float Red_Navi_position[DECISION_POSITION_NUM][2];
extern float Blue_Navi_position[DECISION_POSITION_NUM][2];


//均为1符合，0不符合

typedef struct
{   
  bool IF_Arrived;                 //判断哨兵是否到达指定位置
  
  bool IF_3s_NotHurted;           //判断哨兵是否未受击超过3s
  bool IF_10s_NotHurted;           //判断哨兵是否未受击超过10s
  bool IF_5s_NotHurted;           //判断哨兵是否未受击超过5s
  bool IF_3s_NotFound;             //判断哨兵是否未发现敌人超过3s  
  bool IF_5s_NotFound;             //判断哨兵是否未发现敌人超过5s
  bool  IF_10s_NotFound;            //判断哨兵是否未发现敌人超过10s
  bool IF_HP_Less_50;              //判断哨兵是否血量低于50  
  bool IF_HP_Less_100;             //判断哨兵是否血量低于100
  bool IF_base_armor_spred;        //判断己方基地护甲是否展开
  bool IF_outpost_destroyed;       //判断前哨站是否被击毁
  bool IF_fire_lock;                //判断发射机构是否锁住
 
  bool IF_allowance_less_50;        // 判断允许发弹量是否小于50
  bool IF_allowance_less_100;        // 判断允许发弹量是否小于100
  bool IF_HP_recover;               // 判断是否回血完成
  
  bool If_on_toss;                  // 判断是否在中央荒地上
  bool If_need_to_enemy_fortress;   // 判断是否需要上敌方堡垒//todo
  bool If_stop_navi;                // 判断是否停下来击打敌人
  bool If_chassis_weak;             // 判断是否进入虚弱模式
  bool If_get_allow_17;             // 判断是否需要补给区补给弹丸
  bool IF_fortress_allow_less_50;   //判断堡垒增益点
  bool IF_energy_Mechanism;         // 判断是否需要给打符的车让位   打符时间到且打符点附近有步兵的时候
  bool IF_need_to_protect;          // 判断是否需要去保护基地
  bool If_fortress_free;            // 堡垒增益区是否空闲
  bool If_enemy_outpost_lock;       // 判断敌方前哨站是否停转
  bool IF_enemy_outpost_destroyed;  // 判断对方前哨站是否被摧毁
  bool If_moving_v;                 // 判断是否正在过u型弯
  bool If_chip_base;                // 判断基地是否在吊射基地
  bool If_hp_less_200;              // 判断血量是否小于300
  bool If_enemy_small_energy;       // 判断敌方是否开了小能量机关
  bool If_close_to_enemy_out; // 判断是否距离敌方前哨站较近
  
//  bool If
  
}Judge_condition_t;


//#define find 9


 
extern uint16_t last_dart_time;
extern uint16_t lock_dart_count;
extern uint8_t if_random_dart;
extern uint16_t Last_base_hurt_time;
typedef struct
{
 robot_color_t robot_color; 
}robot_data_t;

bool if_navi_get_new_point();

typedef struct
{
  uint8_t Die_cnt;
  uint8_t If_revive;//是否读条复活
  uint8_t If_Immediately_Revive;
  uint16_t Exchange_Projectile_Num;//哨兵将要兑换的发弹量值
  uint8_t If_remote_exchange_HP;//哨兵远程兑换的血量
  bool if_update;//用来判断裁判系统是否响应了哨兵的自主决策
}Cmd_condition_t;




typedef struct
{
  Vision_ByteBits_t Vision_ByteBits;
  Judge_condition_t Judge_condition;
  decision_mode_t decision_mode;
  decision_mode_t last_decision_mode;
  Cmd_condition_t Cmd_condition;
  uint8_t point;
  uint8_t last_point;
  uint8_t if_point_new;
//  uint8_t last_point;
  robot_data_t robot_data;
  uint8_t top_senior_priority;//自瞄最高优先级车体
  bool If_point_change;             // 判断决策点是否切换
  bool keyboard_disable;            // 一键失能   
}decision_t;
extern decision_t decision;
//extern robot_data_t robot_data;

extern Sentry_cmd_t Sentry_cmd_send;
extern uint8_t if_navi_receive;
extern bool IF_navi_target_renew;

//击打决策相关
static bool judge_if_shoot_sentry(int16_t robot_hp,int16_t outpost_HP);
static bool judge_if_shoot_infantr4(int16_t robot_hp);
static bool judge_if_shoot_hero(int16_t robot_hp);
static bool judge_if_shoot_Engineer(int16_t robot_hp);
static bool judge_if_shoot_infantr3(int16_t robot_hp);
//void judge_shoot_top_senior_priority(decision_t *mode);
bool judge_if_shoot_outpost(uint16_t outpost_hp);
bool judge_if_shoot_base(int16_t outpost_hp);
void judge_shoot_top_senior_priority(decision_t *mode);
void get_referr_data();
void judge_if_shoot(decision_t *mode);
void sentry_shoot_decision(decision_t *mode);
void Sentry_cmd_decision(decision_t *mode);
void decision_point_chose(decision_t *mode);
void judge_if_keyboard_disable(decision_t *mode);//判断是否一键失能
//决策条件相关
bool judge_if_nothurt(Judge_condition_t *mode);
void judge_if_3s_not_found(Judge_condition_t *mode);
void judge_if_5s_not_found(Judge_condition_t *mode);
void judge_if_sentry_recover(Judge_condition_t *mode,uint16_t sentry_hp);
void judge_if_location(Judge_condition_t *mode);
void judge_if_HP_less_100(Judge_condition_t *mode);
void judge_base_armor_spred(Judge_condition_t *mode, int16_t base_HP);
void judge_if_outpost_destroyed(Judge_condition_t *mode,int16_t outpost_HP );
void judge_if_fire_lock(Judge_condition_t *mode);
void judge_if_chip_base(uint16_t base_hp,Judge_condition_t *mode);
void judge_if_HP_less_50(Judge_condition_t *mode);
void judge_if_HP_recover(Judge_condition_t *mode);
void sentry_test_decision(decision_t *mode);
void judge_if_stop_navi();
void AGV_auto_chassis(decision_t *mode);
void IF_decision_point_change();
void Decison_State_Ctl(decision_t *mode);
void Judge_Continuous_Handle(Judge_condition_t *mode);
void Decision_Init(decision_t *mode);
void Decison_State_Ctl(decision_t *mode);
void If_Point_arrived();
void judge_if_chassis_weak();
void judge_if_base_hurted(uint16_t base_hp,Judge_condition_t *mode);
void judge_if_on_toss(Judge_condition_t *mode);
void map_control_fill(decision_t *mode);
void air_control_ctl(decision_t *mode);
void judge_if_fortress_free(Judge_condition_t *mode);
void judge_if_enemy_outpost_lock(Judge_condition_t *mode);
void judge_if_need_to_protect(Judge_condition_t *mode);
void judge_if_location_over(Judge_condition_t *mode);
void judge_if_moving_v(Judge_condition_t *mode);

void judge_if_allowance_less_50(Judge_condition_t *mode);
void judge_if_allowance_less_100(Judge_condition_t *mode);
void judge_if_enemy_outpost_destroyed(Judge_condition_t *mode,int16_t enemy_outpost_HP);
void judge_if_fortress_allow_less_50(Judge_condition_t *mode);
//根据不同的模式进行决策
void sentry_protect_decision(decision_t *mode);
void sentry_guard_decision(decision_t *mode);
void sentry_attach_decision(decision_t *mode);
void sentry_attack_decision(decision_t *mode);
void sentry_att_outpost_decision(decision_t *mode);
void sentry_extreme_decision(decision_t *mode);
void sentry_air_control_decision(decision_t *mode);
void sentry_patrol_decision(decision_t *mode);
void decision_point_fill();
void AGV_auto_mode(decision_t *mode);
void sentry_flying_decision(decision_t *mode);
void sentry_guard_decision(decision_t *mode);
void sentry_conservative_decision(decision_t *mode);
void sentry_two_point_decision(decision_t *mode);
bool judge_if_enemy_small_energy(Judge_condition_t *mode,int16_t outpost_HP );
#endif