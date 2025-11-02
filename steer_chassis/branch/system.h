#ifndef __SYSTEM_H
#define __SYSTEM_H
      /*舵机角度初始化使底盘朝正方向*/
#define steer_init_angle_FR  7662          //对应360-45
#define steer_init_angle_BR  5791          //45
#define steer_init_angle_BL  5218          //对应135
#define steer_init_angle_FL  3132         //对应360-135
//此时舵机正对着底盘的正前方，可以在此基础上计算舵机的偏移角

//#define init_yaw -104    //虚拟头的方向
#include "struct_typedef.h"
#include "bsp_pid.h"
typedef struct
{
	float vx;
	float vy;
	float wz;
}speed_t;
typedef enum
{
no_move=0,//底盘不动
normol_move,

navigation_move,//

}chassis_mode_t;

typedef enum
{
  no_spine=0,
  low_spine,
  mid_spine,
  high_spine,
  lock_spine,
}Vz_state_t;
typedef enum{
  move_on=0,
  move_off,
  
}real_move_state_t;
typedef struct
{
	chassis_mode_t chassis_mode;
	float init_yaw;
  
	speed_t speed_in;
  speed_t speed_usart_get;
	speed_t speed_reslove;
  speed_t last_reslove;
  float dm_ecd;
	float diff_angle;
	uint8_t if_chassis_open;
Vz_state_t Vz_state;
	speed_t chassis[4];
	float speed_set[4];
  float last_speed_set[4];
	float steer_angle[4];
	float steer_ecd[4];
  float steer_final_ecd[4];
  float decay_ecd[4];   //舵角衰减系数
	float speed_direct[4];
	float steer_resolve_ecd[4];
	float steer_speed_set[4];
	float last_ecd[4];
	int speed_flag[4];
  float chassis_last_total_ecd[4];
  float chassis_diff_ecd[4];
  float Sx,Sy;
  float chassis_ecd_diff[4];
  uint8_t game_progress;
  uint8_t if_outpost_des;
  int8_t wz_back;
  float steer_diff_angle;
  int move_flag;
  real_move_state_t real_move_state;
}chassis_t;
#define WHEEL_PERIMETER 			 376.99f	//车轮周长 (轮子直径 * PI 再转换成mm)
#define M3508_RATIO 	 				 19.2032				//电机减速比
#define Radius 								 62				//轮径mm  半径
#define distance_x             180/1000      //舵轮中心到底盘中心的距离
#define distance_y             180/1000     //


#define WHEEL_FACTOR 0.00034   // 电机转速单位rpm转换为真实速度m/s 的常数

void chassis_navi_set(chassis_t *mode);
#define forword_ecd -2.687
extern chassis_t chassis;
extern chassis_mode_t last_chassis_mode;
void sentry_chassis_init();
extern pid_struct_t pid_steer_ecd[4];
extern pid_struct_t pid_steer_speed[4];
void get_diff_angle(chassis_t *mode);
void chassis_data_updata(chassis_t *mode);
void chassis_normol_set(chassis_t *mode);
void chassis_follow_set(chassis_t *mode);
void chassis_spine_set(chassis_t *mode);
void chassis_mode_chose(chassis_t *mode);
void chassis_no_move_set(chassis_t *mode);
void chassis_mode_updata(chassis_t *mode);
void chassis_lock_move(chassis_t *mode);
void chassis_low_spine_set(chassis_t *mode);
float speed_filter(float speed_in);
void chassis_mid_spine_set(chassis_t *mode);
//void chassis_weak_handle(chassis_t *mode);
#endif

