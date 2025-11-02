#ifndef __TRIG_H
#define __TRIG_H


#include "string.h"
#include <stdlib.h> 
#include "math.h"
#include "stdbool.h"
#include "bsp_pid.h"
//#include ""
#define trig_sin_ecd 32700

/******拨弹盘相关的参数******/
#define trig_1rps_speed 2160  //一秒转一圈的速度

#define trig_num 8  // 拨盘格数
#define trig_1fps_speed (trig_1rps_speed/trig_num)
//#define trig_motor_power_limit 35


//一秒转   发
#define con_freq_22 22
#define con_freq_20 20
#define con_freq_18 18
#define con_freq_10 10
#define con_freq_15 15
#define con_freq_12 12
#define con_freq_1   1
#define con_freq_5   5

#define trig_max_power         400
#define trig_outpost_power_limit 250
#define trig_normol_power_limit 80
extern int get_block_cnt;
extern int trig_ecd_set;
extern pid_struct_t pid_trig_sin_ecd;;
extern int ggggiii;;
extern float trig_speed_set;
typedef enum
{
	fire_no=0,
	fire_sin,
	fire_con,
  fire_vision,

}fire_state_t;

typedef enum
{
	no_block=0,
	sin_block,
	con_block,
	
	
}block_type_t;

//卡弹相关
typedef struct
{
	block_type_t block_type;
	
	bool if_fire_block;
	bool if_block_over;
	int cnt;
	
}block_state_t;


typedef struct
{
	bool if_sin_over;
	bool if_con_over;
	bool if_sin_request;
	bool if_con_request;
	bool if_sin_block;
	bool if_con_block;
	bool if_block_react;
	bool if_block_react_over;
	
}trig_flag_t;

//拨弹状态
typedef struct
{
  fire_state_t fire_state;
	fire_state_t last_state;
	bool if_block;
	block_state_t block_state;
	trig_flag_t trig_flag;
	int trig_freq;

}trig_t;
extern       int ddddmmm;
//extern float last_trig_set;
extern int get_trig_cnt;;
extern trig_t trig;
//extern int8_t trig_motor_power_limit;
void trig_mode_chose(trig_t *mode);
bool judge_if_sin_over(void);
void judge_con_block(void);
void judge_sin_block(void);
void tirg_control(trig_t *mode);
bool if_trig_block(void);
void get_trig_motor_state(trig_t *mode);
void trig_sin(trig_t *mode);
void  trig_con(trig_t *mode);
void trig_chose_freq(trig_t *mode);
void block_react(trig_t *mode);
extern int gugugu;
void sin_block_react(trig_t *mode);
void con_block_react(trig_t *mode);
void reset_block_flag(trig_t *mode);
void trig_motor_run(trig_t *mode);
void trig_task_run(trig_t *mode);
void judge_if_block(void);
bool judge_if_sin_over();
float trig_addspeed_limit(float speed_set,int add_speed);
void get_trig_com();
void shoot_rc_mode(trig_t *mode);
void shoot_pc_mode(trig_t *mode);

void trig_vision(trig_t *mode);
#endif