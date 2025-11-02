#ifndef __OMNI_H
#define __OMNI_H

#define install_direction 1  //正对着的
//根据安装方式来进行改变，如若正前方第一个电机是正方向，则为1，若不是则为-1；

#define forward_angle 2000
#define back_ecd      2000+4096

//逆时针解算为右前，左前，左后，右后
enum{
	forward=0,
	left,
	back,
	right,
};

enum{
	r_f=0,
	l_f,
	l_b,
	r_b,
};
typedef enum
{
	no_move=0,//用来总控是否发电流
	follow_mode,
	no_follow_mode,
	around_mode,
	stop_move,//只有底盘不发电流模式
	
}chassis_mode_t;

typedef struct
{
	float vx;
	float vy;
	float wz;
}omni_speed_t;
//四个轮子按照前后左右逆时针排列
typedef struct
{
	omni_speed_t omni_speed;
	
	float diff_ecd;
	float diff_angle;
	float Vx_calc;
	float Vy_calc;
	float Wz_calc;
	
	float speed_f;
	float speed_l;
	float speed_b;
	float speed_r;
	
	float speed_rf;
	float speed_lf;
	float speed_lb;
	float speed_rb;
	chassis_mode_t chassis_mode;
}omni_t;
void omni_no_move_speed_set(omni_t *mode);
void omni_follow_speed_set(omni_t *mode);
void omni_no_follow_speed_set(omni_t *mode);
void omni_top_speed_set(omni_t *mode);
void omni_speed_set(omni_t *mode);
typedef struct
{
	float vx;
	float vy;
}system_t;
void omni_speed_reslove(omni_t *mode);
void omni_chassis_speed_set(omni_t *mode);
void omni_pid_calc(omni_t *mode);
void get_omnidiff_angle(omni_t *mode);
extern omni_t omni;
#endif


