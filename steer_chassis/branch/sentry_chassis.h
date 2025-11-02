#ifndef __SENTRY_CHASSIS_H
#define __SENTRY_CHASSIS_H
#include "system.h"


typedef struct
{
	float Max_PowerBuffer;
	float Real_PowerBuffer;
	float Limit_k;
	float CHAS_LimitOutput;
	float CHAS_TotalOutput;
}Power_Limit_t;




//typedef struct
//{
//  
//  
//}

#define K_Low_setSteer  0.05
#define K_Low_setchassis  0.03

/**
*@nite   Pm=kp*Icmdω+kw*ω2+ki*Icmd2
*@note   motor_3508_kp  电机常数，根据西交开源在电机手册中寻找各个参数来计算
*@note   motor_3508_kw  电机转速系数 根据matlab拟合出参数
*@note   motor_3508_ki  电机力矩电流系数 根据matlab拟合出参数
*@note   constant       电机的空载功率
*/
#define motor_3508_kw 2.0854e-07
#define motor_3508_ki 5.0323e-07

//#define motor_3508_kw 6.1485e-07
//#define motor_3508_ki 2.2471e-07
#define motor_3508_kp 1.99688994e-6f
#define motor_3508_constant 0.72


#define motor_6020_kw   8.9596e-07
#define motor_6020_ki  3.0609e-07
#define motor_6020_kp 1.42074e-6f
#define motor_6020_constant 0.69


#define CONST__MAX_POWERBUFFER                 	60
#define CONST__POWERBUFFER_Urgent              	10         
#define CONST__CHASSIS_TOTAL_OUTPUT_MAX        	90000	// 底盘最大总输出 50000
void chassis_pid_init();
#define communicate 
void speed_in_reslove(chassis_t *mode);
void chassis_speed_get(chassis_t *mode);
void steer_angle_get(chassis_t *mode);
void chassis_clac(chassis_t *mode);
void chassis_speed_set(chassis_t *mode);
void speed_in_reslove_1(chassis_t *mode);
extern float real_chassis;
void chassis_power_limit_set(void);
void chassis_power_get();
void chassis_weak_handle(chassis_t *mode);
float LowPass_SetSteer(float old,float In);
float LowPass_SetChassis(float old,float In);
extern float remain_power;
 extern float get_power;
 extern float get_speed_vx;
extern float get_speed_vy;
extern int ffff;

int chassis_real_move_get();
float limit_addspeed1(float speed_set,float speed_ref,float addspeed_limit);
float limit_addspeed(float speed_set,float speed_ref,float addspeed_limit);
void chassis_speed_weak_handle(chassis_t *mode);
void move_state_change(chassis_t *mode);
#endif