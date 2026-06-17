#include "system.h"
#include "remote_control.h"
#include "ins_task.h"
#include "CAN_receive.h"
//#include "task.h"
#include "cmsis_os.h"
#include "bsp_pid.h"
#include "bsp_transmit.h"
#include "Odometer.h"
#include "sentry_chassis.h"
#include "main.h"
#include "user_lib.h"
#include "detect_task.h"
#include "CAN_transmit.h"
#include "detect_task.h"
//pid_struct_t pid_steer_ecd[4];
//pid_struct_t pid_steer_speed[4];
//pid_struct_t pid_chassis_speed[4];
//pid_struct_t pid_chaais_Sx;
//pid_struct_t pid_chaais_Sy;
//pid_struct_t pid_chassis_follow;
/**两种模式 1 无速度输入后底盘舵电机抱圆锁死 2 无速度输入后底盘电机为有速度输入的最后一秒的角度**/
#define communicate 
//#define steer_lock
chassis_t chassis=
{
	.last_ecd[FR]=7888,/*舵机初始化使底盘朝向正方向,即电机装上时的偏置角*/
	.last_ecd[BR]=3126,
	.last_ecd[BL]=6368,//2380
	.last_ecd[FL]=50,
	.Vz_state=(Vz_state_t)0,
	.speed_direct[0]=1,
	.speed_direct[1]=1,
	.speed_direct[2]=1,
	.speed_direct[3]=1,
	.steer_ecd={0},
	.speed_in.vx=0,
	.speed_in.vy=0,
	.speed_in.wz=0,
	.diff_angle=0,
	.init_yaw=0,
	.chassis_last_total_ecd[0]=0,
  .chassis_last_total_ecd[1]=0,
  .chassis_last_total_ecd[2]=0,
  .chassis_last_total_ecd[3]=0,
  
  .chassis_diff_ecd[0]=0,
  .chassis_diff_ecd[1]=0,
  .chassis_diff_ecd[2]=0,
  .chassis_diff_ecd[3]=0,
  .wz_back=1,
  .move_flag=1,
  .real_move_state=move_off,
};
chassis_mode_t last_chassis_mode;
pid_struct_t pid_chassis_follow;
//int gggg;

first_order_filter_type_t filter_pm;

float fil_pm;
void system_run()
{
float pm_kp=1.0;
first_order_filter_init(&filter_pm,0.02,&pm_kp);
	pid_init(&pid_chassis_follow,3000,140,0,0,3000);
	for(;;)
	{
  chassis_data_updata(&chassis);
  get_diff_angle(&chassis);//解算出云台和地盘的夹角
  chassis_mode_updata(&chassis);
  
  
  USART_TX_send(Tx_Buff,&USART_Tx_data);
  
  /*拟合数据用，拟合后注掉即可*/
//  first_order_filter_cali(&filter_pm,pm_power);
//  fil_pm=filter_pm.out;
////		gggg++;
//		get_diff_angle(&chassis);
//		speed_in_reslove(&chassis);
//		chassis_speed_get(&chassis);
//		steer_angle_get(&chassis);
//		chassis_clac(&chassis);
		osDelay(1);
	}
	
}

float get_diff_yaw=0;
void sentry_chassis_init()
{
	
  chassis.Sx=0;
  chassis.Sy=0;
  chassis.chassis_last_total_ecd[0]= chassis_motor[0].motor_measure.total_ecd;
  chassis.chassis_last_total_ecd[1]= chassis_motor[1].motor_measure.total_ecd;
  chassis.chassis_last_total_ecd[2]= chassis_motor[2].motor_measure.total_ecd;
  chassis.chassis_last_total_ecd[3]= chassis_motor[3].motor_measure.total_ecd;
  
}
uint8_t last_seq;
uint16_t lost_time=0;
uint16_t lkld;


float steer_can_diff;
int steer_if_ok(int steer_set_current , int get_steer_current)
{
  int steer_ok;
//  steer_can_diff = abs(steer_set_current - get_steer_current);
  if(abs(steer_set_current - get_steer_current)<1000)
  {
    steer_ok = 1;
    
  }
  else 
  {
    steer_ok =0;
    steer_can_diff++;
  }
  return steer_ok;
}

void chassis_data_updata(chassis_t *mode)
{

 
 
 if(last_seq==transmit_seq)
 {
   lost_time++;
 }
 else lost_time=0;
 
// steer_if_ok(steer_motor[0].motor_tar.set_current,steer_motor[0].motor_measure.feedback_current);
 
 if( ( lost_time>1000)
    ||(toe_offline[CHASSIS_MOTOR1_TOE].communication_state==COMMUNICATION_NONE)
    ||(toe_offline[CHASSIS_MOTOR2_TOE].communication_state==COMMUNICATION_NONE)
    ||(toe_offline[CHASSIS_MOTOR3_TOE].communication_state==COMMUNICATION_NONE)
    ||(toe_offline[CHASSIS_MOTOR4_TOE].communication_state==COMMUNICATION_NONE)
//    || error_time>100
//    ||steer_if_ok(steer_motor[0].motor_tar.set_current,steer_motor[0].motor_measure.feedback_current)==0
//    ||steer_if_ok(steer_motor[1].motor_tar.set_current,steer_motor[1].motor_measure.feedback_current)==0
//    ||steer_if_ok(steer_motor[2].motor_tar.set_current,steer_motor[2].motor_measure.feedback_current)==0
////    ||steer_if_ok(steer_motor[3].motor_tar.set_current,steer_motor[3].motor_measure.feedback_current)==0
//    ||(toe_offline[STEER_MOTOR1_TOE].communication_state==COMMUNICATION_NONE)
//    ||(toe_offline[STEER_MOTOR2_TOE].communication_state==COMMUNICATION_NONE)
//    ||(toe_offline[STEER_MOTOR3_TOE].communication_state==COMMUNICATION_NONE)
//    ||(toe_offline[STEER_MOTOR4_TOE].communication_state==COMMUNICATION_NONE)
    )

 { 
 
  mode->chassis_mode=no_move;
  mode->speed_in.vx=0;
  mode->speed_in.vy=0;
  lkld++;
//  mode->if_chassis_weak=1;
 }

 else 
 {
  mode->chassis_mode=(chassis_mode_t)USART_Rx_data.chassis_mode;
//  mode->if_chassis_weak=1;

  mode->speed_usart_get.vx=limit_addspeed(speed_filter(USART_Rx_data.chassis_vx/WHEEL_FACTOR*2.5f),mode->speed_usart_get.vx,60.0f);
  mode->speed_usart_get.vy= limit_addspeed(speed_filter(USART_Rx_data.chassis_vy/WHEEL_FACTOR*2.5f),mode->speed_usart_get.vy,60.0f);
  chassis_weak_handle(mode);
  mode->if_chassis_open=USART_Rx_data.if_chassis_open;
  mode->Vz_state=USART_Rx_data.Vz_state;
  mode->dm_ecd=USART_Rx_data.big_yaw_ecd;
  mode->init_yaw=USART_Rx_data.up_yaw;
  mode->game_progress=USART_Rx_data.game_progress;
 }
 
  last_seq=transmit_seq;
}

void chassis_weak_handle(chassis_t *mode)
{

    mode->speed_in.vx=mode->speed_usart_get.vx;
    mode->speed_in.vy=mode->speed_usart_get.vy;

}
float speed_filter(float speed_in)
{
 float speed_out=0;
 if(speed_in<50&&speed_in>-50)
 {
   speed_out=0;
 }
 else speed_out=speed_in;
 return speed_out;
 
}

/* 归一化角度到 [-PI, PI] */
static inline float NormAngle(float a)
{
    while (a >  PI) a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
}

float follow_ecd = 0.0f;

void get_diff_angle(chassis_t *mode)
{
    float dm  = mode->dm_ecd;
    float fwd = forword_ecd;

    float diff = NormAngle(dm - fwd);
    if (diff < 0.0f) diff += 2.0f * PI;
    mode->diff_angle = 2.0f * PI - diff;

    follow_ecd = NormAngle(fwd - dm);
}
    
    
    

void chassis_mode_updata(chassis_t *mode)
{

   chassis_mode_chose(mode);

 
 last_chassis_mode=mode->chassis_mode;
}

float Last_Wz_speed;
void chassis_mode_chose(chassis_t *mode)
{
  switch(mode->chassis_mode)
  {

  
    case no_move:
    {
     chassis_no_move_set(mode);
     break;
    }
    case normol_move:
    {
    
//    chassis_lock_move(mode);

//   chassis_no_move_set(mode);
  chassis_follow_set(mode);
     break;
     
    }

    
    case navigation_move:
    {
    chassis_low_spine_set(mode);
//    chassis_navi_set(mode);
//    chassis_no_move_set(mode);
//        chassis_spine_set(mode);
//chassis_mid_spine_set(mode);
//chassis_lock_move(mode);
//     chassis_follow_set(mode);
     break;
    }
    
//    case 
  }   
  
  if(Last_Wz_speed==0&&mode->Vz_state!=no_spine)
  {
    mode->wz_back*=-1;
  }
  
  switch(mode->Vz_state)
  {
    case no_spine:
    {
     mode->speed_in.wz=0;
     
     break;
    }
    
    case low_spine:
    {
    
      mode->speed_in.wz=1500;
     break;      
    }
    
        case mid_spine:
    {
      mode->speed_in.wz=3000;
     break;      
    }
        case high_spine:
    {
      mode->speed_in.wz=5000;
//      mode->speed_in.wz=(4000+2000*fabs(sin(chassis_tim)));
//      mode->speed_in.wz = 
     break;      
    }
    case lock_spine:
    { 
     pid_calc(&pid_chassis_follow,follow_ecd,0);
     mode->speed_in.wz=limit_addspeed(-pid_chassis_follow.output,mode->speed_in.wz,25);
      break;
    }
    default:
    {
      mode->speed_in.wz=0;
      break;
    }
    
  }
  
//mode->speed_in.wz=0;


  
//  mode->speed_in.wz=limit_addspeed(mode->speed_in.wz,Last_Wz_speed,50);
    
    
    Last_Wz_speed=mode->speed_in.wz;
}



void chassis_mid_spine_set(chassis_t *mode)
{
   mode->speed_in.vx=mode->speed_in.vx;
   mode->speed_in.vy=mode->speed_in.vy;
   

}

void chassis_low_spine_set(chassis_t *mode)
{
 mode->speed_in.vx=mode->speed_in.vx;
 mode->speed_in.vy=mode->speed_in.vy;
 


}

void chassis_no_move_set(chassis_t *mode)
{ 
 mode->speed_in.vx=0;
 mode->speed_in.vy=0;

 
}
void chassis_normol_set(chassis_t *mode)
{
 mode->speed_in.vx=mode->speed_in.vx;
 mode->speed_in.vy=mode->speed_in.vy;

}
void chassis_follow_set(chassis_t *mode)
{
  mode->speed_in.vx=mode->speed_in.vx;
 mode->speed_in.vy=mode->speed_in.vy;

}
float yytt;
//为了在陀螺时移动把更多的功率分配给移动所以才将陀螺速度降低
void chassis_spine_set(chassis_t *mode)
{ 


  mode->speed_in.vx=mode->speed_in.vx;
  mode->speed_in.vy=mode->speed_in.vy;
//mode->speed_in.wz=(4000+1000*fabs(sin(chassis_tim)))*mode->wz_back;

}

void chassis_navi_set(chassis_t *mode)
{ 


  mode->speed_in.vx=mode->speed_in.vx;
  mode->speed_in.vy=mode->speed_in.vy;


}


void chassis_lock_move(chassis_t *mode)
{
 mode->speed_in.vx=mode->speed_in.vx;
 mode->speed_in.vy=mode->speed_in.vy;
 pid_calc(&pid_chassis_follow,follow_ecd,0);
 mode->speed_in.wz=limit_addspeed(-pid_chassis_follow.output,mode->speed_in.wz,25);
}
//void chassis_spine_change()
//{
// 
//}
void printf_task(void const * argument)
{
    while (1)
    {
//        printf("%f,%f,%f,%f,%f\n",
//               (float)error_time,
//               (float)steer_motor[0].motor_measure.feedback_current,
//               (float)steer_motor[0].motor_tar.set_current,
//               0.0f, 0.0f);
        osDelay(5);
    }
}
