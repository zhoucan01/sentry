#include "shoot.h"



#include "bsp_pid.h"
#include "system.h"
#include "CAN_receive.h"
#include "ins_task.h"
#include "cmsis_os.h"
#include "struct_typedef.h"
#include "stm32f4xx_hal.h"
#include "remote_control.h"
#include "Nautilus_Vision.h"

#include "stdbool.h"

shoot_t stander_shoot=
{
	.shoot_speed_set=0,
	.if_shoot_speed_yes=0,
	.get_now_speed=0,
	.shoot_speed_fix=0.0f,
};
pid_struct_t pid_shoot_speed[2];

com_mode_t shoot_com;

void shoot_run()
{
vTaskDelay(1);
	shoot_pid_init();
	for(;;)
	{
//		sssss++;
		if_shoot_com();
		switch(shoot_com)
		{
			case com_nom:
			{
				shoot_speed_set(&stander_shoot);
				shoot_pid_clac();
				
				break;
			}
			case com_err:
			{
				stander_shoot.shoot_speed_set=0;
				shoot_pid_clac();
//				set_motor_current(can2_200,pid_shoot_speed[0].output,
//				pid_shoot_speed[1].output
//				,0,0);
				break;
			}
		}
			
		
		vTaskDelay(1);
	}
	
}
int ioijij;
int get_over_cnt;
void shoot_pid_init()
{
	char i;
	
		pid_init( &pid_shoot_speed[0] ,5, 0.0f, 0, PID_shoot_IMAX, PID_shoot_MAX );
		pid_init( &pid_shoot_speed[1] ,5, 0.0f, 0, PID_shoot_IMAX, PID_shoot_MAX );
	
	
}

int fire_num=0;
bool if_num_updata=NO;
float over_speed;
void shoot_speed_set(shoot_t *mode)
{
	
	if(fire_num!=control_data.shoot_num)
	{
		if_num_updata=YES;
		fire_num=control_data.shoot_num;
		
	}
	else if_num_updata=NO;
//	
	if(if_num_updata==YES&&control_data.shoot_speed>15)
	{
		ioijij++;
		Temp_Fix_30S();
		SpeedAdapt(control_data.shoot_speed,MIN_SPEED , MAX_SPEED ,&(mode->shoot_speed_fix) , UP_NUM , DOWN_NUM);
	}
	
	if(control_data.shoot_mode==shoot_on)
	{
		mode->get_now_speed=control_data.shoot_speed;
		if(mode->get_now_speed>25)
		{
      over_speed=mode->get_now_speed;
			get_over_cnt++;
		}
		
		
		mode->shoot_speed_set=FRICTION_L3_SPEED+mode->shoot_speed_fix+mode->shoot_tem_fix;
//		
	}
	else if(control_data.shoot_mode==shoot_no)
	{
		mode->shoot_speed_set=0;
		
	}
	
}



void if_shoot_com(void)
{

	  if(control_data.shoot_mode!=shoot_no&&gimbal_system.control_com==com_nom)
	{
    shoot_com=com_nom;
  }
  else shoot_com=com_err;
}




void shoot_pid_clac(void)
{
	if(control_data.shoot_mode==shoot_on)
  {
  
    shoot_motor[0].motor_tar.set_current=pid_calc(&pid_shoot_speed[0],
    shoot_motor[0].motor_measure.speed_rpm,stander_shoot.shoot_speed_set);
    shoot_motor[1].motor_tar.set_current=pid_calc(&pid_shoot_speed[1],
    shoot_motor[1].motor_measure.speed_rpm,-stander_shoot.shoot_speed_set);
	}
  else if(control_data.shoot_mode==shoot_no)
  {
  shoot_motor[0].motor_tar.set_current=pid_calc(&pid_shoot_speed[0],
    shoot_motor[0].motor_measure.speed_rpm,0);
    shoot_motor[1].motor_tar.set_current=pid_calc(&pid_shoot_speed[1],
    shoot_motor[1].motor_measure.speed_rpm,0);
  }
}




//反馈3508电机是否达到目标转速
bool Report_IF_Fric3508_SetSpeed(void)
{
  bool res = 0;
  if(fabs(shoot_motor[0].motor_measure.speed_rpm - stander_shoot.shoot_speed_set) < 500 && 
		fabs(-shoot_motor[1].motor_measure.speed_rpm - (stander_shoot.shoot_speed_set)) < 500
    &&abs(shoot_motor[0].motor_measure.speed_rpm)>4000
    &&abs(shoot_motor[1].motor_measure.speed_rpm)>4000)
  {
		res = 1;
	}
  else 
	{
		res = 1;
	}
  return res;
}

uint8_t SpeedErr_cnt=0;
int bbbbb;
void SpeedAdapt(float real_S , float min_S, float max_S,float *fix , float up_num , float down_num)
{
  if(real_S < min_S && real_S > 8)
    SpeedErr_cnt++;
  else if(real_S >= min_S && real_S <= max_S )
		SpeedErr_cnt = 0;
  if(SpeedErr_cnt == 1)//射速偏低
  {
    SpeedErr_cnt = 0;
    *fix += up_num;
  }
  if(real_S > max_S)//射速偏高
    *fix -= down_num;
}


/**
 * @brief 检查射速
 * @note  摩擦轮变速，动态调节转速
 */
void Temp_Fix_30S(void)
{
  float temp_scope = 35;//假设变化范围为35摄氏度
  float temp_low = 35;//初始温度设定为35摄氏度
  float res = 0;
  float temp_real;
  
  temp_real = ((float)shoot_motor[0].motor_measure.temperate + 
               (float)shoot_motor[1].motor_measure.temperate)/2;//平均温度
//  temp_real = (float)Shoot.Friction.moto_Fric[1].temp;

  if(temp_real >= temp_low)
    res = (temp_real - temp_low)/temp_scope * (-168);
  if(temp_real < temp_low)
    res = 0;
  if(temp_real > temp_low + temp_scope)
    res = -168;
  
  stander_shoot.shoot_tem_fix = res;
}

///*返回射速，单位m/s*/
//float Report_RealShootSpeed(void)
//{
//	return shoot_data.initial_speed;
//}