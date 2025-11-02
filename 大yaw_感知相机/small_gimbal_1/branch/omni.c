#include "omni.h"
#include "system.h"
#include "CAN_receive.h"
#include "remote_control.h"
#include "bsp_pid.h"
#include "math.h"
#include "bsp_transmit.h"
#include "struct_typedef.h"
omni_t omni;
pid_struct_t pid_follow;
chassis_mode_t chassis_mode;
//#define omni_system
#define omni_45

pid_struct_t omni_chassis_pid[4];

system_t system;
void omni_speed_get(void)
{
	pid_init(&pid_follow,10,0,0,0,100);
	for(int i=0;i<4;i++)
	{
		pid_init(&omni_chassis_pid[i],5.25,0,0,0,16000);
	}
	
	while(1)
	{
			get_omnidiff_angle(&omni);
			omni_speed_set(&omni);
			omni_speed_reslove(&omni);
		  omni_chassis_speed_set(&omni);
		  omni_pid_calc(&omni);
		
	}
}


void get_omnidiff_angle(omni_t *mode)
{omni.diff_ecd=yaw_motor.motor_measure.ecd-forward_angle;
	mode->diff_angle = mode->diff_ecd/8192*2*3.1415;
	
}

void omni_speed_set(omni_t *mode)
{
	
	
	
	#ifdef omni_system
	
	if(rc_ctrl.rc.s_l==RC_SW_DOWN)
	{mode->omni_speed.wz=0;
	mode->omni_speed.vx=0;
	mode->omni_speed.vy=0;}
	else if(rc_ctrl.rc.s_l==RC_SW_MID)
	{mode->omni_speed.vx=rc_ctrl.rc.ch0;
	mode->omni_speed.vy=rc_ctrl.rc.ch1;
	mode->omni_speed.wz+=pid_follow.output;}
	else if(rc_ctrl.rc.s_r==RC_SW_UP)
	{mode->omni_speed.wz=200;
	}
	#else 
	if(mode->chassis_mode==no_move)
	{
		omni_no_move_speed_set(mode);
	}
	else if(mode->chassis_mode==follow_mode)
	{
		omni_follow_speed_set(mode);
		
	}
	else if(mode->chassis_mode==no_follow_mode)
	{
		omni_no_follow_speed_set(mode);
	}
	else if(mode->chassis_mode==around_mode)
	{
		omni_top_speed_set(mode);
	}
	
	#endif
	
}


void omni_no_move_speed_set(omni_t *mode)
{
	
	mode->omni_speed.wz=0;
		mode->omni_speed.vx=0;
		mode->omni_speed.wz=0;
	
	
}
void omni_follow_speed_set(omni_t *mode)
{
	mode->omni_speed.wz=pid_calc(&pid_follow,omni.diff_ecd,0);
	
	
	mode->omni_speed.vx=system.vx;
	mode->omni_speed.vy=system.vy;
}


void omni_no_follow_speed_set(omni_t *mode)
{
	mode->omni_speed.wz=0;
	
	mode->omni_speed.vx=system.vx;
	mode->omni_speed.vy=system.vy;
}

void omni_top_speed_set(omni_t *mode)
{
	mode->omni_speed.vx=system.vx;
	mode->omni_speed.vy=system.vy;
	mode->omni_speed.wz=1800;
	//根据功率来判断小陀螺转速
//	switch(robot_status.chassis_power_limit)
//	{
//		case 45:
//			stander_chassis.Wz_set = 1800;
//			break;
//		case 50:
//			stander_chassis.Wz_set = 2300;
//			break;
//		case 55:
//			stander_chassis.Wz_set = 2300;
//			break;
//		case 60:
//			stander_chassis.Wz_set = 2500;
//			break;
//		case 65:
//			stander_chassis.Wz_set = 2500;
//			break;
//		case 70:
//			stander_chassis.Wz_set = 3000;
//			break;
//		case 75:
//			stander_chassis.Wz_set = 3000;
//			break;
//		case 80:
//			stander_chassis.Wz_set = 3400;
//			break;
//		case 85:
//			stander_chassis.Wz_set = 3400;
//			break;
//		case 90:
//			stander_chassis.Wz_set = 3800;
//			break;
//		case 95:
//			stander_chassis.Wz_set = 3900;
//			break;
//		case 100:
//			stander_chassis.Wz_set = 4500;
//			break;
//		case 120:
//			stander_chassis.Wz_set = 4500;
//			break;
//		default:
//			stander_chassis.Wz_set = 1800;
//			break;
//	}
	
	
	
}


void omni_speed_reslove(omni_t *mode)
{
	
	mode->Vy_calc = -(mode->omni_speed.vx*sin(mode->diff_angle))+
	mode->omni_speed.vy*cos(mode->diff_angle);
	
	mode->Vx_calc = (mode->omni_speed.vx*cos(mode->diff_angle))+
	mode->omni_speed.vy*sin(mode->diff_angle);
	
	mode->Wz_calc = mode->omni_speed.wz;

}




/*
   轮子摆的方向
    逆时针
   四个轮子这样摆为正(前轮对着正方向)
   ______
  |      |
  |      |
  |______|  


*/

#ifdef omni_45

void omni_chassis_speed_set(omni_t *mode)
{
	mode->speed_f=mode->Vy_calc*-(install_direction)+mode->omni_speed.wz;
	mode->speed_l=mode->Vx_calc*-(install_direction)+mode->omni_speed.wz;
	mode->speed_b=mode->Vy_calc*(install_direction)+mode->omni_speed.wz;
	mode->speed_r=mode->Vx_calc*install_direction+mode->omni_speed.wz;
}

void omni_pid_calc(omni_t *mode)
{
	chassis_motor[forward].motor_tar.set_current = pid_calc(&omni_chassis_pid[forward],
	chassis_motor[forward].motor_measure.speed_rpm,mode->speed_f);
	
	chassis_motor[left].motor_tar.set_current = pid_calc(&omni_chassis_pid[left],
	chassis_motor[left].motor_measure.speed_rpm,mode->speed_l);
	
	chassis_motor[back].motor_tar.set_current = pid_calc(&omni_chassis_pid[back],
	chassis_motor[back].motor_measure.speed_rpm,mode->speed_b);
	
	chassis_motor[right].motor_tar.set_current = pid_calc(&omni_chassis_pid[right],
	chassis_motor[right].motor_measure.speed_rpm,mode->speed_r);
	
}
#else 

//第二种即四个轮子分别在右前，左前，左后，右后四个方向

void omni_chassis_speed_set(omni_t *mode)
{
	mode->speed_rf=mode->Vx_calc*sin(4/3.1415)
	              -mode->Vy_calc*sin(4/3.1415)+mode->Wz_calc;
	mode->speed_lf=-mode->Vx_calc*sin(4/3.1415)
	-mode->Vy_calc*sin(4/3.1415)+mode->Wz_calc;
	mode->speed_lb=-mode->Vx_calc*sin(4/3.1415)
	+mode->Vy_calc*sin(4/3.1415)+mode->Wz_calc;
	mode->speed_rb=mode->Vx_calc*sin(4/3.1415)
	+mode->Vy_calc*sin(4/3.1415)+mode->Wz_calc;
}


void omni_pid_calc(omni_t *mode)
{
	chassis_motor[r_f].motor_tar.set_current=pid_calc(&omni_chassis_pid[r_f],
	chassis_motor[r_f].motor_measure.speed_rpm,mode->speed_rf);
	chassis_motor[l_f].motor_tar.set_current=pid_calc(&omni_chassis_pid[l_f],
	chassis_motor[l_f].motor_measure.speed_rpm,mode->speed_lf);
	chassis_motor[l_b].motor_tar.set_current=pid_calc(&omni_chassis_pid[l_b],
	chassis_motor[l_b].motor_measure.speed_rpm,mode->speed_lb);
	chassis_motor[r_b].motor_tar.set_current=pid_calc(&omni_chassis_pid[r_b],
	chassis_motor[r_b].motor_measure.speed_rpm,mode->speed_rb);
	
}


#endif


//void omni_speed_set()







