#include "system.h"
#include "motor.h"
#include "CAN_receive.h"
//#include "task.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "CAN_transmit.h"
#include "SuperCAP.h"
#define steer
//#define chassis_open
#define supercap
#include "remote_control.h"
int ddddd;
void motor_run()
{
	for(;;)
	{
		
		#ifdef steer
		steer_normol_send();
		
		#else 
		steer_error_send();
		
		#endif
		
		#ifdef chassis_open
		chassis_normol_send();
		#else 
//		chassis_error_send();
		#endif
    vTaskDelay(1);
//    #ifdef supercap
//    Send_SupPower(&hcan1);
//    #else 
//    #endif
//		vTaskDelay(1);
	}
}

int psdmv;
void steer_normol_send()
{
	if(chassis.if_chassis_open==1&&chassis.chassis_mode!=no_move)
	{
		
		ddddd++;
//    set_motor_current(can2_1FE,0,0,
//	0,steer_motor[FR].motor_tar.set_current);
	set_motor_current(can2_1FE,steer_motor[FL].motor_tar.set_current,steer_motor[BL].motor_tar.set_current,
	steer_motor[BR].motor_tar.set_current,steer_motor[FR].motor_tar.set_current);
	}
	else
	{
  psdmv++;
		set_motor_current(can2_1FE,0,0,0,0);
	}
}
void steer_error_send()
{
	set_motor_current(can2_1FE,0,0,0,0);
}


void chassis_normol_send()
{
	
	if(chassis.if_chassis_open==1&&chassis.chassis_mode!=no_move)
	{
//	set_motor_current(can1_200,0,0,
//	 0,chassis_motor[FR].motor_tar.set_current );

  //chassis_motor[BR].motor_tar.set_current,chassis_motor[FR].motor_tar.set_currentchassis_motor[FL].motor_tar.set_current
	  set_motor_current(can1_200,chassis_motor[FL].motor_tar.set_current,chassis_motor[BL].motor_tar.set_current,
	  chassis_motor[BR].motor_tar.set_current,chassis_motor[FR].motor_tar.set_current);
	}
		else
	{
		set_motor_current(can1_200,0,0,0,0);
	}
		
}

void chassis_error_send()
{
	set_motor_current(can1_200,0,0,0,0);
}