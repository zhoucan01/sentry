#include "system.h"
#include "motor.h"
#include "CAN_receive.h"
//#include "task.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "CAN_transmit.h"
#include "gimbal_big_yaw.h"
#include "remote_control.h"
#define gimbal_run
int ddddd;
int yaw_flag=1;
void motor_run()
{
	for(;;)
	{
		
		DM_start_run();
    dm_current_run();
    
		vTaskDelay(1);
	}
}


void DM_start_run()
{
 if(gimbal.if_big_yaw_can==1)
 {
  
   if(yaw_motor.Err==0)
   {
   pitch_Enable();
   
    yaw_flag=1;
   }
 }
 else if(gimbal.if_big_yaw_can==0)
 {
   
   if(yaw_motor.Err==1)
   {
   
   pitch_Unable();
   yaw_flag=0;
   }
 }
 
}


void pitch_Enable(void)
{

	for(int i=0;i<10;i++)
	{
	
   dm_motor_START(dm_yaw);
	 vTaskDelay(1);
	}
}

void 	pitch_Unable(void)
{

	dm_motor_STOP(dm_yaw);
	vTaskDelay(1);
	
}

void dm_current_run(void)
{
#ifdef gimbal_run
  dm_motor_Ctrl(dm_yaw,0,0,0,0,yaw_motor.Torque_SET);
  vTaskDelay(1);
  #else 
  dm_motor_Ctrl(dm_yaw,0,0,0,0,0);
  
  #endif
}