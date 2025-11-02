#include "small_gimbal.h"
#include "cmsis_os.h"
#include "bsp_transmit.h"
#include "cmsis_os.h"
#include "system.h"
#include "remote_control.h"
#include "bsp_transmit.h"
#include "referee.h"
#include "gimbal_big_yaw.h"
#include "can.h"
#include "can_receive.h"
com_mode_t small_gimbal_com;
small_gimbal_t small_gimbal;
void small_gimbal_run()
{
  for(;;)
  {
   get_gimbal_com();
   switch(small_gimbal_com)
   {
     
     case com_nom:
     {
      system_mode_to_small(&small_gimbal);
       break;
     }
     
     case com_err:
     {
//       sentry_system.small_gimbal_mode=small_gimbal_off;
       small_gimbal.gimbal_pitch_in=0;
       small_gimbal.gimbal_yaw_in=0;
       small_gimbal.pitch_can=0;
       break;
     }
   }
   
    vTaskDelay(2);
      
  }
}

void system_mode_to_small(small_gimbal_t *mode)
{
  switch(sentry_system.small_gimbal_mode)
  {
    case small_gimbal_pc:
    {
     mode->gimbal_pitch_in=0;
     mode->gimbal_yaw_in=0;
     mode->pitch_can=1;
     break;
    }
    case small_gimbal_rc:
    {
      mode->gimbal_pitch_in=rc_ctrl.rc.ch3;
     mode->gimbal_yaw_in=rc_ctrl.rc.ch2;
     mode->pitch_can=1;
     break;
    }
    case small_gimbal_off:
    {
      mode->gimbal_pitch_in=0;
     mode->gimbal_yaw_in=0;
     mode->pitch_can=0;
     break;
    }
  } 
}

void get_gimbal_com()
{

  if(robot_status.power_management_gimbal_output==0||sentry_system.chassis_mode==no_move||get_if_communite_broke()==1)
  {
    small_gimbal_com=com_err;
  }
  else small_gimbal_com=com_nom;

 
}


