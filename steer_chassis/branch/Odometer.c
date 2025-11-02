#include "Odometer.h"
#include "system.h"
#include "cmsis_os.h"
#include "bsp_dwt.h"
#include "math.h"
#define _USE_MATH_DEFINES
#include "CAN_receive.h"
#include "bsp_transmit.h"
location_t location=
{
  .Sx=0,
  .Sy=0,
  .Sx_set=0,
  .Sy_set=0,
  .init_yaw=0,
};
float real_vx;
float real_vy;
uint64_t get_time_0,get_time_1;
uint64_t get_diff_time;

void Odometer_run(void const * argument)
{
loacation_init();
  for(;;)
  {
  Odometer_calculate(&chassis);
  
  get_time_0= DWT_GetTimeline_us();
  get_diff_time=get_time_0-get_time_1;
  get_time_1= get_time_0;
   
  real_vx=location.vx*(1000000/get_diff_time);
  real_vy=location.vy*(1000000/get_diff_time);
   osDelay(1);
  }
}
//底盘里程计计算从上电位置开始坐标为(0,0),规定一个陀螺仪的正方向，将位移分解到x和y方向
void Odometer_calculate(chassis_t *mode)
{
 
 if(USART_Rx_data.chassis_mode==no_move)
 {
   location.Sx=0;
   location.Sy=0;
   location.vx=0;
   location.vy=0;
 }
 
  mode->chassis_diff_ecd[0]=chassis_motor[0].motor_measure.total_ecd-mode->chassis_last_total_ecd[0];
  mode->chassis_diff_ecd[1]=chassis_motor[1].motor_measure.total_ecd-mode->chassis_last_total_ecd[1];
  mode->chassis_diff_ecd[2]=chassis_motor[2].motor_measure.total_ecd-mode->chassis_last_total_ecd[2];
  mode->chassis_diff_ecd[3]=chassis_motor[3].motor_measure.total_ecd-mode->chassis_last_total_ecd[3];

  mode->chassis_last_total_ecd[0]=chassis_motor[0].motor_measure.total_ecd;
  mode->chassis_last_total_ecd[1]=chassis_motor[1].motor_measure.total_ecd;
  mode->chassis_last_total_ecd[2]=chassis_motor[2].motor_measure.total_ecd;
  mode->chassis_last_total_ecd[3]=chassis_motor[3].motor_measure.total_ecd;
  
  for(int i=0;i<4;i++)
  {
    location.steer_real_angle[i]=steer_motor[i].motor_measure.ecd-location.steer_init_ecd[i];
    if(location.steer_real_angle[i]<0)
    {
    location.steer_real_angle[i]+=8192;
    }
    location.steer_real_angle[i]=location.steer_real_angle[i]/8192*2*3.14;
  }
  
  location.vx=(mode->chassis_diff_ecd[0]*cos(location.steer_real_angle[0])+mode->chassis_diff_ecd[1]*cos(location.steer_real_angle[1])
              +mode->chassis_diff_ecd[2]*cos(location.steer_real_angle[2])+mode->chassis_diff_ecd[3]*cos(location.steer_real_angle[3]))*0.0000023968/4;
  location.vy=(mode->chassis_diff_ecd[0]*sin(location.steer_real_angle[0])+mode->chassis_diff_ecd[1]*sin(location.steer_real_angle[1])
              +mode->chassis_diff_ecd[2]*sin(location.steer_real_angle[2])+mode->chassis_diff_ecd[3]*sin(location.steer_real_angle[3]))*0.0000023968/4;



//编码器解算 


  USART_Rx_data.yaw_diff=USART_Rx_data.up_yaw-0;
   
   USART_Rx_data.ecd_diff=USART_Rx_data.yaw_diff*2*3.14/360;
   
   
   location.relative_init_angle=USART_Rx_data.big_yaw_ecd-USART_Rx_data.ecd_diff;
   if(location.relative_init_angle>3.1416)
   {
     location.relative_init_angle-=3.1416*2;
   }
   else if(location.relative_init_angle<-3.1416)
   {
     location.relative_init_angle+=3.1416*2;
   }
   
   location.relative_angle=forword_ecd-location.relative_init_angle;
   
   if(location.relative_angle>3.1416)
   {
     location.relative_angle-=3.1416*2;
   }
   else if(location.relative_angle<-3.1416)
   {
     location.relative_angle+=3.1416*2;
   }
   
   
  

  

  location.Sx-=(location.vx*cos(location.relative_angle)+location.vy*sin(location.relative_angle));

  location.Sy+=(-location.vx*sin(location.relative_angle)+location.vy*cos(location.relative_angle));
  

}
void loacation_init()
{ 
 location.steer_init_ecd[0]=3132;
 location.steer_init_ecd[1]=5218;
 location.steer_init_ecd[2]=5791;
 location.steer_init_ecd[3]=7662;
}