#include "sentry_chassis.h"
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
#include "arm_math.h"
#include "can.h"
#include "bsp_dwt.h"
#include "motor.h"
#include "SuperCAP.h"
pid_struct_t pid_steer_ecd[4];
pid_struct_t pid_steer_speed[4];
pid_struct_t pid_chassis_speed[4];
pid_struct_t pid_chaais_Sx;
pid_struct_t pid_chaais_Sy;
pid_struct_t chassis_buffer;
pid_struct_t cap_buff;
#define steer
#define chassis_open
//pid_struct_t pid_chassis_follow;
/**两种模式 1 无速度输入后底盘舵电机抱圆锁死 2 无速度输入后底盘电机为有速度输入的最后一秒的角度**/
#define communicate 
//#define steer_lock


//uint64_t get_time_0,get_time_1;
//uint64_t get_diff_time;
void sentry_chassis_run()
{
  
  chassis_pid_init();
  for(;;)
  { 
  
    
    speed_in_reslove_1(&chassis);//速度坐标系转换解算
    chassis_speed_get(&chassis);//将三轴速度转换成两轴速度便于解算出舵机的角度
    steer_angle_get(&chassis);//根据vx,vy解算出舵机的角度并加上劣弧转小圈算法配合电机正反转
    chassis_speed_set(&chassis);//将vx,vy合成算出轮子真实速度
    chassis_clac(&chassis);//各参数pid计算
    
    
//   get_time_0= DWT_GetTimeline_us();
    chassis_power_limit_set();
    
//    get_diff_time=get_time_0-get_time_1;
//   get_time_1= get_time_0;
   
   
  chassis_power_get();
  
  

  
  
  		#ifdef steer
		steer_normol_send();
		
		#else 
		steer_error_send();
		
		#endif
		
		#ifdef chassis_open
		chassis_normol_send();
		#else 
		chassis_error_send();
		#endif
    
    
    osDelay(1);
  }
}



float steer_ecdkp=1.4;
float steer_speed_kp=15;
void chassis_pid_init()
{


for(int i=0;i<4;i++)
	{
		
//		pid_init(&pid_steer_ecd[i],steer_ecdkp,0,0.25,0,600);
//		pid_init (&pid_steer_speed[i],steer_speed_kp,0,0,0,16384);
//		pid_init(&pid_chassis_speed[i],5,0,0,0,16000);
    pid_init(&pid_chassis_speed[i],5,0.1,0,2500,16000);

//	pid_init(&pid_chassis_follow,10,0,0,0,1000);
	}
      pid_init(&pid_chassis_speed[2],5.0,0.1,0,4000,16000);
    pid_init(&pid_chassis_speed[1],5,0.1,0,2500,16000);
  
//  pid_init(&pid_chassis_speed[2],6,0,0,0,16000);
//  pid_init(&pid_chassis_speed[3],6,0,0,0,16000);  
  
  pid_init(&pid_steer_ecd[0],0.5,0,0.0,0,400);
	pid_init (&pid_steer_speed[0],55,0,0,0,16384);
    
  
  pid_init(&pid_steer_ecd[1],0.5,0,0,0,500);
  pid_init (&pid_steer_speed[1],50,0,0,0,16384);
 
  pid_init(&pid_steer_ecd[2],0.5,0,0.0,0,400);
  pid_init (&pid_steer_speed[2],50,0,0,0,16384);
  
  pid_init(&pid_steer_ecd[3],0.5,0,0.0,0,400);
  pid_init (&pid_steer_speed[3],50,0,0,0,16384);
  
    pid_init(&chassis_buffer,0.5,0,0,0,30);
  pid_init(&pid_chaais_Sx,10.0,0,0,0,1500);
  pid_init(&pid_chaais_Sy,10,0,0,0,1500);
  
  pid_init(&cap_buff,0.5,0.001,0,5,36./0);
//  pid_init(&pid_chassis_follow,5,0,0,0,200);
}
 



/**
* @note 云台速度转换到底盘坐标系
*
*
*/
void speed_in_reslove(chassis_t *mode)
{

//if(rc_ctrl.rc.s_r==2)
//{
#ifndef communicate
	if(abs(rc_ctrl.rc.ch3)<200)
	{
		mode->speed_in.vx=0;
	}
	else mode->speed_in.vx=rc_ctrl.rc.ch3*3.5;
	
	if(abs(rc_ctrl.rc.ch2)<200)
	{
		mode->speed_in.vy=0;
	}
	else  mode->speed_in.vy=-rc_ctrl.rc.ch2*3.5;
	
	if(rc_ctrl.rc.s_l==RC_SW_UP)
	{
		mode->speed_in.wz=3500;
	}
	else 
	{
		mode->speed_in.wz=0;
	}
	if(rc_ctrl.rc.s_l!=RC_SW_DOWN)
	{
		mode->if_chassis_open=1;
	}
		else
	{
		mode->if_chassis_open=0;
	}
  //	mode->init_yaw=
	#else 
	

mode->speed_in.vx=mode->speed_in.vx;
mode->speed_in.vy=mode->speed_in.vy;
mode->speed_in.wz=mode->speed_in.wz;
	#endif
  
  if(mode->speed_in.vx==0&&mode->speed_in.wz==0&&mode->speed_in.vy==0)
  {
      mode->speed_in.vx=1;
      mode->speed_reslove.vy=-(mode->speed_in.vx*sin(mode->diff_angle))+
    (mode->speed_in.vy)*cos(mode->diff_angle);
    mode->speed_reslove.vx=(mode->speed_in.vx)*cos(mode->diff_angle)+
    (mode->speed_in.vy)*sin(mode->diff_angle);
    mode->speed_reslove.wz=mode->speed_in.wz;
    mode->speed_in.vx=0;
  }
  else 
  {
    	mode->speed_reslove.vy=-(mode->speed_in.vx*sin(mode->diff_angle))+
	(mode->speed_in.vy)*cos(mode->diff_angle);
	mode->speed_reslove.vx=(mode->speed_in.vx)*cos(mode->diff_angle)+
	(mode->speed_in.vy)*sin(mode->diff_angle);
	mode->speed_reslove.wz=mode->speed_in.wz;
  }

	
}

float get_speed_vx;
float get_speed_vy;
void speed_in_reslove_1(chassis_t *mode)
{
 
//    if(mode->move_flag==1)
//   {
//     mode->speed_in.vx*=1;
//     mode->speed_in.vy*=1;
//     
//   }
//   else if(mode->move_flag==0)
//   {
//     mode->speed_in.vx*=0.0001;
//     mode->speed_in.vy*=0.0001;
//   }
   
   
   /*根据底盘速度来判断底盘是否停下来*/
 move_state_change(mode);
 
   if(mode->move_flag==0)
 {
   mode->speed_in.vx=0;
   mode->speed_in.vy=0;
   mode->speed_in.wz=0;
 }
 switch(mode->chassis_mode)
 {
 case no_move:
   {
    
    /*按理来说no_move模式下不用赋速度，但是此刻舵向仍然需要得到角度，此刻将vx赋为1，舵向的目标值会输出到朝着底盘正方向*/
    mode->speed_in.vx=1;
      mode->speed_reslove.vy=-(mode->speed_in.vx*sin(mode->diff_angle))+
    (mode->speed_in.vy)*cos(mode->diff_angle);
    mode->speed_reslove.vx=(mode->speed_in.vx)*cos(mode->diff_angle)+
    (mode->speed_in.vy)*sin(mode->diff_angle);
    mode->speed_reslove.wz=mode->speed_in.wz;
    mode->speed_in.vx=0;
    
    break;
   }

  
   case normol_move:
   {
       if(mode->speed_in.vx==0&&mode->speed_in.wz==0&&mode->speed_in.vy==0)
    {
      
      
      mode->speed_in.vx=1;
      mode->speed_reslove.vy=-(mode->speed_in.vx*sin(mode->diff_angle))+
    (mode->speed_in.vy)*cos(mode->diff_angle);
    mode->speed_reslove.vx=(mode->speed_in.vx)*cos(mode->diff_angle)+
    (mode->speed_in.vy)*sin(mode->diff_angle);
    mode->speed_reslove.wz=mode->speed_in.wz;
    mode->speed_in.vx=0;
    }
    else 
    {
        mode->speed_reslove.vy=-(mode->speed_in.vx*sin(mode->diff_angle))+
    (mode->speed_in.vy)*cos(mode->diff_angle);
    mode->speed_reslove.vx=(mode->speed_in.vx)*cos(mode->diff_angle)+
    (mode->speed_in.vy)*sin(mode->diff_angle);
    mode->speed_reslove.wz=mode->speed_in.wz;
    }
    break;
   }
   
  
   
   default:
   {
     mode->speed_reslove.vy=-(mode->speed_in.vx*sin(mode->diff_angle))+
    (mode->speed_in.vy)*cos(mode->diff_angle);
    mode->speed_reslove.vx=(mode->speed_in.vx)*cos(mode->diff_angle)+
    (mode->speed_in.vy)*sin(mode->diff_angle);
    mode->speed_reslove.wz=mode->speed_in.wz;
    break;
   }
 }

get_speed_vx=mode->speed_reslove.vx;
get_speed_vy=mode->speed_reslove.vy;


}


float limit_addspeed(float speed_set,float speed_ref,float addspeed_limit)
{
	if(fabs(speed_set-speed_ref)>addspeed_limit)
	{
		if(speed_set>speed_ref)
		{
			speed_set=speed_ref+addspeed_limit;
		}
		else if(speed_set<speed_ref)
		{
			speed_set=speed_ref-addspeed_limit;
		}
	}
	return speed_set;
}
int ffff;
float limit_addspeed1(float speed_set,float speed_ref,float addspeed_limit)
{
	if(fabs(speed_set-speed_ref)>addspeed_limit)
	{
  
  ffff++;
		if(speed_set>speed_ref)
		{
			speed_set=speed_ref+addspeed_limit;
		}
		else if(speed_set<speed_ref)
		{
			speed_set=speed_ref-addspeed_limit;
		}
	}
	return speed_set;
}


/**
*@note 底盘逆运动学解算，计算出底盘坐标系下的vx,vy
*
*
*
*/
int hghgh;

void chassis_speed_get(chassis_t *mode)
{

	
  
  
     mode->chassis[FR].vx=mode->speed_reslove.vx
	                     -(mode->speed_reslove.wz+0)*sin(3.1415/4);
	mode->chassis[FR].vy=mode->speed_reslove.vy
	                     +(mode->speed_reslove.wz+0)*sin(3.1415/4);
	
	mode->chassis[BR].vx=mode->speed_reslove.vx
	                     -(mode->speed_reslove.wz+0*mode->wz_back)*sin(3.1415/4);
	mode->chassis[BR].vy=mode->speed_reslove.vy
	                     -(mode->speed_reslove.wz+0*mode->wz_back)*sin(3.1415/4);
	
	mode->chassis[BL].vx=mode->speed_reslove.vx
	                     +(mode->speed_reslove.wz+0*mode->wz_back)*sin(3.1415/4);
	mode->chassis[BL].vy=mode->speed_reslove.vy
	                     -(mode->speed_reslove.wz+0*mode->wz_back)*sin(3.1415/4);
	
	mode->chassis[FL].vx=mode->speed_reslove.vx
	                     +mode->speed_reslove.wz*sin(3.1415/4);
	mode->chassis[FL].vy=mode->speed_reslove.vy
	                     +mode->speed_reslove.wz*sin(3.1415/4);	
                       
                       
//  get_speed_vx = mode->chassis[FR].vx;
//  
//  get_speed_vy = mode->chassis[FR].vy;
                       
	mode->speed_set[FR]=sqrt(pow(mode->chassis[FR].vx,2)+
	pow(mode->chassis[FR].vy,2));
	
	mode->speed_set[BR]=sqrt(pow(mode->chassis[BR].vx,2)+
	pow(mode->chassis[BR].vy,2));
	
	mode->speed_set[BL]=sqrt(pow(mode->chassis[BL].vx,2)+
	pow(mode->chassis[BL].vy,2));
	
	mode->speed_set[FL]=sqrt(pow(mode->chassis[FL].vx,2)+
	pow(mode->chassis[FL].vy,2));
	
  
  
  /*在轮子跟随云台模式解算时在上面赋了速度才能计算出跟随角度，这里将速度还原*/
  if(mode->chassis_mode!=no_move&& mode->speed_in.vx==0&&mode->speed_in.vy==0&&mode->speed_in.wz==0)
  {
   for(int i=0;i<4;i++)
   {
     mode->speed_set[i]=0;
   }
  }
}


/**
* @note用速度解算出舵机角度并转换成编码器值,并加上转劣弧算法，将速度也随着算法赋正负
*/
int gggg;
int tttt;
float steer_fr_ecd;
void steer_angle_get(chassis_t *mode)
{
	
	    if(atan2(mode->chassis[FR].vy,mode->chassis[FR].vx)<0)
			{
				mode->steer_angle[FR]=atan2(mode->chassis[FR].vy,mode->chassis[FR].vx)+2*PI;
			}
			else {mode->steer_angle[FR]=atan2(mode->chassis[FR].vy,mode->chassis[FR].vx);}
			
			if(atan2(mode->chassis[BR].vy,mode->chassis[BR].vx)<0)
			{
				mode->steer_angle[BR]=atan2(mode->chassis[BR].vy,mode->chassis[BR].vx)+2*PI;
			}
			else {mode->steer_angle[BR]=atan2(mode->chassis[BR].vy,mode->chassis[BR].vx);}
			
			if(atan2(mode->chassis[BL].vy,mode->chassis[BL].vx)<0)
			{
				mode->steer_angle[BL]=atan2(mode->chassis[BL].vy,mode->chassis[BL].vx)+2*PI;
			}
			else {mode->steer_angle[BL]=atan2(mode->chassis[BL].vy,mode->chassis[BL].vx);}
			
			if(atan2(mode->chassis[FL].vy,mode->chassis[FL].vx)<0)
			{
				mode->steer_angle[FL]=atan2(mode->chassis[FL].vy,mode->chassis[FL].vx)+2*PI;
			}
			else {mode->steer_angle[FL]=atan2(mode->chassis[FL].vy,mode->chassis[FL].vx);}
	
	
    mode->steer_resolve_ecd[FR]=mode->steer_angle[FR]/(2*PI)*8192+steer_init_angle_FR;
		mode->steer_resolve_ecd[BR]=mode->steer_angle[BR]/(2*PI)*8192+steer_init_angle_BR;
		mode->steer_resolve_ecd[BL]=mode->steer_angle[BL]/(2*PI)*8192+steer_init_angle_BL;
		mode->steer_resolve_ecd[FL]=mode->steer_angle[FL]/(2*PI)*8192+steer_init_angle_FL;

    for(int i=0;i<4;i++)
		{
				if(mode->steer_resolve_ecd[i]>8192)
						mode->steer_resolve_ecd[i]-=8192;
					else if(mode->steer_resolve_ecd[i]<-8192)
						mode->steer_resolve_ecd[i]+=8192;
					
					if((mode->steer_resolve_ecd[i]-steer_motor[i].motor_measure.ecd)>4096)
					{
						mode->steer_resolve_ecd[i]-=8192;
					}
					else if(mode->steer_resolve_ecd[i]-steer_motor[i].motor_measure.ecd<-4096)
					{
						mode->steer_resolve_ecd[i]+=8192;
					}
					
		
   
			if(mode->steer_resolve_ecd[i]-steer_motor[i].motor_measure.ecd>2048)
					{
					  mode->speed_direct[i]=-1;
						mode->steer_ecd[i]=mode->steer_resolve_ecd[i]-4096;
					}
					else if(mode->steer_resolve_ecd[i]-steer_motor[i].motor_measure.ecd<-2048)
					{mode->speed_direct[i]=-1;

						mode->steer_ecd[i]=mode->steer_resolve_ecd[i]+4096;
					}
					else 
          {
            mode->speed_direct[i]=1;
            mode->steer_ecd[i]=mode->steer_resolve_ecd[i];
          }	
			}
		
		
		

      if(mode->chassis_mode==no_move)
   {
     mode->last_ecd[FR]=steer_motor[FR].motor_measure.ecd;
		 mode->last_ecd[BR]=steer_motor[BR].motor_measure.ecd;
		 mode->last_ecd[BL]=steer_motor[BL].motor_measure.ecd;
		 mode->last_ecd[FL]=steer_motor[FL].motor_measure.ecd;
   }


   /*除陀螺时对舵向转向进行衰减，防止瞬时功率变化过大对超电进行冲击*/
    if(chassis.speed_in.wz!=0)
    {
      for(int i=0;i<4;i++)
      {
      
//        if(mode->last_ecd[i]>500&&mode->last_ecd[i]<7500&&mode->steer_ecd[i]>500&&mode->steer_ecd[i]<7500)
//        {
//          mode->steer_ecd[i]=LowPass_SetSteer(mode->last_ecd[i],mode->steer_ecd[i]);
//        }
//        else {
          mode->steer_ecd[i]=mode->steer_ecd[i];
//        }
//        
      }
     }
     else 
     {
     
         for(int i=0;i<4;i++)
        {
          mode->steer_ecd[i]=mode->steer_ecd[i];
        }
     }
     
     
   
  
   if(mode->move_flag==0)
   {
     for(int i=0;i<4;i++)
     {
      mode->steer_final_ecd[i]=mode->last_ecd[i];
     }
   }else if(mode->move_flag==1)
   {
     for(int i=0;i<4;i++)
     {
      mode->steer_final_ecd[i]=mode->steer_ecd[i];
     }
   }
	
	  mode->last_ecd[FR]=mode->steer_final_ecd[FR];
		mode->last_ecd[BR]=mode->steer_final_ecd[BR];
		mode->last_ecd[BL]=mode->steer_final_ecd[BL];
		mode->last_ecd[FL]=mode->steer_final_ecd[FL];
    
    
}

void chassis_speed_set(chassis_t *mode)
{

  mode->speed_set[FR]=sqrt(pow(mode->chassis[FR].vx,2)+
	pow(mode->chassis[FR].vy,2));
	
	mode->speed_set[BR]=sqrt(pow(mode->chassis[BR].vx,2)+
	pow(mode->chassis[BR].vy,2));
	
	mode->speed_set[BL]=sqrt(pow(mode->chassis[BL].vx,2)+
	pow(mode->chassis[BL].vy,2));
	
	mode->speed_set[FL]=sqrt(pow(mode->chassis[FL].vx,2)+
	pow(mode->chassis[FL].vy,2));
  
	/*在轮子跟随云台模式解算时在上面赋了速度才能计算出跟随角度，这里将速度还原*/
  if((mode->chassis_mode!=no_move&& mode->speed_in.vx==0&&mode->speed_in.vy==0&&mode->speed_in.wz==0)||mode->chassis_mode==no_move)
  {
   for(int i=0;i<4;i++)
   {
     mode->speed_set[i]=0;
   }
  }
  
  
  
  float k=0;
  
  
  for(int i=0;i<4;i++)
  {
    mode->speed_set[i]*=mode->speed_direct[i];
    
  }
  
  
  
  /*除陀螺时检测舵向是否转到目标位置，用cos函数进行衰减*/
  if(mode->speed_in.wz==0)
  {
      for(int i=0;i<4;i++)
    {
      mode->steer_diff_angle=fabs(mode->steer_ecd[i]-steer_motor[i].motor_measure.ecd)*2*3.14/8192;
      k=arm_cos_f32(mode->steer_diff_angle);
      mode->speed_set[i]*=k*k*k;
    }
  }
  
//  for(int i=0;i<4;i++)
//  {
//     mode->speed_set[i]=LowPass_SetChassis(mode->last_speed_set[i],mode->speed_set[i]);
//     mode->last_speed_set[i]=mode->speed_set[i];
//  }
}







uint8_t cap_state = 0;
float input_power = 0;		 // input power from battery (referee system)
float chassis_max_power_t;
//uint8_t cap_state = 0;
//float input_power = 0;		 // input power from battery (referee system)
/**
 * @description: 有超电的底盘功率限制
 * @brief:对于底盘的电机功率使用进行控制，以更好的控制车体功率
 * @return none
Pm=CTIcmdω+k1ω2+k2Icmd2
 */
 float get_power;
 float remain_power;
void chassis_power_limit_set(void)
{
	uint16_t max_power_limit = 40;//最大功率初始化
	fp32 chassis_max_power = 0;
  fp32 chassis_steer_powe=0;//舵轮的功率
  fp32 chassis_remain_power=0;//扣去舵轮的功率,剩余的轮向功率
	
  float initial_steer_power[4];
	float initial_give_power[4]; // initial power from PID calculation
  float initial_total_steer_power = 0;
	float initial_total_power = 0;
	fp32 scaled_give_power[4];
  int steer_power;
	fp32 chassis_power = 0.0f; 
	fp32 chassis_power_buffer = 0.0f;

//	fp32 toque_coefficient = 1.99688994e-6f; // (20/16384)*(0.3)*(187/3591)/9.55
//	fp32 a = 1.23e-07;						 // k1
//	fp32 k2 = 1.453e-07;					 // k2
//	fp32 constant = 4.081f;
	

chassis_power=100.0f;
chassis_power_buffer=USART_Rx_data.chassis_buff;//缓冲功率


   max_power_limit = USART_Rx_data.chassis_Power_limit;         //哨兵为100,其他车体根据裁判系统功率填入数据
  
	
	input_power = max_power_limit- pid_calc(&chassis_buffer ,chassis_power_buffer ,30); ;//计算出的此时可用的功率

	//
	
  
//  SuperCAP.C_Vol=0;//超电的电压，没有超电就填0，有超电注掉这一行
  
  
  //根据自己代码什么时候用超电什么时候cap_state为1,
  
	if(SuperCAP.C_Vol >= 11.5f)
	{
    if(USART_Rx_data.chassis_Power_limit<40)
    {
     chassis_max_power = input_power + 20;
    }
    
    else chassis_max_power = input_power + 60;

			

	}
	else
	{
		chassis_max_power = input_power;
	}
  
  
  chassis_max_power_t=chassis_max_power;
  for(int i = 0;i < 4;i++)
  {
    initial_steer_power[i]= motor_6020_kp*steer_motor[i].motor_measure.speed_rpm*steer_motor[i].motor_tar.set_current
                            +motor_6020_kw*steer_motor[i].motor_measure.speed_rpm*steer_motor[i].motor_measure.speed_rpm
                            +motor_6020_ki*steer_motor[i].motor_tar.set_current*steer_motor[i].motor_tar.set_current+motor_6020_constant;
                            
     if(initial_steer_power<0)
     {
       continue;
     }
     
     initial_total_steer_power+=initial_steer_power[i];
  }
  
  if(chassis.speed_in.wz!=0)
  {
    steer_power=chassis_max_power*0.8;
  }
  else steer_power=chassis_max_power*0.8 ;
  get_power=initial_total_steer_power;
  if (initial_total_steer_power > steer_power) // 判断是否超过最大舵向功率
		{
    
    
			fp32 power_scale = steer_power / initial_total_steer_power;
      initial_total_steer_power=steer_power;
			for (uint8_t i = 0; i < 4; i++)
			{
				scaled_give_power[i] = initial_steer_power[i] * power_scale; // get scaled power
				if (scaled_give_power[i] < 0)
				{
					continue;
				}	

				fp32 b = motor_6020_kp * steer_motor[i].motor_measure.speed_rpm;
				fp32 c = motor_6020_kw * steer_motor[i].motor_measure.speed_rpm * steer_motor[i].motor_measure.speed_rpm - scaled_give_power[i] + motor_6020_constant;

				if (steer_motor[i].motor_tar.set_current > 0) // Selection of the calculation formula according to the direction of the original moment
				{
					fp32 temp = (-b + sqrt(b * b - 4 * motor_6020_ki * c)) / (2 * motor_6020_ki);
					if (temp > 16000)
					{
						steer_motor[i].motor_tar.set_current = 16000;//max_current_out 16384
					}
					else
						steer_motor[i].motor_tar.set_current = temp;
				}
				else
				{
					fp32 temp = (-b - sqrt(b * b - 4 * motor_6020_ki * c)) / (2 * motor_6020_ki);
					if (temp < -16000)
					{
						steer_motor[i].motor_tar.set_current = -16000;
					}
					else
						steer_motor[i].motor_tar.set_current = temp;
				}
			}
		}
  
  
  
  //因为6020预测准确率没有3508好，加上只用第一个电机的数据拟合其他的参数，肯定会不准，在这里预留5w来做抵消误差
  chassis_remain_power=chassis_max_power-initial_total_steer_power-3;
  remain_power=chassis_remain_power;
  if(chassis_remain_power<0)
  {
    chassis_remain_power=0;
  }
  
	for(int i = 0;i < 4;i++)
	{
  
		initial_give_power[i] = chassis_motor[i].motor_tar.set_current* motor_3508_kp * chassis_motor[i].motor_measure.speed_rpm +
										motor_3508_kw * chassis_motor[i].motor_measure.speed_rpm * chassis_motor[i].motor_measure.speed_rpm +
										motor_3508_ki * chassis_motor[i].motor_tar.set_current * chassis_motor[i].motor_tar.set_current + motor_3508_constant;
		
	  if (initial_give_power < 0) // negative power not included (transitory)
		{  
		  continue;
		} 
		initial_total_power += initial_give_power[i];
	}
	
		if (initial_total_power > chassis_remain_power) // 判断是否超过最大轮向功率
		{
			fp32 power_scale = chassis_remain_power / initial_total_power;
			for (uint8_t i = 0; i < 4; i++)
			{
				scaled_give_power[i] = initial_give_power[i] * power_scale; // get scaled power
				if (scaled_give_power[i] < 0)
				{
					continue;
				}	

				fp32 b = motor_3508_kp * chassis_motor[i].motor_measure.speed_rpm;
				fp32 c = motor_3508_kw * chassis_motor[i].motor_measure.speed_rpm * chassis_motor[i].motor_measure.speed_rpm - scaled_give_power[i] + motor_3508_constant;

				if (chassis_motor[i].motor_tar.set_current > 0) // Selection of the calculation formula according to the direction of the original moment
				{
					fp32 temp = (-b + sqrt(b * b - 4 * motor_3508_ki * c)) / (2 * motor_3508_ki);
					if (temp > 16000)
					{
						chassis_motor[i].motor_tar.set_current = 16000;//max_current_out 16384
					}
					else
						chassis_motor[i].motor_tar.set_current = temp;
				}
				else
				{
					fp32 temp = (-b - sqrt(b * b - 4 * motor_3508_ki * c)) / (2 * motor_3508_ki);
					if (temp < -16000)
					{
						chassis_motor[i].motor_tar.set_current = -16000;
					}
					else
						chassis_motor[i].motor_tar.set_current = temp;
				}
			}
		}
}


float real_chassis=0.0f;

void chassis_power_get()
{
  float initial_total_steer_power = 0;
	float initial_total_power = 0;
  float initial_steer_power[4];
	float initial_give_power[4]; // initial power from PID calculation
  
  
  for(int i = 0;i < 4;i++)
  {
    initial_steer_power[i]= motor_6020_kp*steer_motor[i].motor_measure.speed_rpm*steer_motor[i].motor_tar.set_current
                            +motor_6020_kw*steer_motor[i].motor_measure.speed_rpm*steer_motor[i].motor_measure.speed_rpm
                            +motor_6020_ki*steer_motor[i].motor_tar.set_current*steer_motor[i].motor_tar.set_current+motor_6020_constant;
                            
     if(initial_steer_power<0)
     {
       continue;
     }
     
     initial_total_steer_power+=initial_steer_power[i];
  }
  
  for(int i = 0;i < 4;i++)
	{
  
		initial_give_power[i] = chassis_motor[i].motor_tar.set_current* motor_3508_kp * chassis_motor[i].motor_measure.speed_rpm +
										motor_3508_kw * chassis_motor[i].motor_measure.speed_rpm * chassis_motor[i].motor_measure.speed_rpm +
										motor_3508_ki * chassis_motor[i].motor_tar.set_current * chassis_motor[i].motor_tar.set_current + motor_3508_constant;
		
	  if (initial_give_power < 0) // negative power not included (transitory)
		{  
		  continue;
		} 
		initial_total_power += initial_give_power[i];
	}
  
  
  real_chassis=initial_total_steer_power+initial_total_power;
  
  if(real_chassis<0)
  {
   real_chassis=0;
  }
}





/**
  * @brief  车子速度设定的低通滤波
  * @param  
  * @retval 滤波后最新值
  */
float LowPass_SetChassis(float old,float In)
{
    return (1-K_Low_setchassis)*(old)+K_Low_setchassis*In;   
}

/**
  * @brief  车子舵轮设定的低通滤波
  * @param  
  * @retval 滤波后最新值
  */
float LowPass_SetSteer(float old,float In)
{
    return (1-K_Low_setSteer)*(old)+K_Low_setSteer*In;   
}

void chassis_clac(chassis_t *mode)
{
 int n=3;
	for(int i=0;i<4;i++)
	{
		mode->steer_speed_set[i]=pid_calc(&pid_steer_ecd[i],steer_motor[i].motor_measure.ecd,mode->steer_final_ecd[i]);
		steer_motor[i].motor_tar.set_current= pid_calc(&pid_steer_speed[i],steer_motor[i].motor_measure.speed_rpm,mode->steer_speed_set[i]);
		chassis_motor[i].motor_tar.set_current=pid_calc(&pid_chassis_speed[i],chassis_motor[i].motor_measure.speed_rpm,mode->speed_set[i]);
	}
}








int real_move_state=0;

/*检测整个底盘速度，若有速度反馈则代表底盘正在移动,  */
int chassis_real_move_get()
{


  
  uint16_t real_speed=abs(chassis_motor[0].motor_measure.speed_rpm)+abs(chassis_motor[1].motor_measure.speed_rpm)+
           abs(chassis_motor[2].motor_measure.speed_rpm)+abs(chassis_motor[3].motor_measure.speed_rpm);
           
  real_speed/=4;
           
           if(real_speed<150)
           {
             real_move_state=move_off;
           }
           else real_move_state=move_on;

   return real_move_state;
}





/*若模式切换后底盘未赋速度则当运动完全停止后再转舵，特别是底盘陀螺转向舵向跟随时底盘的转动的惯性全作用在轮子上，这也稍微避免了翘头的问题*/
int yyyy;
int last_state=no_move;
//void move_state_change(chassis_t *mode)
//{


//   if(mode->chassis_mode!=last_state&&mode->move_flag==1)
//   {
//   
//   yyyy++;
//     mode->move_flag=0;
//   }
//   if((chassis_real_move_get()==move_off||fabs(mode->speed_in.vx)>0||fabs(mode->speed_in.vy)>0)&&mode->move_flag==0)
//   {
//     mode->move_flag=1;
//   }
//   last_state=mode->chassis_mode;
//}

void move_state_change(chassis_t *mode)
{
   if(mode->speed_in.vx==0&&mode->speed_in.vy==0&&fabs(mode->speed_in.wz)<500&&mode->move_flag==1)
   {
     mode->move_flag=1;
   }
   if((chassis_real_move_get()==move_off||(mode->speed_in.vx!=0||mode->speed_in.vy!=0))&&mode->move_flag==0)
   {
     mode->move_flag=1;
   }
}
