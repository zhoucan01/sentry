#include "trig.h"
#include "shoot.h"
#include "system.h"
#include "CAN_receive.h"
#include "ins_task.h"
#include "cmsis_os.h"
#include "struct_typedef.h"
#include "stm32f4xx_hal.h"
#include "remote_control.h"
#include "Nautilus_Vision.h"
#include "gimbal.h"
#include "motor.h"
#include "can.h"
//#define vision_sin
com_mode_t trig_com;
trig_t trig=
{
	.fire_state=fire_no,
	.last_state=fire_no,
	.if_block=0,
	.block_state.block_type=no_block,
	.block_state.if_fire_block=0,
//	.block_state.if_block_over=1,
	.block_state.cnt=0,
	
	
	.trig_flag.if_sin_over=1,
	.trig_flag.if_con_over=1,
	.trig_flag.if_sin_request=0,
	.trig_flag.if_con_request=0,
//	.trig_flag.if_sin_block=0,
//	.trig_flag.if_con_block=0,
	.trig_flag.if_block_react=1,
	.trig_flag.if_block_react_over=1,
	.trig_freq=0,
};

pid_struct_t  pid_trig_sin_ecd;
pid_struct_t  pid_trig_sin_speed;
pid_struct_t  pid_trig_con;

float last_trig_set=0.0f;
int get_trig_cnt;

void trig_pid_init()
{
	pid_init(&pid_trig_sin_ecd,1.0,0,0.1,10,4000);
	pid_init(&pid_trig_sin_speed,8,0.0005,0.0,100,10000);
	pid_init(&pid_trig_con,25,0,0,0,10000);
}

int trig_ecd_set=0.00f;
float trig_speed_set=0.0f;
int fgfgfg;

void trig_run(void const * argument)
{

vTaskDelay(2);
	trig_pid_init();
	for(;;)
	{
    get_trig_com();
		trig_task_run(&trig);
//small_to_big(&hcan1,0x402);
		
		vTaskDelay(1);	
	}
}

int mmmmmmm;
void receive_wheel_state()
{
  Wheel_State=control_data.other_data.wheel_state;
}
void trig_task_run(trig_t *mode)
{

receive_wheel_state();
//	WHEEL_STATE_Ctrl();
	trig_mode_chose(mode);     //获取控制信息
	get_trig_motor_state(mode);//获取电机是否卡弹
	tirg_control(mode);
	trig_motor_run(mode);
	
}

int pklkppl;


void trig_mode_chose(trig_t *mode)
{

 if(trig_com==com_nom)
 {
    if(control_data.gimbal_mode==small_gimbal_rc)
    {
      shoot_rc_mode(mode);
    }
    else if(control_data.gimbal_mode==small_gimbal_pc)
    {
      shoot_pc_mode(mode);
    }
	}
  else 
  {
  pklkppl++;
////  shoot_rc_mode(mode);
    mode->fire_state=fire_no;
    trig_ecd_set=trig_motor.motor_measure.total_ecd;
    trig_motor.motor_tar.set_current=0;
  }
  
	
	
}
 
void get_trig_com()
{
//&&Report_IF_Fric3508_SetSpeed()==1

  if(control_data.shoot_mode!=shoot_no&&gimbal_system.control_com==com_nom)
	{
    trig_com=com_nom;
  }
  else trig_com=com_err;
  
}
int lplpl;
void shoot_rc_mode(trig_t *mode)
{
  switch(Wheel_State)
      {
        case ZERO_rc:
        {
          
          mode->fire_state=fire_no;
          mode->trig_flag.if_sin_request=1;
          break;
          
        }
        
        case DOWN_SHORT_rc:
        {

          mode->fire_state=fire_sin;
          break;
        }
        
        case DOWN_LONG_rc:
        {
        lplpl++;
          if(mode->last_state==fire_sin)
          {
                if(trig_motor.motor_measure.total_ecd-(float)trig_ecd_set>-1400)
              {
              mode->fire_state=fire_con;
              }
              else mode->fire_state=fire_sin;
          }
          else mode->fire_state=fire_con;
          break;
        }
      }
      
//      if(Report_IF_Fric3508_SetSpeed()==1)
//      {
//        mode->fire_state=mode->fire_state;
//      }
//      else mode->fire_state=fire_no;
}

void shoot_pc_mode(trig_t *mode)
{

  if(gimbal.vision_on==1)
  {
  
  #ifdef vision_sin
    mode->fire_state=fire_vision;
    #else 
   mode->fire_state=fire_con;
    #endif
  }
  else 
  {
      switch(Wheel_State)
      {
        case ZERO_rc:
        {
          
          mode->fire_state=fire_no;
          mode->trig_flag.if_sin_request=1;
          break;
          
        }
        
        case DOWN_SHORT_rc:
        {
//          mmmmmmm++;
          mode->fire_state=fire_sin;
          break;
        }
        
        case DOWN_LONG_rc:
        {
          if(mode->last_state==fire_sin)
          {
                if(trig_motor.motor_measure.total_ecd-(float)trig_ecd_set>-1400)
              {
              mode->fire_state=fire_con;
              mmmmmmm++;
              }
              else mode->fire_state=fire_sin;
          }
          else mode->fire_state=fire_con;
          break;
        }
      }
    }
}






//fire_state_t last_fire_state;
int kkkkk;
void tirg_control(trig_t *mode)
{//&&Report_IF_Fric3508_SetSpeed()==YES
	if(control_data.shoot_mode==shoot_on)
	{
	    trig_chose_freq(mode);//根据热量选择射频
			if(mode->trig_flag.if_block_react_over==0)
			{
	   			block_react(mode);
			}
			
			
			else
			{
					switch(mode->fire_state)
					{
						case fire_sin:
						{
							if(mode->last_state==fire_con)
							{
								trig_ecd_set=trig_motor.motor_measure.total_ecd;//更新目标值为当前编码器值
							}
							trig_sin(mode);
						break;
						}
						
						case fire_con:
						{
							trig_ecd_set=trig_motor.motor_measure.total_ecd;//更新目标值为当前编码器值
						mmmmmmm++;
							trig_con(mode);
							break;
						}
						
						case fire_no:
						{
//							mode->trig_flag.if_sin_request=1;
							if(judge_if_sin_over()==YES)
							{
								mode->trig_flag.if_sin_request=1;
							trig_ecd_set=trig_motor.motor_measure.total_ecd;
							}
							
//							trig_ecd_set=trig_motor.motor_measure.total_ecd;
							
							break;
						}
            
            
            case fire_vision:
            {
              trig_vision(mode);
              break;
            }
				}
						
					mode->last_state=mode->fire_state;
			}
	}
	else 
	{
		trig_ecd_set=trig_motor.motor_measure.total_ecd;
	}
}



//int8_t trig_motor_power_limit;
void trig_sin(trig_t *mode)
{
	if(mode->trig_flag.if_sin_request==1)
	{
      kkkkk++;
      if(control_data.shoot_power>=
      trig_max_power-trig_normol_power_limit)
    {
      trig_ecd_set=trig_motor.motor_measure.total_ecd;
    }
      else trig_ecd_set+=trig_sin_ecd;
      mode->trig_flag.if_sin_request=0;
	}
}

void trig_con(trig_t *mode)
{
  
  

  if(gimbal.vision_on==0)
  {
    trig_speed_set=trig_1fps_speed*mode->trig_freq;
  }
  else if(gimbal.vision_on==1)
  {
    if(Rx_Vision.fire&&Rx_Vision.Fire_Mode>0)
    {
    
        if(mode->trig_freq==0)
        {
          mode->trig_freq=0;
        }
        else 
        {
          
//          if(Rx_Vision.Fire_Mode==1)
//          {
//            mode->trig_freq=con_freq_20;
//          }
//          else if(Rx_Vision.Fire_Mode==2)
//          {
//            mode->trig_freq=con_freq_15;
//          }
//          else if(Rx_Vision.Fire_Mode==3)
//          {
//            mode->trig_freq=con_freq_12;
           
           if(Rx_Vision.armor_id==ARMOR_OUTPOST)
           {
             mode->trig_freq=con_freq_12;
           }
           else 
           {
             mode->trig_freq=con_freq_20;
           }
           
           
//          }
//          else mode->trig_freq=con_freq_12;
        }
//        mode->trig_freq=con_freq_12;
         trig_speed_set=trig_1fps_speed*mode->trig_freq;
    }
    else trig_speed_set=0;
     
  }


}
int kkkkk;
void trig_vision(trig_t *mode)
{ 
  if(TJ_Vision_Rx.Vision_gimbal_mode==control_fire)
  {
  
  if(judge_if_sin_over()==YES)
  {
    trig_ecd_set+=(trig_sin_ecd*4);
  }
    else trig_ecd_set=trig_ecd_set;
  }
  else 
  {
  kkkkk++;
   trig_ecd_set=trig_motor.motor_measure.total_ecd;
  }
  
}


void get_trig_motor_state(trig_t *mode)
{
	
		judge_if_block();
	
}



int get_block_cnt;
int gugugu;
void judge_if_block(void)
{
		
		if(trig_motor.motor_measure.feedback_current>7000&&
			trig_motor.motor_measure.speed_rpm<50&&trig_motor.motor_measure.speed_rpm>=0
		&&trig.if_block==0)
	{
		  gugugu++;
			trig.block_state.cnt++;
	}
	else 
	{
			trig.block_state.cnt=0;
	}
	
		if(trig.block_state.cnt>70)
		{
			get_block_cnt++;
			trig.if_block=1;
			if(trig.fire_state==fire_con)
			{
				trig.block_state.block_type=con_block;
			}
			else { 
			trig.block_state.block_type=sin_block;
			}
			trig.trig_flag.if_block_react_over=0;
		}

		if(trig.if_block==0&&trig.trig_flag.if_block_react==1)
		{
			trig.trig_flag.if_block_react=0;
		}
			
}



void trig_chose_freq(trig_t *mode)
{
  if(control_data.gimbal_mode==small_gimbal_pc)
  {
  
      if(((control_data.shoot_power>=400-trig_outpost_power_limit)&&Rx_Vision.armor_id==ARMOR_OUTPOST)||
          ((control_data.shoot_power>=400-trig_normol_power_limit)&&Rx_Vision.armor_id!=ARMOR_OUTPOST) )
		{
      mode->trig_freq=0;
		}
    else mode->trig_freq=con_freq_20;
    
    
  }
   else mode->trig_freq=con_freq_20;
  
}

int fffggg;
void block_react(trig_t *mode)
{
	fffggg++;
	if(mode->block_state.block_type==sin_block)
	{
		sin_block_react(mode);
	}
	
	if(mode->block_state.block_type==con_block)
	{
		con_block_react(mode);
	}
	
	
}
/**
*@note下供弹无法回拨，回拨易卡死，目前卡弹时减去当前值的1/100，主要为了卸力，
*@note后续机械装上无法反拨的拨弹盘后卡弹时保持当前ecd为目标值
*/
uint16_t react_time;

void sin_block_react(trig_t *mode)
{
	if(mode->trig_flag.if_block_react==0)
	{
		trig_ecd_set=trig_motor.motor_measure.total_ecd+trig_sin_ecd*0;
		
		mode->trig_flag.if_block_react=1;
   
	}
	

		if(mode->trig_flag.if_block_react==1&&abs(trig_ecd_set-trig_motor.motor_measure.total_ecd)<1000)
		{
			 react_time++;
		}
    if(react_time>50)
    {
      reset_block_flag(mode);
      react_time=0;
    }
	
}


int block_first;
void con_block_react(trig_t *mode)
{
	if(mode->trig_flag.if_block_react==0)
	{
		trig_ecd_set=trig_motor.motor_measure.total_ecd+trig_sin_ecd*0;
		
		mode->trig_flag.if_block_react=1;
    
	}
	

		if(mode->trig_flag.if_block_react==1&&abs(trig_ecd_set-trig_motor.motor_measure.total_ecd)<1000)
		{
    reset_block_flag(mode);
//      react_time++;
      
		}
    
//    if(react_time>1)
//    {
//      react_time=0;
//			
//    }


}


void reset_block_flag(trig_t *mode)
{
	mode->block_state.block_type=no_block;
	
	mode->trig_flag.if_block_react_over=1;
	
	mode->if_block=0;
}


int ddddmmm;
int bbbbbbb;
void trig_motor_run(trig_t *mode)
{
	if(trig_com==com_nom)
	{
//    if(gimbal.vision_on==1)
//    {
//     trig_speed_set=trig_addspeed_limit(trig_speed_set,50);
//    }
//    else trig_speed_set=trig_speed_set;
		
		if((mode->fire_state!=fire_con||
		mode->if_block==1))
		{
		ddddmmm++;
			pid_calc(&pid_trig_sin_ecd,trig_motor.motor_measure.total_ecd,trig_ecd_set);
			trig_motor.motor_tar.set_current=pid_calc(&pid_trig_sin_speed,
			trig_motor.motor_measure.speed_rpm,pid_trig_sin_ecd.output);
		
		}
		else if(mode->fire_state==fire_con&&mode->if_block==0)
		{
    bbbbbbb++;
			trig_motor.motor_tar.set_current=pid_calc(&pid_trig_con,
			trig_motor.motor_measure.speed_rpm,trig_speed_set);
		}
	}
	else 
	{
		
		trig_motor.motor_tar.set_current=0;
	}
	
}


float trig_addspeed_limit(float speed_set,int add_speed)
{ 
 
 static float last_speed;
 
 if(speed_set<last_speed)
 {
   if(speed_set-last_speed<-add_speed)
   {
     speed_set=last_speed-add_speed;
   }
   else speed_set=speed_set;
 }
 
 else speed_set=speed_set;
  
  last_speed=speed_set;
  return speed_set;
}

bool judge_if_sin_over()
{
	bool res;
	if(trig_motor.motor_measure.total_ecd-trig_ecd_set>-1400)
	{
		res = YES;
	}
	else res = NO;
	
	return res;
	
}