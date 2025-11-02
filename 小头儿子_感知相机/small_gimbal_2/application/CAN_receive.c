
#include "main.h"
#include "CAN_receive.h"
#include "can.h"
#include "CAN_transmit.h"
#include "system.h"
#include "Nautilus_Vision.h"
//motor_data_t chassis_motor[4];
//motor_data_t yaw_motor;
//motor_data_t shoot_motor[2];
//motor_data_t trig_motor;
//motor_data_t steer_motor[4];
SuperCap_t SuperCAP;
motor_data_t yaw_motor;
motor_data_t shoot_motor[2];
DM_motor_measure_t pitch_motor;
motor_data_t trig_motor;
//DM_motor_measure_t yaw_motor;
control_data_t control_data;
//receive_gimbal_data_t receive_gimbal_data;
void get_motor_measure(motor_measure_t *ptr, uint8_t data[8])                                                     
	{                                                                                    
		ptr->last_ecd = ptr->ecd;                                                    
		ptr->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);                             
		ptr->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);                       
		ptr->feedback_current = (uint16_t)((data)[4] << 8 | (data)[5]);                
		ptr->temperate = (data)[6];                                                 		
		ptr->ECD_angle = (ptr->ecd *360) / 8192;                                          
		if ((ptr->ecd - ptr->last_ecd) > 4096)                                       
		{                                                                               
			(ptr->round_cnt)--;                                                        
		}                                                                                
		else if (ptr->ecd - ptr->last_ecd < -4096)                                   
		{                                                                                
			(ptr->round_cnt)++;                                                        
		}                                                                                
		ptr->total_ecd = (ptr->round_cnt * 8192) + (ptr->ecd - ptr->offset_ecd); 
		ptr->total_angle = (ptr->round_cnt * 360) + (ptr->ECD_angle);                 
	}                                                                                  

//	int aaa = 0;

int lmlml;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t rx_data[8];
	CAN_RxHeaderTypeDef rx_header;
	if(hcan->Instance == hcan1.Instance)
	{
		if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_header,rx_data) == HAL_OK)
		{
    lmlml++;
			switch(rx_header.StdId)
			{
				
//				case RECEIVE_ID_2:
//        {
//        get_gimbal_data_2(rx_data,&receive_gimbal_data);
//         break;
//        }
				  
				
				case trig_motor_id:
        {
          get_motor_measure(&trig_motor.motor_measure,rx_data);
          break;
        }

        case FRICR_L_ID:
        {
         get_motor_measure(&shoot_motor[0].motor_measure,rx_data);
         break;
        }
        
        
        case FRICR_R_ID:
        {
         get_motor_measure(&shoot_motor[1].motor_measure,rx_data);
         break;
        }
				
				case SEND_ID_HEAD2:
        {
         get_gimbal_data(rx_data,&control_data);
         communication_frame_rate_update(BOARD_TOE);
         break;
        }
        
        case SHOOT_SEND_ID_HEAD2:
        {
         get_shoot_data(rx_data,&control_data);
         break;
        }
        
        
				default:
				{
					
          break;
				}
			}	
		}
	}
	
}



void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef*hcan)
{
	uint8_t rx_data[8];
	CAN_RxHeaderTypeDef rx_header;
	if(hcan->Instance == hcan2.Instance)
	{
		if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO1,&rx_header,rx_data ) == HAL_OK)
		{
			switch(rx_header.StdId)
			{
				
//        case RECEIVE_ID_1:
//        {
//          get_gimbal_data_1(rx_data,&receive_gimbal_data);
//          break;
//        }
       case pitch_motor_id:
				{
					get_dm4310_motor_measure(&pitch_motor ,rx_data);
  
          communication_frame_rate_update(PITCH_GIMBAL_MOTOR_TOE);
					break;
				}
        
        case yaw_motor_id:
        {
          get_motor_measure(&yaw_motor.motor_measure,rx_data);
          break;
        }
        
        
        
        
        default:
				{
					
          break;
				}
        
			}
		}

	}
}

const SuperCap_t *get_supercup_pointer(void)
{
	return &SuperCAP;
}


/** 
  * @brief        获取4310电机数据
  * @param        {DM_motor_measure_t} *ptr   4310电机结构体指针
  * @param        {uint8_t} rx_data[]   can数据地址
  * @return       {*}
  */
void get_dm4310_motor_measure(DM_motor_measure_t *ptr, uint8_t rx_data[])
{
  
  ptr->Err        = rx_data[0]>>4;
	ptr->Position_int = (rx_data[1] << 8) | rx_data[2];
	ptr->Velocity_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
	ptr->Torque_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
	ptr->Position = uint_to_float(ptr->Position_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
	ptr->Velocity = uint_to_float(ptr->Velocity_int, V_MIN, V_MAX, 12); // (-45.0,45.0)
	ptr->Torque = uint_to_float(ptr->Torque_int, T_MIN, T_MAX, 12);
	ptr->Angle = Absolute_angle_Solve(ptr->Position);
	
}


void get_gimbal_data(uint8_t rx_data[8],control_data_t *data)
{
  
  Algorithm_16_u data_yaw,data_pitch,data_wheel;
  

  
  
  data->small_control_pack=*((small_control_pack_t*)&rx_data[0]);
  data_yaw.d[0]=rx_data[1];
  data_yaw.d[1]=rx_data[2];
  data_pitch.d[0]=rx_data[3];
  data_pitch.d[1]=rx_data[4];
  data->other_data=*((other_data_t*)&rx_data[5]);
  data->top_senior_priority=rx_data[6];
  
  data->if_arrvied=data->other_data.if_arrvied;
  data_wheel.d[0]=0;
  data_wheel.d[1]=0;
  data->gimbal_mode=data->small_control_pack.gimbal_mode;
  data->robot_color=data->small_control_pack.robot_color;
  data->if_gimbal_can=data->small_control_pack.pitch_can;
  data->shoot_mode=data->small_control_pack.shoot_mode;
  gimbal_system.vision_mode=data->small_control_pack.vision_mode;
  data->trig_limit=data->small_control_pack.trig_mode;
  data->gimbal_yaw_in=data_yaw.data;
  data->gimbal_pitch_in=data_pitch.data;
  data->data_wheel=data_wheel.data;
  
}


void get_shoot_data(uint8_t rx_data[8],control_data_t *data)
{
 Algorithm_8_u data_num,data_speed,data_power;
 
// data->robot_color=rx_data[0];
// data_num.d[0]=rx_data[1];
// data_num.d[1]=rx_data[2];
// data_speed.d[0]=rx_data[3];
// data_speed.d[1]=rx_data[4];
// data_power.d[0]=rx_data[5];
// data_power.d[1]=rx_data[6];
// data->shoot_mode=rx_data[7];

//Send_Vision.shoot_all=data
data->Vision_ByteBits=*((Vision_ByteBits_t*)&rx_data[0]);
 data_num.d[0]=rx_data[1];
 data_num.d[1]=rx_data[2];
 data_speed.d[0]=rx_data[3];
 data_speed.d[1]=rx_data[4];
 data_power.d[0]=rx_data[5];
 data_power.d[1]=rx_data[6];
// data->shoot_mode=rx_data[7];
  data->curise_mode=rx_data[7];
  
  
  
 data->shoot_num=data_num.data;
 data->shoot_speed=(float)data_speed.data/10;
 data->shoot_power=data_power.data;
  
}