

/*
 * _______________#########_______________________ 
 * ______________############_____________________ 
 * ______________#############____________________ 
 * _____________##__###########___________________ 
 * ____________###__######_#####__________________ 
 * ____________###_#######___####_________________ 
 * ___________###__##########_####________________ 
 * __________####__###########_####_______________ 
 * ________#####___###########__#####_____________ 
 * _______######___###_########___#####___________ 
 * _______#####___###___########___######_________ 
 * ______######___###__###########___######_______ 
 * _____######___####_##############__######______ 
 * ____#######__#####################_#######_____ 
 * ____#######__##############################____ 
 * ___#######__######_#################_#######___ 
 * ___#######__######_######_#########___######___ 
 * ___#######____##__######___######_____######___ 
 * ___#######________######____#####_____#####____ 
 * ____######________#####_____#####_____####_____ 
 * _____#####________####______#####_____###______ 
 * ______#####______;###________###______#________ 
 * ________##_______####________####______________ 
 */
/*
 *   佛曰:  
 *        写字楼里写字间，写字间里程序员；  
 *        程序人员写程序，又拿程序换酒钱。  
 *        酒醒只在网上坐，酒醉还来网下眠；  
 *        酒醉酒醒日复日，网上网下年复年。  
 *        但愿老死电脑间，不愿鞠躬老板前；  
 *        奔驰宝马贵者趣，公交自行程序员。  
 *        别人笑我忒疯癫，我笑自己命太贱；  
 *        不见满街漂亮妹，哪个归得程序员？
 */

#include "main.h"
#include "CAN_receive.h"
#include "can.h"
#include "CAN_transmit.h"
#include "detect_task.h"

motor_data_t chassis_motor[4];
motor_data_t yaw_motor;
motor_data_t shoot_motor[2];
motor_data_t trig_motor;
motor_data_t steer_motor[4];
SuperCap_t SuperCAP;
DM_motor_measure_t pitch_motor;
float pm_current=0.0f;
float pm_voltage=0.0f;
float pm_power=0.0f;
int opoj;
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
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t rx_data[8];
	CAN_RxHeaderTypeDef rx_header;
	if(hcan->Instance == hcan1.Instance)
	{
		if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_header,rx_data) == HAL_OK)
		{
			switch(rx_header.StdId)
			{

				case FR_3508_ID:
				{
					get_motor_measure(&chassis_motor[FR].motor_measure,rx_data);
          
          communication_frame_rate_update(CHASSIS_MOTOR1_TOE);
					break;
				}
				case BR_3508_ID:
				{
					get_motor_measure(&chassis_motor[BR].motor_measure,rx_data);
          communication_frame_rate_update(CHASSIS_MOTOR2_TOE);
					break;
				}
				
				
				case BL_3508_ID:
				{
					get_motor_measure(&chassis_motor[BL].motor_measure,rx_data);
          communication_frame_rate_update(CHASSIS_MOTOR3_TOE);
					break;
				}
				case FL_3508_ID:
				{
					get_motor_measure(&chassis_motor[FL].motor_measure,rx_data);
          communication_frame_rate_update(CHASSIS_MOTOR4_TOE);
					break;
				}
				// 超级电容

				case pm_id:
        {
          pm_voltage=(float)((int32_t)(rx_data[1]<<8)|(int32_t)(rx_data[0]))/100.0;
          pm_current=(float)((int32_t)(rx_data[3]<<8)|(int32_t)(rx_data[2]))/100.0;
          pm_power = pm_voltage*pm_current;
          break;
        }
				
//				case 0x00:
//				{
//					get_dm4310_motor_measure(&dm_pitch_motor ,rx_data);
//					break;
//				}
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
				        
				
				case FR_6020_ID:
				{
					get_motor_measure(&steer_motor[FR].motor_measure,rx_data);
          
					break;
					
				}
				case BR_6020_ID:
				{
					get_motor_measure(&steer_motor[BR].motor_measure,rx_data);

					break;
				}
				case BL_6020_ID:
				{
					get_motor_measure(&steer_motor[BL].motor_measure,rx_data);
          
					break;
				}
				case FL_6020_ID:
				{
					get_motor_measure(&steer_motor[FL].motor_measure,rx_data);
          
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
	ptr->Position_int = (rx_data[1] << 8) | rx_data[2];
	ptr->Velocity_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
	ptr->Torque_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
	ptr->Position = uint_to_float(ptr->Position_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
	ptr->Velocity = uint_to_float(ptr->Velocity_int, V_MIN, V_MAX, 12); // (-45.0,45.0)
	ptr->Torque = uint_to_float(ptr->Torque_int, T_MIN, T_MAX, 12);
	ptr->Angle = Absolute_angle_Solve(ptr->Position);
}