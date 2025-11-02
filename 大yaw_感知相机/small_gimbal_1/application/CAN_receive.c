

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


//motor_data_t chassis_motor[4];
//motor_data_t yaw_motor;
//motor_data_t shoot_motor[2];
//motor_data_t trig_motor;
//motor_data_t steer_motor[4];
SuperCap_t SuperCAP;
DM_motor_measure_t pitch_motor;
DM_motor_measure_t yaw_motor;
receive_gimbal_data_t receive_gimbal_data;
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
				
				case RECEIVE_ID_2:
        {
        get_gimbal_data_2(rx_data,&receive_gimbal_data);
         break;
        }
				  
				
				
				
				case 0x002:
				{
					get_dm4310_motor_measure(&yaw_motor ,rx_data);
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
				
        case RECEIVE_ID_1:
        {
          get_gimbal_data_1(rx_data,&receive_gimbal_data);
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
void get_dm4310_motor_measure(DM_motor_measure_t *ptr, uint8_t data[])
{
  
  ptr->Err = data[0] >> 4;
	ptr->Position_int=(data[1]<<8)|data[2];
	ptr->Velocity_int =(data[3]<<4)|(data[4]>>4);
	ptr->Torque_int =((data[4]&0xF)<<8)|data[5];
	
	ptr->Position = uint_to_float(ptr->Position_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
	ptr->Velocity = uint_to_float(ptr->Velocity_int, V_MIN, V_MAX, 12); // (-45.0,45)
	ptr->Torque = uint_to_float(ptr->Torque_int, T_MIN, T_MAX, 12); // (-18.0,18)
	ptr->Angle   = ptr->Position * 57.29577f;
	
	ptr->T_MOS = data[6];
	ptr->T_Rotor = data[7];
	
}


void get_gimbal_data_1(uint8_t rx_data[8],receive_gimbal_data_t *data)
{
  data->gimbal_1_found=rx_data[0];
}

void get_gimbal_data_2(uint8_t rx_data[8],receive_gimbal_data_t *data)
{
  data->gimbal_2_found=rx_data[0];
}