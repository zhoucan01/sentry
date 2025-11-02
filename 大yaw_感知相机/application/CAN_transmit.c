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

#include "CAN_transmit.h"
#include "CAN_receive.h"

set_ID_t can1_200 = {.hcan = &hcan1,
	                   .Tx_StdId = 0x200,};
set_ID_t can1_1FF = {.hcan = &hcan1,
	                   .Tx_StdId = 0x1FF,};	
set_ID_t can2_200 = {.hcan = &hcan2,
	                   .Tx_StdId = 0x200,};
set_ID_t can2_1FF = {.hcan = &hcan2,
	                   .Tx_StdId = 0x1FF,};
set_ID_t dm_yaw = {.hcan = &hcan1,
	                   .Tx_StdId = 0x003};
	
/**
 * @description: CAN发送
 * @param {set_ID_t} set_ID
 * @param {int16_t} Motor_ID_1
 * @param {int16_t} Motor_ID_2
 * @param {int16_t} Motor_ID_3
 * @param {int16_t} Motor_ID_4
 * @return none
 */

void set_motor_current(set_ID_t set_ID,int16_t Motor_ID_1,int16_t Motor_ID_2,int16_t Motor_ID_3,int16_t Motor_ID_4)
{
	CAN_TxHeaderTypeDef tx_header;
	uint8_t tx_data[8];
	uint32_t send_mail_box;
	tx_header.StdId=set_ID.Tx_StdId;
	tx_header.IDE=CAN_ID_STD;
	tx_header.RTR=CAN_RTR_DATA;
	tx_header.DLC=0x08;
	tx_data[0]=(Motor_ID_1>>8)&0xff;
	tx_data[1]=(Motor_ID_1)&0xff;
	tx_data[2]=(Motor_ID_2>>8)&0xff;
	tx_data[3]=(Motor_ID_2)&0xff;
	tx_data[4]=(Motor_ID_3>>8)&0xff;
	tx_data[5]=(Motor_ID_3)&0xff;
	tx_data[6]=(Motor_ID_4>>8)&0xff;
	tx_data[7]=(Motor_ID_4)&0xff;
	HAL_CAN_AddTxMessage(set_ID.hcan,&tx_header,tx_data,&send_mail_box);
}

/*************************************** 4310电机发送 **************************************/
/**
 * @brief 转换函数 int-float
 * @note  需要先确定两个等比例转换的最大最小值
 */
int float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
    }
/**
 * @brief 转换函数 float-int
 * @note  需要先确定两个等比例转换的最大最小值
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
    }

/**
 * @brief 4310绝对位置解析
如下所示零点边左半圈逆时针旋转为0-180度
			  右半圈顺时针旋转为0-(-180)度
			0
	
	        +
	
		180    -180
 * @note  达妙科技电调,GM4310
 */
float Absolute_angle_Solve(float output_Position)
{
	float res;
	int n;
	res = (output_Position*57.29577f);
	n = res/180;//计算转过180度的个数
	if(n%2 == 0)//偶数圈，保留Position正负号
		res = res - n*180.0f;
	else{		//奇数圈，与Position正负号相反
		if(n>=0)
			res = res - (n+1)*180.0f;
		else res = res - (n-1)*180.0f;
	}
	return res;
}
	
/**
 * @brief MIT模式电机控制
 * @note  达妙科技电调,GM4310
 */
int gggggg;
void dm_motor_Ctrl(set_ID_t set_ID, float _pos, float _vel, float _KP, float _KD, float _torq)
{
	gggggg++;
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
	vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
	kp_tmp  = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);
	
	CAN_TxHeaderTypeDef tx_header;
	uint8_t             tx_data[8];
	
	uint32_t	pTxMailbox;

	tx_header.StdId = set_ID.Tx_StdId;
	tx_header.IDE   = CAN_ID_STD;
	tx_header.RTR   = CAN_RTR_DATA;
	tx_header.DLC   = 8;
	
	tx_data[0] = (pos_tmp >> 8);
	tx_data[1] = pos_tmp;
	tx_data[2] = (vel_tmp >> 4);
	tx_data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	tx_data[4] = kp_tmp;
	tx_data[5] = (kd_tmp >> 4);
	tx_data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	tx_data[7] = tor_tmp;
	
	HAL_CAN_AddTxMessage(set_ID.hcan, &tx_header, tx_data,&pTxMailbox); 
}	

/**
 * @brief 进入电机模式
 * @note  达妙科技电调,GM4310
 */
void dm_motor_START(set_ID_t set_ID)
{
	CAN_TxHeaderTypeDef tx_header;
	uint8_t             tx_data[8];

	tx_header.StdId = set_ID.Tx_StdId;
	tx_header.IDE   = CAN_ID_STD;
	tx_header.RTR   = CAN_RTR_DATA;
	tx_header.DLC   = 8;
	
	tx_data[0] = 0xFF;
	tx_data[1] = 0xFF;
	tx_data[2] = 0xFF;
	tx_data[3] = 0xFF;
	tx_data[4] = 0xFF;
	tx_data[5] = 0xFF;
	tx_data[6] = 0xFF;
	tx_data[7] = 0xFC;
	
	HAL_CAN_AddTxMessage(set_ID.hcan, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0); 
}	

/**
 * @brief 退出电机模式
 * @note  达妙科技电调,GM4310
 */
void dm_motor_STOP(set_ID_t set_ID)
{
	CAN_TxHeaderTypeDef tx_header;
	uint8_t             tx_data[8];

	tx_header.StdId = set_ID.Tx_StdId;
	tx_header.IDE   = CAN_ID_STD;
	tx_header.RTR   = CAN_RTR_DATA;
	tx_header.DLC   = 8;
	
	tx_data[0] = 0xFF;
	tx_data[1] = 0xFF;
	tx_data[2] = 0xFF;
	tx_data[3] = 0xFF;
	tx_data[4] = 0xFF;
	tx_data[5] = 0xFF;
	tx_data[6] = 0xFF;
	tx_data[7] = 0xFD;
	
	HAL_CAN_AddTxMessage(set_ID.hcan, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0); 
}

/**
 * @brief 设置电机零点
 * @note  达妙科技电调,GM4310
 */
void dm_motor_SetZERO(set_ID_t set_ID)
{
	CAN_TxHeaderTypeDef tx_header;
	uint8_t             tx_data[8];

	tx_header.StdId = set_ID.Tx_StdId;
	tx_header.IDE   = CAN_ID_STD;
	tx_header.RTR   = CAN_RTR_DATA;
	tx_header.DLC   = 8;
	
	tx_data[0] = 0xFF;
	tx_data[1] = 0xFF;
	tx_data[2] = 0xFF;
	tx_data[3] = 0xFF;
	tx_data[4] = 0xFF;
	tx_data[5] = 0xFF;
	tx_data[6] = 0xFF;
	tx_data[7] = 0xFE;
	
	HAL_CAN_AddTxMessage(set_ID.hcan, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0); 
}	

/**
 * @brief 清除错误
 * @note  达妙科技电调,GM4310
 */
void DM4310_motor_ClearError(set_ID_t set_ID)
{
	CAN_TxHeaderTypeDef tx_header;
	uint8_t             tx_data[8];

	tx_header.StdId = set_ID.Tx_StdId;
	tx_header.IDE   = CAN_ID_STD;
	tx_header.RTR   = CAN_RTR_DATA;
	tx_header.DLC   = 8;
	
	tx_data[0] = 0xFF;
	tx_data[1] = 0xFF;
	tx_data[2] = 0xFF;
	tx_data[3] = 0xFF;
	tx_data[4] = 0xFF;
	tx_data[5] = 0xFF;
	tx_data[6] = 0xFF;
	tx_data[7] = 0xFB;
	
	HAL_CAN_AddTxMessage( set_ID.hcan, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0); 
}	