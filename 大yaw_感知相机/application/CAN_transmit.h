#ifndef CAN_TRANSMIT_H
#define CAN_TRANSMIT_H

#include "main.h"
#include "can.h"

typedef struct 
{
	CAN_HandleTypeDef *hcan;
	uint32_t Tx_StdId;
}set_ID_t;

extern set_ID_t can1_200;
extern set_ID_t can2_200;
extern set_ID_t can1_1FF;
extern set_ID_t can2_1FF;
extern set_ID_t dm_yaw;

void set_motor_current(set_ID_t set_ID,int16_t Motor_ID_1,int16_t Motor_ID_2,int16_t Motor_ID_3,int16_t Motor_ID_4);
void dm_motor_Ctrl(set_ID_t set_ID, float _pos, float _vel, float _KP, float _KD, float _torq);
void dm_motor_START(set_ID_t set_ID);
void dm_motor_STOP(set_ID_t set_ID);
void dm_motor_SetZERO(set_ID_t set_ID);
void DM4310_motor_ClearError(set_ID_t set_ID);
int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
float Absolute_angle_Solve(float output_Position);
#endif
