#ifndef _BSP_CAN_H
#define _BSP_CAN_H

#include "can.h"
void can_filter_init(void);
void can1_user_init(CAN_HandleTypeDef* hcan );
void can2_user_init(CAN_HandleTypeDef* hcan );
#endif
