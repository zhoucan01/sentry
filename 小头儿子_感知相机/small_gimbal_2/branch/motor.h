#ifndef __MOTOR_H
#define __MOTOR_H

#include "cmsis_os.h"
#include "Nautilus_Vision.h"
void pitch_Enable(void);
void 	pitch_Unable(void);
void gimbal_normol_send();
void gimbal_error_send();

#define send_id 0x401

void shoot_error_send();
void shoot_normol_send();
void dm_pitch_start();


void small_to_big(CAN_HandleTypeDef *hcan,uint32_t id_range);
#endif