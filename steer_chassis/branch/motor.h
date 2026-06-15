/**
 ******************************************************************************
 * @file    motor.h
 * @brief   电机CAN通信头文件
 ******************************************************************************
 */

#ifndef __MOTOR_H
#define __MOTOR_H

void steer_normol_send(void);
void steer_error_send(void);
void chassis_normol_send(void);
void chassis_error_send(void);

#endif /* __MOTOR_H */
