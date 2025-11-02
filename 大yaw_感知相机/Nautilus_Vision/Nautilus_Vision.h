/*******************************************************************
  * File Name   : Nautilus_Vision
  * Description : 视觉相关代码
  * Author      : 孙羽
  * QQ          ：984464809
  * Telephone   : 15235320302  
********************************************************************
  *
  * Copyright (c) 2022 Nautilus - Wuhan Institute of Technology
  * All rights reserved.
  *

ps: 见.c
*******************************************************************/

#ifndef __NAUTILUS_Vision_H
#define __NAUTILUS_Vision_H

#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "bsp_pid.h"
#include "remote_control.h"

#include "user_lib.h"
#include "stdint.h"
#include "stdbool.h"
//#include "Auto.h"
//#include "Chassis.h"
#include "bsp_transmit.h"

#define YES 1
#define NO 0
//任务开始空闲一段时间
#define Algorithm_TASK_INIT_TIME    5
//自动化任务控制间隔 1ms
#define Algorithm_CONTROL_TIME_MS   1

#define SEND_DATA_COUNT 29

#define CONST_HEAD0             0XA5     // 帧头
#define CONST_END0              0XAA     // 帧尾

#define CONST_HEAD1             0XB0     // 帧头
#define CONST_END1              0XBB    // 帧尾

#define CONST_HEAD2             0XB1    // 帧头
#define CONST_END2              0XBB    // 帧尾

#define TIM_5    450
#define TIM_3    200

#define ALLOWANCE_SHOOT_NUM   50
#define HP_MIN_DOUBLE         300
#define HP_MIN                150
#define ONE_MININE            60
#define HALF_ONE_MININE       30

// #define PC_DATA_COUNT 48

typedef union
{
    float data;
    uint8_t d[4];
} Algorithm_float_u;

typedef union
{
    uint16_t data;
    uint8_t d[2];
} Algorithm_16_u;


typedef __packed struct 
{  
	uint8_t bit0 : 1;  
	uint8_t bit1 : 1;  
	uint8_t bit2 : 1;  
	uint8_t bit3 : 1;  
	uint8_t bit4 : 1;  
	uint8_t bit5 : 1;  
	uint8_t bit6 : 1; 
	uint8_t bit7 : 1;  
	uint8_t bit8 : 1;
	uint8_t bit9 : 1;
	uint8_t bit10 : 1;
	uint8_t bit11 : 1;
	
  uint8_t reserved : 4;
}ByteBits_t;







typedef struct
{
	bool IF_nav;
	float pos_x_set;
	float pos_y_set;
	float yaw_set;
} Navi_t;

typedef struct
{
	bool IF_location;
	
	  float pos_x;        // 坐标x
    float pos_y;        // 坐标y
    float yaw;          // 航向角
} Odometry_t;

typedef struct 
{
	float map_x;  
	float map_y;
	float map_yaw;
}Navi_coord_t;

typedef struct
{
	Navi_t Navi;
	Odometry_t Odome;
	Navi_coord_t Navi_coord;
	uint8_t Vision_Flag;
	uint8_t IF_MOVE;
	
	uint16_t Map_Start_x;
	uint16_t Map_Start_y;
	
	uint8_t IF_turn_over;
} Auto_t;



typedef struct 
{
	uint8_t Plan;
	ByteBits_t Judge;
}Decision_info_t;

typedef struct 
{
	Odometry_t odom;  
	Navi_t  Navi;
//	Decision_info_t Decision;
}Tx_Data_t;

typedef struct 
{
	float vy_obj;  
	float vx_obj; 
	float wz_obj;
}Move_t;

typedef struct 
{
	uint16_t Start_x;
	uint16_t Start_y;
	
	int8_t delta_x[49];  
	int8_t delta_y[49];
}Path_t;

typedef struct 
{
	float map_x;  
	float map_y;
	float map_yaw;
}Navi_coordinate_t;

typedef struct 
{
	Move_t  PC_Move; 
	Path_t  PC_Path;
	Navi_coordinate_t Navi_coord;
	
	int8_t navi_state;
	uint8_t seq;
	
	uint8_t seq_path_x;
	uint8_t seq_path_y;

}Rx_Data_t;

typedef __packed struct
{
    float input;        // ê?è?êy?Y
    float out;          // ??2¨ê?3?μ?êy?Y
    float num;          // ??2¨2?êy
    float frame_period; // ??2¨μ?ê±?????? μ￥?? s
    float last_out;
} my_first_order_filter_type_t;

extern Tx_Data_t Tx_Data;
extern Rx_Data_t Rx_Data;



typedef struct 
{
	fp64 atan_angle[4];
	fp64 radian_angle[4];
	
	int16_t Forward_ecd[4];
	int16_t delt_ecd[4];
	
}Solution_t;

typedef enum
{
		
    CHASSIS_ZERO_FORCE    = 0,          //底盘无力
    CHASSIS_RC_FOLLOW     = 1,          //底盘跟随
    CHASSIS_RC_SPIRAL     = 2,          //小陀螺
    CHASSIS_STAY          = 3,
		CHASSIS_PC            = 4,          //PC控制

} Chassis_state_e;

typedef struct
{
	fp32 totalCurrentTemp;
	fp32 current[4];
	fp32 power_current[4];
	fp32 speed[4];
	fp32 POWER_MAX;
	fp32 MAX_current[4];
  fp32 SPEED_MIN;	
	fp32  K;
} Power_Control;

typedef struct									//结构体内参数只使用了一部分
{


    fp32 vx;                          //chassis vertical speed, positive means forward,unit m/s. 底盘速度 前进方向 前为正，单位 m/s
    fp32 vy;                          //chassis horizontal speed, positive means letf,unit m/s.底盘速度 左右方向 左为正  单位 m/s
    fp32 wz;                          //chassis rotation speed, positive means counterclockwise,unit rad/s.底盘旋转角速度，逆时针为正 单位 rad/s
	  fp32 vz;
    fp32 vx_set;                      //chassis set vertical speed,positive means forward,unit m/s.底盘设定速度 前进方向 前为正，单位 m/s
    fp32 vy_set;                      //chassis set horizontal speed,positive means left,unit m/s.底盘设定速度 左右方向 左为正，单位 m/s
    fp32 wz_set;                      //chassis set rotation speed,positive means counterclockwise,unit rad/s.底盘设定旋转角速度，逆时针为正 单位 rad/s


    fp32 vx_max_speed;  //max forward speed, unit m/s.前进方向最大速度 单位m/s
    fp32 vx_min_speed;  //max backward speed, unit m/s.后退方向最大速度 单位m/s
    fp32 vy_max_speed;  //max letf speed, unit m/s.左方向最大速度 单位m/s
    fp32 vy_min_speed;  //max right speed, unit m/s.右方向最大速度 单位m/s
	
		fp32 position_x;
	  fp32 position_y;
		fp32 turn_angle;
	
		int ecd_int[4];

    Chassis_state_e Chassis_state;
		
		Power_Control  power_control;
	  Power_Control  rudder_power_control;
		
		int16_t chassis_power_buffer;

} Chassis_Move_t;




void Recive_Data_Handle(Auto_t*ptr,uint8_t *buff,uint32_t Len,Rx_Data_t *data);
void Serial_Send_Data(Auto_t*auto_data,Chassis_Move_t*Move,Tx_Data_t *data);
static void Serial_Data_Handle(Auto_t*auto_data,Chassis_Move_t*Move,Tx_Data_t*data);
void my_first_order_filter_init(my_first_order_filter_type_t *first_order_filter_type, float frame_period, const float num);
void my_first_order_filter_cali(my_first_order_filter_type_t *first_order_filter_type, float input);
void Decison_Judgedata_Handle(ByteBits_t *data,uint8_t *plan);
//void Normal_decision_Handle(ByteBits_t *data,USART_Rx_data_t *vision_data);
// void Serial_Send_Init(Tx_Data_t *data_init);
#endif /* __NAUTILUS_Vision_H */

