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
#include "user_lib.h"
#include "stdbool.h"
#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "bsp_pid.h"
//#include "remote_control.h"
//#include "Auto.h"
//任务开始空闲一段时间
#define Algorithm_TASK_INIT_TIME    5
//自动化任务控制间隔 1ms
#define Algorithm_CONTROL_TIME_MS   1

#define SEND_DATA_COUNT 29
#define VISION_RX_COUNT 15
#define VISION_TX_COUNT 25

#define VISION_TX_HEAD    0XA5 
#define VISION_TX_TAIL    0XAA 

#define VISION_RX_HEAD    0XA5 
#define VISION_RX_TAIL    0XAA 

#define CONST_HEAD0             0XA5     // 帧头
#define CONST_END0              0XAA     // 帧尾

#define CONST_HEAD1             0XB0     // 帧头
#define CONST_END1              0XBB    // 帧尾

#define CONST_HEAD2             0XB1    // 帧头
#define CONST_END2              0XBB    // 帧尾

#define PC_DATA_COUNT 48

#define GRAVITY 9.78

typedef struct
{
	uint8_t  Vision_mode;              // 视觉模式
	uint32_t target_sight_count;       // 跑打跑时连续射击的时间
	uint32_t target_lostsight_count;   // 目标丢失次数
	uint32_t target_found_count;       // 视觉发现次数
	uint8_t target_lost_flag;          // 目标是否丢失标志
	uint8_t vision_Error_Flag;         // 错误标志
} Vision_state_t;

typedef __packed struct 
{  
	uint8_t detect_color : 1;      // 当前车的颜色
	bool reset_tracker : 1;             
	uint8_t reserved : 6; 
}VisionBits_t;

typedef struct
{
	uint8_t head; 
  uint8_t robot_color;
	uint8_t Vision_Mode;
	
	float roll;
  float pitch;
	float yaw;
	
	bool shoot_all;
	bool shoot_Hero;
	bool shoot_Engineer;
	bool shoot_balance;
	bool shoot_infantr4;
	bool shoot_infantr5;
	bool shoot_Sentry;
	bool shoot_outpost;
	bool shoot_base;
	
	uint8_t tail ;
	
}Vision_Send_Data_t;
extern Vision_Send_Data_t Send_Vision;
typedef __packed struct 
{  
	bool tracking : 1;      
	uint8_t id : 3;             
	uint8_t armors_num : 3; 
  uint8_t reserved : 1;	
}Rx_VisionBits_t;

typedef unsigned char uint8_t;
enum ARMOR_ID
{
    ARMOR_OUTPOST = 0,
    ARMOR_HERO = 1,
    ARMOR_ENGINEER = 2,
    ARMOR_INFANTRY3 = 3,
    ARMOR_INFANTRY4 = 4,
    ARMOR_INFANTRY5 = 5,
    ARMOR_GUARD = 6,
    ARMOR_BASE = 7
};

typedef struct
{
	uint8_t head;                    // 帧头

	float pitch_obj;
	float yaw_obj;
	
  
  float yaw_add;
  float total_yaw;
	uint8_t Flag_Found;
	enum ARMOR_ID armor_id;
	uint8_t fire;
	uint8_t Fire_Mode;
	
	uint8_t Vision_seq;
	
	uint8_t  IF_stay;
	
	uint8_t tail;
	
}Vision_rx_Data_t;

enum ARMOR_NUM
{
    ARMOR_NUM_BALANCE = 2,
    ARMOR_NUM_OUTPOST = 3,
    ARMOR_NUM_NORMAL = 4
};

enum BULLET_TYPE
{
    BULLET_17 = 0,
    BULLET_42 = 1
};


//设置参数
struct SolveTrajectoryParams
{
    float k;             //弹道系数

    //自身参数
    enum BULLET_TYPE bullet_type;  //自身机器人类型 0-步兵 1-英雄
    float current_v;      //当前弹速
    float current_pitch;  //当前pitch
    float current_yaw;    //当前yaw

    //目标参数
    float xw;             //ROS坐标系下的x
    float yw;             //ROS坐标系下的y
    float zw;             //ROS坐标系下的z
    float vxw;            //ROS坐标系下的vx
    float vyw;            //ROS坐标系下的vy
    float vzw;            //ROS坐标系下的vz
    float tar_yaw;        //目标yaw
    float v_yaw;          //目标yaw速度
    float r1;             //目标中心到前后装甲板的距离
    float r2;             //目标中心到左右装甲板的距离
    float dz;             //另一对装甲板的相对于被跟踪装甲板的高度差
    int bias_time;        //偏置时间
    float s_bias;         //枪口前推的距离
    float z_bias;         //yaw轴电机到枪口水平面的垂直距离
    enum ARMOR_ID armor_id;     //装甲板类型  0-outpost 6-guard 7-base
                                //1-英雄 2-工程 3-4-5-步兵 
    enum ARMOR_NUM armor_num;   //装甲板数字  2-balance 3-outpost 4-normal
};

//用于存储目标装甲板的信息
struct tar_pos
{
    float x;           //装甲板在世界坐标系下的x
    float y;           //装甲板在世界坐标系下的y
    float z;           //装甲板在世界坐标系下的z
    float yaw;         //装甲板坐标系相对于世界坐标系的yaw角
};
//单方向空气阻力模型
extern float monoDirectionalAirResistanceModel(float s, float v, float angle);
//完全空气阻力模型
extern float completeAirResistanceModel(float s, float v, float angle);
//pitch弹道补偿
extern float pitchTrajectoryCompensation(float s, float y, float v);
//根据最优决策得出被击打装甲板 自动解算弹道
extern void autoSolveTrajectory(float *pitch, float *yaw, float *aim_x, float *aim_y, float *aim_z);


//typedef union
//{
//    float data;
//    uint8_t d[4];
//} Algorithm_float_u;

//typedef union
//{
//    int16_t  data;
//    uint8_t d[2];
//} Algorithm_16_u;

//typedef union
//{
//    uint16_t  data;
//    uint8_t d[2];
//} Algorithm_8_u;


//typedef struct 
//{
//Odometry_t odom;  
//Navi_t  Navi;
//}Tx_Data_t;

typedef struct 
{
	float vy_obj;  
	float vx_obj; 
	float wz_obj;
}Move_t;

typedef struct 
{
	int8_t delta_x[49];
	int8_t delta_y[49];
}Path_t;

typedef struct 
{
	Move_t PC_Move; 
	Path_t  PC_Patth;
}Rx_Data_t;



//extern  Tx_Data_t Tx_Data;
//extern Rx_Data_t Rx_Data;
extern Vision_Send_Data_t Send_Vision;
extern Vision_rx_Data_t Rx_Vision;
extern Vision_state_t Vision_state;

extern first_order_filter_type_t vision_filter_yaw;
extern first_order_filter_type_t vision_filter_pitch;

extern pid_struct_t pid_Vision_Pitch_angle;   //Pitch轴云台角度PID结构体
extern pid_struct_t pid_Vision_Pitch_speed;   //Pitch轴云台速度PID结构体
extern pid_struct_t pid_Vision_Yaw_angle;     //Yaw轴云台角度PID结构体
extern pid_struct_t pid_Vision_Yaw_speed;     //Yaw轴云台速度PID结构体

extern pid_struct_t pid_Curise_Pitch_angle;   //Pitch轴云台角度PID结构体
extern pid_struct_t pid_Curise_Pitch_speed;   //Pitch轴云台速度PID结构体
extern pid_struct_t pid_Curise_Yaw_angle;     //Yaw轴云台角度PID结构体
extern pid_struct_t pid_Curise_Yaw_speed;     //Yaw轴云台速度PID结构体


void Recive_Data_Handle(uint8_t *buff,uint32_t Len,Rx_Data_t *data);
//void Serial_Send_Data(Auto_t*auto_data,Chassis_Move_t*Move,Tx_Data_t *data);
//static void Serial_Data_Handle(Auto_t*auto_data,Chassis_Move_t*Move,Tx_Data_t*data);
void Rx_Vision_Handle(Vision_rx_Data_t *data,uint8_t *buff,uint32_t Len);
void Vision_Task(void);
void Vision_Send_Data(Vision_Send_Data_t *data);
void Vision_Gimbal_Init();



#endif /* __NAUTILUS_Vision_H */

