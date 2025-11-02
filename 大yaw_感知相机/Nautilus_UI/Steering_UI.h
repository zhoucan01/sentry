///*******************************************************************
//  * File Name   : Steering_UI
//  * Description : 舵轮步兵UI的使用，基于孙老师UI
//  * Author      : 刘嘉诚
//  * QQ          ：
//  * Telephone   : 
//********************************************************************
//  *
//  * Copyright (c) 2022 Nautilus - Wuhan Institute of Technology
//  * All rights reserved.
//  *

//ps: RoboMaster_裁判系统串口协议附录 V1.3
//	频率最大为10Hz
//*******************************************************************/
//#ifndef __STEERING_UI_H
//#define __STEERING_UI_H

//#include "Nautilus_UI.h"


//#define Circle_Wide	10			//烧饼射击状态圆环弧线宽度（此圆圈宏定义未使用）
//#define Circle_Rad	30			//圆环半径
//#define Circle_Loc_X	960		//圆环位置
//#define Circle_Loc_Y	540

//#define ROBOT_RAD	30
//#define ROBOT_WIDE	10
//#define ROBOT_LOC_Y	1080-160
//#define ROB1_LOCAT	750			//红方英雄X位置
//#define ROB2_LOCAT	630
//#define ROB3_LOCAT	510
//#define ROB4_LOCAT	390
//#define ROB5_LOCAT	270
//#define ROB7_LOC_X	200
//#define ROB7_LOC_Y	1080-220
//#define ROB7_RAD	30

//#define FLOAT_DELTA_X	150
//#define FLOAT_DELTA_Y	50


//void UI_SEND(void);
//void UI_ID_Set(void);
//void UI_Cap_Voltage(void);				//电容电量
//void UI_Gun_Sight(void);				//瞄准镜
//void UI_Pitch_Angle(void);				//pitch轴角度显示
//void UI_Shoot_Mode(int8_t state);
//void UI_Robot1_Mode(int8_t state);		//敌方1号机器人状态
//void UI_Robot2_Mode(int8_t state);
//void UI_Robot3_Mode(int8_t state);
//void UI_Robot4_Mode(int8_t state);
//void UI_Robot5_Mode(int8_t state);
//void UI_Robot7_Mode(int8_t state);
//void UI_Robot10_Mode(int8_t state);
//void UI_MyRobot_Mode(uint8_t state);	//我方哨兵跟随机器人状态
//void UI_Sentry_Angle(float Angle_1,float Angle_2,float Angle_3,float Angle_4,float Angle_5,float Angle_6,float Angle_7);
//void UI_Data_Disp(int data1, int data2);

//void UI_Reset(void);

//extern FloInt_Data_t SC_Vol;

//extern FloInt_Data_t Pitch_Angle;
//extern FloInt_Data_t Int_Data1;
//extern FloInt_Data_t Int_Data2;

//extern FloInt_Data_t MyRobot_StateID1;
//extern FloInt_Data_t MyRobot_StateID2;
//extern FloInt_Data_t MyRobot_StateID3;
//extern FloInt_Data_t MyRobot_StateID4;
//extern FloInt_Data_t MyRobot_StateID5;
//extern FloInt_Data_t Robot_StateID1;
//extern FloInt_Data_t Robot_StateID2;
//extern FloInt_Data_t Robot_StateID3;
//extern FloInt_Data_t Robot_StateID4;
//extern FloInt_Data_t Robot_StateID5;

//extern FloInt_Data_t Sentry_Angle_1;
//extern FloInt_Data_t Sentry_Angle_2;
//extern FloInt_Data_t Sentry_Angle_3;
//extern FloInt_Data_t Sentry_Angle_4;
//extern FloInt_Data_t Sentry_Angle_5;
//extern FloInt_Data_t Sentry_Angle_6;
//extern FloInt_Data_t Sentry_Angle_7;



//extern Graph_Data_t GUN_Sight_MainAxis;			//瞄准镜中轴
//extern Graph_Data_t GUN_Sight_LateralAxis_1;	//横轴线 1
//extern Graph_Data_t GUN_Sight_LateralAxis_2;	//横轴线 2
//extern Graph_Data_t GUN_Sight_LateralAxis_3;	//横轴线 3
//extern Graph_Data_t GUN_Sight_LateralAxis_4;
//extern Graph_Data_t GUN_Sight_LateralAxis_5;
//extern Graph_Data_t GUN_Sight_LateralAxis_6;

//extern Graph_Data_t GUN_Sight_LateralAxis_7;
//extern Graph_Data_t GUN_Sight_LateralAxis_8;
//extern Graph_Data_t GUN_Sight_LateralAxis_9;
//extern Graph_Data_t GUN_Sight_LateralAxis_10;

//extern FloInt_Data_t GUN_Sight_MeterAxis_1;
//extern FloInt_Data_t GUN_Sight_MeterAxis_2;
//extern FloInt_Data_t GUN_Sight_MeterAxis_3;


//extern Graph_Data_t MyRobot_Mode_1;
//extern Graph_Data_t MyRobot_Mode_2;
//extern Graph_Data_t MyRobot_Mode_3;
//extern Graph_Data_t MyRobot_Mode_4;
//extern Graph_Data_t MyRobot_Mode_5;
//extern Graph_Data_t MyRobot_Mode_7;
//extern Graph_Data_t MyRobot_Mode_10;
//extern Graph_Data_t Robot_Mode_1;
//extern Graph_Data_t Robot_Mode_2;
//extern Graph_Data_t Robot_Mode_3;
//extern Graph_Data_t Robot_Mode_4;
//extern Graph_Data_t Robot_Mode_5;
//extern Graph_Data_t Robot_Mode_7;
//extern Graph_Data_t Robot_Mode_10;

//extern Graph_Data_t Shoot_Mode_1;

//#endif





























/*******************************************************************
  * File Name   : Steering_UI
  * Description : 舵轮步兵UI的使用，基于孙老师UI
  * Author      : 刘嘉诚
  * QQ          ：
  * Telephone   : 
********************************************************************
  *
  * Copyright (c) 2022 Nautilus - Wuhan Institute of Technology
  * All rights reserved.
  *

ps: RoboMaster_裁判系统串口协议附录 V1.3
	频率最大为10Hz
*******************************************************************/
#ifndef __STEERING_UI_H
#define __STEERING_UI_H

#include "Nautilus_UI.h"

void UI_Task(void);
void UI_ID_Set(void);
void UI_Cap_Voltage(void);//电容电量
void UI_Gun_Sight(void);//瞄准镜
void UI_Action_Mode(void);//运动模式显示
void UI_Hostile_Direction(void);//敌对方向提示
void UI_Anticollition(void);//防撞提示
void UI_Pitch_Angle(void);//pitch轴角度显示
void UI_Yaw_Angle(void);
void UI_Fric_Mode(void);
void UI_Shoot_Mode(void);
void UI_Vision_state(void);
void UI_Cap(void);
void UI_Special_ACT(void);
void UI_Vision_Outpost_Mode(void);
void UI_Run_Warning(void);
void Vision_IF_Ready(void);


void UI_Reset(void);

extern FloInt_Data_t SC_Vol;

extern FloInt_Data_t Pitch_Angle;
extern FloInt_Data_t Fricmotor_Speed;

extern FloInt_Data_t Yaw_Angle;


extern Graph_Data_t GUN_Sight_MainAxis;//瞄准镜中轴
extern Graph_Data_t GUN_Sight_LateralAxis_1;//横轴线 1
extern Graph_Data_t GUN_Sight_LateralAxis_2;//横轴线 2
extern Graph_Data_t GUN_Sight_LateralAxis_3;//横轴线 3
extern Graph_Data_t GUN_Sight_LateralAxis_4;
extern Graph_Data_t GUN_Sight_LateralAxis_5;
extern Graph_Data_t GUN_Sight_LateralAxis_6;
extern Graph_Data_t GUN_Sight_LateralAxis_7;
extern Graph_Data_t GUN_Sight_LateralAxis_8;
extern Graph_Data_t GUN_Sight_LateralAxis_9;
extern Graph_Data_t GUN_Sight_LateralAxis_10;

extern FloInt_Data_t GUN_Sight_MeterAxis_1;
extern FloInt_Data_t GUN_Sight_MeterAxis_2;
extern FloInt_Data_t GUN_Sight_MeterAxis_3;

extern Graph_Data_t miao_zhun_kuang;


extern Graph_Data_t Hostile_Direction_1;//右斜线"/"
extern Graph_Data_t Hostile_Direction_2;//左斜线"\"

extern Graph_Data_t Anticollition_1;//防撞提示-横线
extern Graph_Data_t Anticollition_2;//左斜"\"
extern Graph_Data_t Anticollition_3;//右斜"/"

extern Graph_Data_t SC_Vol_R;
extern Graph_Data_t SC_Vol_P;
extern FloInt_Data_t SC_Vol_N;
extern Graph_Data_t Car_Body;

extern Graph_Data_t SC_Outline_Arc_1;
extern Graph_Data_t SC_Outline_Arc_2;
extern Graph_Data_t SC_Outline_Line_1;
extern Graph_Data_t SC_Outline_Line_2;
extern Graph_Data_t SC_Vol_Arc;


extern Graph_Data_t visual_field_1;
extern Graph_Data_t visual_field_2;
extern Graph_Data_t visual_field_3;
extern Graph_Data_t visual_field_4;
extern Graph_Data_t Vision_Mode_1;	//动态

extern Graph_Data_t Special_ACT_1;//云台朝向
extern Graph_Data_t Special_ACT_2;//边框圆
extern Graph_Data_t Special_ACT_3;//灯条1/4圆

extern String_Data_t Vision_state_yaw;
extern String_Data_t Vision_state_pit;

extern Graph_Data_t Action_Mode_1;

extern Graph_Data_t Fric_Mode_1;

extern Graph_Data_t Shoot_Mode_1;

extern Graph_Data_t Vision_Outpost_Mode;

extern Graph_Data_t Gimbal_connect_status;


extern Graph_Data_t Run_Warning_1;
extern Graph_Data_t Run_Warning_2;
extern Graph_Data_t Run_Warning_3;
extern Graph_Data_t Run_Warning_4;
extern Graph_Data_t Run_Warning_5;
extern Graph_Data_t Run_Warning_6;
extern Graph_Data_t Run_Warning_7;
extern Graph_Data_t Run_Warning_8;
extern Graph_Data_t Run_Warning_9;
extern Graph_Data_t Vision_Ready_mode;

void Gimbal_status_mode(void) ;

void UI_Vision_Mode(void);

#endif
