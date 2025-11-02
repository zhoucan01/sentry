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

/*************
添加车身位置
pitch轴视觉角度校准




车身校准占一位

*************/
#include "Steering_UI.h"
#include "bsp_can.h"
#include "referee.h"
#include "INS_task.h"
#include "Nautilus_Vision.h"
#include "remote_control.h"
#include "shoot.h"

#include "math.h"
#include "cmsis_os.h"


/**
 * @brief UI发送总函数
 * @note  UI send
 * @param 
 */
int UI_cnt;
void UI_Task(void)
{
	while(1)
	{
		UI_cnt++;
		UI_ID_Set();
		if(UI_cnt == 1)
		{
			UI_Gun_Sight();			//瞄准镜
			UI_Anticollition();		//防撞提示
//			UI_SendGraph(7, GUN_Sight_MainAxis, GUN_Sight_LateralAxis_1, GUN_Sight_LateralAxis_2, GUN_Sight_LateralAxis_3, GUN_Sight_LateralAxis_4, GUN_Sight_LateralAxis_5, GUN_Sight_LateralAxis_6);
//			UI_SendGraph(7,GUN_Sight_LateralAxis_7,GUN_Sight_LateralAxis_8,GUN_Sight_LateralAxis_9,GUN_Sight_MeterAxis_1,GUN_Sight_LateralAxis_10);
//		  UI_SendGraph(7,GUN_Sight_MeterAxis_2,GUN_Sight_MeterAxis_3,Anticollition_1,Anticollition_2,Anticollition_3);
			UI_SendGraph(1,miao_zhun_kuang);
//			UI_SendGraph(7,Run_Warning_3,Run_Warning_4,Run_Warning_5,Run_Warning_6,Run_Warning_7,Run_Warning_8,Run_Warning_9);
			
		}
		if(UI_cnt == 5)
		{	
			UI_Vision_state();
			UI_Cap_Voltage();		//电容电压
			UI_Cap();
			UI_Pitch_Angle();		//pitch轴角度
			UI_Yaw_Angle();
			UI_Hostile_Direction();	//敌方提示
			UI_Action_Mode();
			UI_Shoot_Mode();
			UI_Run_Warning();
			Vision_IF_Ready();
			UI_Vision_Outpost_Mode();
			UI_Fric_Mode();
			Gimbal_status_mode();
			UI_Special_ACT();
			UI_Hostile_Direction();
			UI_Reset();				//放在浮点型之后运行
			UI_SendGraph(5, SC_Vol, Pitch_Angle,  Hostile_Direction_1, Fric_Mode_1,Action_Mode_1);//, Vision_state_yaw, Vision_state_pit);

			UI_SendGraph(7,Yaw_Angle,Special_ACT_3,Special_ACT_2,Special_ACT_1,SC_Vol, Pitch_Angle,SC_Vol_Arc);
			
//			UI_SendGraph(1,Fric_Mode_1);

		}
		if(UI_cnt == 10)
		{
			UI_SendGraph(3, SC_Vol_N, SC_Vol_P, SC_Vol_R);
			UI_cnt = 0;
		}
		osDelay(2);
	}
}

/**
 * @brief 根据裁判系统信息变更ui发送ID
 * @note  Change send ID
 * @param 
 */
void UI_ID_Set(void)
{
	Robot_ID = robot_status.robot_id;
	Cilent_ID = Robot_ID + 0x0100;
}

/**
 * @brief 1--辅助瞄准
 * @note  auxiliary aim
 * @param 命名方式 第一位表示瞄准线类，第二位表示瞄准线分类（竖线1）（横线2），第三位从上到下，从左到右1234。。
 */
Graph_Data_t GUN_Sight_MainAxis;//瞄准镜中轴
Graph_Data_t GUN_Sight_LateralAxis_1;//横轴线 1
Graph_Data_t GUN_Sight_LateralAxis_2;//横轴线 2
Graph_Data_t GUN_Sight_LateralAxis_3;//横轴线 3
Graph_Data_t GUN_Sight_LateralAxis_4;
Graph_Data_t GUN_Sight_LateralAxis_5;
Graph_Data_t GUN_Sight_LateralAxis_6;
Graph_Data_t GUN_Sight_LateralAxis_7;
Graph_Data_t GUN_Sight_LateralAxis_8;
Graph_Data_t GUN_Sight_LateralAxis_9;
Graph_Data_t GUN_Sight_LateralAxis_10;


FloInt_Data_t GUN_Sight_MeterAxis_1;
FloInt_Data_t GUN_Sight_MeterAxis_2;
FloInt_Data_t GUN_Sight_MeterAxis_3;

Graph_Data_t miao_zhun_kuang;

void UI_Gun_Sight(void)
{
//	Line_Draw( &GUN_Sight_MainAxis, "111", 1, 0, 1, 1, 960, 600, 960, 0 );//操作1(添加), 图层0, 颜色1(黄色), 线宽2

//	Line_Draw( &GUN_Sight_LateralAxis_1, "121", 1, 0, 1, 1, 900, 420, 1020, 420 );//中心点960*540（x*y）	零点为屏幕左下角	4m环高下方
//	Line_Draw( &GUN_Sight_LateralAxis_2, "122", 1, 0, 1, 1, 925, 300, 995, 300 );//		8.2m狙击点
//	Line_Draw( &GUN_Sight_LateralAxis_3, "123", 1, 0, 1, 1, 940, 330, 980, 330 );//3m
//	Line_Draw( &GUN_Sight_LateralAxis_4, "124", 1, 0, 1, 1, 945, 275, 975, 275 );//4m
//	Line_Draw( &GUN_Sight_LateralAxis_5, "125", 1, 0, 1, 1, 900, 240, 1020, 240 );
//	Line_Draw( &GUN_Sight_LateralAxis_6, "126", 1, 0, 4, 1, 940, 360, 980, 360 );//身位姿态提醒
//	Line_Draw( &GUN_Sight_LateralAxis_7, "127", 1, 0, 4, 1, 940, 380, 980, 380 );//前哨站6m
//	Line_Draw( &GUN_Sight_LateralAxis_8, "128", 1, 0, 4, 1, 940, 319, 980, 319 );//
//	Line_Draw( &GUN_Sight_LateralAxis_9, "112", 1, 0, 4, 1, 925, 420, 925, 390 );//
//	Line_Draw( &GUN_Sight_LateralAxis_10,"113", 1, 0, 4, 1, 995, 420, 995, 390 );
	
	Rectangle_Draw(&miao_zhun_kuang,"121",1, 0,UI_Color_Green,1,926,465,984,416);
	Float_Draw(&GUN_Sight_MeterAxis_1,"met6",1, 0, 4, 10,1, 3, 1030, 390, 6 );
	Float_Draw(&GUN_Sight_MeterAxis_2,"met7",1, 1, 4, 10,1, 3, 1030 ,360 ,7 );
	Float_Draw(&GUN_Sight_MeterAxis_3,"met9",1, 2, 4, 10,1, 3, 1030 ,320 ,9 );
	
//	Line_Draw( &GUN_Sight_LateralAxis_10,"114", 1, 0, 2, 1, 995, 420, 995, 390 );
	
 
}

Graph_Data_t Run_Warning_1;
Graph_Data_t Run_Warning_2;
Graph_Data_t Run_Warning_3;
Graph_Data_t Run_Warning_4;
Graph_Data_t Run_Warning_5;
Graph_Data_t Run_Warning_6;
Graph_Data_t Run_Warning_7;
Graph_Data_t Run_Warning_8;
Graph_Data_t Run_Warning_9;

void UI_Run_Warning()
{
	if(robot_status.current_HP <= robot_status.maximum_HP *0.5  )
	{
			Line_Draw( &Run_Warning_1, "991", 1, 0, 4, 5,1520,750,1620, 750 );//
			Line_Draw( &Run_Warning_2, "992", 1, 0, 4, 5,1520,750,1520, 600 );//
			Line_Draw( &Run_Warning_3, "993", 1, 0, 4, 5,1520,600,1620, 600 );//
			Line_Draw( &Run_Warning_4, "994", 1, 0, 4, 5,1620,600,1620,670 );//
			Line_Draw( &Run_Warning_5, "995", 1, 0, 4, 5,1620,670,1550,670 );//
			
			Line_Draw( &Run_Warning_6, "996", 1, 0, 4, 5,1670,750,1770,750 );//
			Line_Draw( &Run_Warning_7, "997", 1, 0, 4, 5,1770,750,1770,600 );//
			Line_Draw( &Run_Warning_8, "998", 1, 0, 4, 5,1770,600,1670,600 );//
			Line_Draw( &Run_Warning_9, "999", 1, 0, 4, 5,1670,600,1670,750 );//
	}
	else
	{
			Line_Draw( &Run_Warning_1, "991", 3, 0, 4, 5,1520,750,1620, 750 );//
			Line_Draw( &Run_Warning_2, "992", 3, 0, 4, 5,1520,750,1520, 600 );//
			Line_Draw( &Run_Warning_3, "993", 3, 0, 4, 5,1520,600,1620, 600 );//
			Line_Draw( &Run_Warning_4, "994", 3, 0, 4, 5,1620,600,1620,670 );//
			Line_Draw( &Run_Warning_5, "995", 3, 0, 4, 5,1620,670,1550,670 );//
			
			Line_Draw( &Run_Warning_6, "996", 3, 0, 4, 5,1670,750,1770,750 );//
			Line_Draw( &Run_Warning_7, "997", 3, 0, 4, 5,1770,750,1770,600 );//
			Line_Draw( &Run_Warning_8, "998", 3, 0, 4, 5,1770,600,1670,600 );//
			Line_Draw( &Run_Warning_9, "999", 3, 0, 4, 5,1670,600,1670,750 );//
	}
		
}

Graph_Data_t Vision_Ready_mode;
void Vision_IF_Ready()
{	
	Rectangle_Draw(&Vision_Ready_mode,"731", 1 , 2 , 1 , 6 , 620 , 130 , 660 ,170 );
	if(Rx_Vision_Message.Vision_Flag_IF_Ready==1)
		Rectangle_Draw(&Vision_Ready_mode,"731", 2 , 2 , 2 , 6 , 620 , 130 , 660 ,170 );
	else if(Rx_Vision_Message.Vision_Flag_IF_Ready==2)
		Rectangle_Draw(&Vision_Ready_mode,"731", 2 , 2 , 1 , 6 , 620 , 130 , 660 ,170 );
	else
		Rectangle_Draw(&Vision_Ready_mode,"731", 2 , 2 , 3 , 6 , 620 , 130 , 660 ,170 );
	
}
/**
		视觉瞄准yaw、pitch的开关
**/
String_Data_t Vision_state_yaw;
String_Data_t Vision_state_pit;
void UI_Vision_state(void)
{
	Char_Draw(&Vision_state_yaw, "bbb", 1,  1, 1, 5, 7, 1, 1750, 520, "Y");
	Char_Draw(&Vision_state_pit, "bcb", 1,  1, 1, 5, 7, 1, 1750, 480, "P");
	if(vision_yaw_state)
	Char_Draw(&Vision_state_yaw, "bbb", 2,  1, 1, 5, 7, 1, 1750, 520, "Y");
	else Char_Draw(&Vision_state_yaw, "bbb", 2,  1, 2, 5, 7, 1, 1750, 520, "Y");
	if(vision_pit_state)
	Char_Draw(&Vision_state_pit, "bcb", 2,  1, 1, 5, 7, 1, 1750, 480, "P");
	else Char_Draw(&Vision_state_pit, "bcb", 2,  1, 2, 5, 7, 1, 1750, 480, "P");
}
/**
 * @brief 2--车身身位动态反馈
 * @note  
 * @param 
 */
//Graph_Data_t Hostile_Direction_1;//枪口指示
//Graph_Data_t Hostile_Direction_2;//左斜线"\"
//void UI_Hostile_Direction(void)
//{
//	Line_Draw( &Hostile_Direction_1, "201", 1, 1, 0, 5, 960, 70, 960 + 40*sin(delat), 70 + 40*cos(delat));
//	Line_Draw( &Hostile_Direction_1, "201", 2, 1, 0, 5, 960, 70, 960 + 40*sin(delat), 70 + 40*cos(delat));//图层1 主色0
//	Line_Draw( &Hostile_Direction_2, "202", 1, 1, 0, 5, 657, 864, 707, 814 );
//}
Graph_Data_t Hostile_Direction_1;//枪口指示
Graph_Data_t Hostile_Direction_2;//左斜线"\"
Graph_Data_t Car_Body;
void UI_Hostile_Direction(void)
{
	Line_Draw( &Hostile_Direction_1, "201", 1, 1, 0, 5, 1540 + 40*sin(RSv_t.ref), 380 + 40*cos(RSv_t.ref), 1540 + 70*sin(RSv_t.ref), 380 + 70*cos(RSv_t.ref));
	Line_Draw( &Hostile_Direction_1, "201", 2, 1, 0, 5, 1540 + 40*sin(RSv_t.ref), 380 + 40*cos(RSv_t.ref), 1540 + 70*sin(RSv_t.ref), 380 + 70*cos(RSv_t.ref));//图层1 主色0
	Circle_Draw( &Car_Body, "CB1", 1, 1, 0, 1, 1540, 380, 40);
	// Line_Draw( &Hostile_Direction_2, "202", 1, 1, 0, 5, 657, 864, 707, 814 );
}
/**
 * @brief 3--防撞提示
 * @note  
 * @param 命名方式 第一位表示防撞提示类，第二位表示...
 */
Graph_Data_t Anticollition_1;//防撞提示-横线
Graph_Data_t Anticollition_2;//左斜"\"
Graph_Data_t Anticollition_3;//右斜"/"
//中心点960*540（x*y）	零点为屏幕左下角
void UI_Anticollition(void)
{
//	Line_Draw( &Anticollition_1, "123", 1, 2, 1, 2, 900, 380, 1020, 380 );
	Line_Draw( &Anticollition_1, "311", 1, 2, 3, 2, 500, 265, 1350, 265 );//操作1(添加), 图层2, 颜色1(黄色), 线宽2
	Line_Draw( &Anticollition_2, "321", 1, 2, 3, 2,465,200 , 500, 265 );
	Line_Draw( &Anticollition_3, "322", 1, 2, 3, 2, 1460, 0, 1350, 265 );
}

/**
 * @brief 电容电压显示
 * @note  display SuperCap voltage
 * @param 
 */

FloInt_Data_t SC_Vol;
Graph_Data_t SC_Vol_R;
Graph_Data_t SC_Vol_P;
FloInt_Data_t SC_Vol_N;
float C_Vol_Last;
void UI_Cap_Voltage(void)
{
		Float_Draw(&SC_Vol, "abc", 1, 6, 6, 40, 2, 3, 200, 700, SuperCAP.C_Vol);//图层6 字号40 线宽3 颜色6(青色)
		Float_Draw(&SC_Vol_N, "svn", 1, 6, 6, 20, 2, 3, 1300, 100, 100*(SuperCAP.C_Vol-15)/10);
		// Rectangle_Draw(&SC_Vol_R, "svr", 1, 6, 7, 1, 640, 100, 1280, 120);
		Line_Draw(&SC_Vol_P, "svp", 1, 6, 2, 18, 641, 100, (uint32_t)(640+(640*(SuperCAP.C_Vol-15)/10)), 100);
		if((SuperCAP.C_Vol-C_Vol_Last)<=0.2f || (SuperCAP.C_Vol-C_Vol_Last)>=-0.2f){
			Float_Draw(&SC_Vol, "abc", 2, 6, 6, 40, 2, 3, 200, 700, SuperCAP.C_Vol);//修改
			Float_Draw(&SC_Vol_N, "svn", 2, 6, 6, 20, 2, 3, 1300, 100, 100*(SuperCAP.C_Vol-15)/10);
    if(SuperCAP.C_Vol - 15 > 0)
			Line_Draw(&SC_Vol_P, "svp", 2, 6, 2, 18, 641, 100, (uint32_t)(640+(640*(SuperCAP.C_Vol-15)/10)), 100);
		else
			Line_Draw(&SC_Vol_P, "svp", 2, 6, 2, 18, 641, 100, 0, 100);
		}
		C_Vol_Last = SuperCAP.C_Vol;
}
/**
 * @brief Pitch轴角度及摩擦轮转速
 * @note  display Pitch Angle
 * @param 
 */
FloInt_Data_t Pitch_Angle;
FloInt_Data_t Fricmotor_Speed;
double Pitch_Last;
void UI_Pitch_Angle(void)
{
	uint8_t decision = 1;
	if(fabs(pitch - Rx_Vision_Message.Vision_Pitch_Angle ) < 0.3)
		decision = 2;
	Float_Draw(&Pitch_Angle, "acb", 1, 7, 3, 15, 1, 3, 1050, 540, -pitch);//图层7 字号15 小数位数1 线宽3 颜色3(橙色)
	Float_Draw(&Fricmotor_Speed, "adb", 1, 7, 1, 30, 1, 3, 200, 630, Shoot.Friction.friciont_num_target);//图层7 字号15 小数位数1 线宽3 颜色3(橙色)
	if((pitch-Pitch_Last)<=0.1 || (pitch-Pitch_Last)>=-0.1)
		Float_Draw(&Pitch_Angle, "acb", 2, 7, decision, 15, 1, 3, 1050, 540, -pitch);//有变化则修改
		Float_Draw(&Fricmotor_Speed, "adb", 2, 7, 1, 30, 1, 3, 200, 630, Shoot.Friction.friciont_num_target);

	Pitch_Last = pitch;
}

FloInt_Data_t Yaw_Angle;
double Yaw_Last;
void UI_Yaw_Angle(void)
{
	 	uint8_t decision = 1;

	Float_Draw(&Yaw_Angle, "ayb", 1, 6, 3, 15, 2, 3, 1250, 250, yaw );//图层7 字号15 小数位数1 线宽3 颜色3(橙色)
	if((yaw-Yaw_Last )<=0.1 || (yaw-Yaw_Last )>=-0.1)
		Float_Draw(&Yaw_Angle, "ayb", 2, 6, decision, 15, 2, 3, 1250, 250,yaw);//有变化则修改
  Yaw_Last = yaw;
}

/**
 * @brief 刷新UI
 * @note  防止浮点型在UI清除之后不显示
 * @param 
 */
int p=0;
void UI_Reset(void)
{
	if(rc_ctrl.keyboard.key_E == 1)
	{
		p++;
		//静态UI创建操作
		UI_Gun_Sight();			//瞄准镜
		UI_Anticollition();		//防撞提示
		UI_Hostile_Direction();	//敌方提示
		UI_SendGraph(7, GUN_Sight_MainAxis, GUN_Sight_LateralAxis_1, GUN_Sight_LateralAxis_2, 
	 	GUN_Sight_LateralAxis_3, GUN_Sight_LateralAxis_4, Anticollition_1, Anticollition_3);
		UI_SendGraph(7,Anticollition_1,Anticollition_2,Anticollition_3,Run_Warning_1,Run_Warning_2,Run_Warning_3,Run_Warning_4);
		UI_SendGraph(7,Run_Warning_5,Run_Warning_6,Run_Warning_7,Run_Warning_8,Run_Warning_9,Vision_Ready_mode,Gimbal_connect_status);
		
		HAL_Delay(1);
		//动态UI创建操作
		Arc_Draw(&SC_Outline_Arc_1, "711", 1, 6, 1, 1, 45, 135, 960, 540, 301, 301);//操作1 创建，图层6，颜色1黄色，线宽1，起始角度，终点角度，圆心（960，540）,xy半径
		Arc_Draw(&SC_Outline_Arc_2, "712", 1, 6, 1, 1, 45, 135, 960, 540, 295, 295);//操作1 创建，图层6，颜色1黄色，线宽1，起始角度，终点角度，圆心（960，540）,xy半径
		Line_Draw(&SC_Outline_Line_1, "713", 1, 6, 1, 1, 1168, (1080-331), 1173, (1080-326) );//操作1 创建，图层6，颜色1黄色，线宽1，
		Line_Draw(&SC_Outline_Line_2, "714", 1, 6, 1, 1, 1168, (1080-747), 1173, (1080-753) );//操作1 创建，图层6，颜色1黄色，线宽1，
		Arc_Draw(&SC_Vol_Arc, "715", 1, 6, 5, 5, 45, 135, 960, 540, 298, 298);//操作1 创建，图层6，颜色5粉色，线宽5，起始角度，终点角度，圆心（960，540）,xy半径
		
		UI_SendGraph( 7, visual_field_1, visual_field_2, visual_field_3, visual_field_4, SC_Outline_Arc_1 ,SC_Outline_Arc_2, SC_Outline_Line_1);
		
		
		Float_Draw(&SC_Vol, "abc", 1, 6, 6, 40, 2, 3, 200, 700, SuperCAP.C_Vol);//图层6 字号40 线宽3 颜色6(青色)
		
		Float_Draw(&Pitch_Angle, "acb", 1, 7, 3, 15, 1, 3, 1050, 540, pitch);//图层7 字号15 小数位数1 线宽3 颜色3(橙色)
		
		Float_Draw(&Yaw_Angle, "ayb", 1, 6, 3, 15, 2, 3, 1250, 250, yaw );//图层7 字号15 小数位数1 线宽3 颜色3(橙色)
		
		Float_Draw(&Fricmotor_Speed, "adb", 1, 7, 1, 30, 1, 3, 200, 630, Shoot.Friction.friciont_num_target);
		
		Circle_Draw(&Action_Mode_1, "411", 1, 3, 1, 8, 800, 150, 15);//操作1 添加，图层3，颜色1黄色，线宽8，, 半径5
		
		Circle_Draw(&Fric_Mode_1, "511", 1, 4, 1, 8, 1113, 146, 15);//操作1 添加，图层4，颜色1黄色，线宽8，, 半径5
		
		Circle_Draw(&Shoot_Mode_1, "611", 1, 5, 1, 8, 1120, 150, 15);//操作1 添加，图层6，颜色1黄色，线宽8，, 半径5
		
		Circle_Draw(&Vision_Outpost_Mode, "811", 1, 6, 1, 8, 1280, 150, 15);//操作1 添加，图层6，颜色1黄色，线宽8，, 半径5
		
		Line_Draw( &Hostile_Direction_1, "201", 1, 1, 0, 5, 960, 70, 960 + 40*sin(RSv_t.ref), 70 + 40*cos(RSv_t.ref));
		
		Rectangle_Draw(&Vision_Ready_mode,"731", 1 , 2 , 1 , 6 , 620 , 130 , 660 ,170 );

		Circle_Draw(&Gimbal_connect_status, "541", 1, 6, 1, 8, 960, 100, 15);//操作1 添加，图层6，颜色1黄色，线宽8，, 半径5
		
		Float_Draw(&SC_Vol, "abc", 1, 6, 6, 40, 2, 3, 200, 700, SuperCAP.C_Vol);//图层6 字号40 线宽3 颜色6(青色)
		Float_Draw(&SC_Vol_N, "svn", 1, 6, 6, 3, 2, 3, 1300, 100, 100*(SuperCAP.C_Vol-15)/10);
		// Rectangle_Draw(&SC_Vol_R, "svr", 1, 6, 7, 1, 640, 100, 1280, 120);
		Line_Draw(&SC_Vol_P, "svp", 1, 6, 2, 2, 641, 100, (uint32_t)(640+(640*(SuperCAP.C_Vol-15)/10)), 100);
		
		Line_Draw(&Special_ACT_1, "911", 1, 1, 0, 6, 462, 330, 462, 370 );	//操作1 添加，图层1，颜色0主色，线宽4
		Circle_Draw(&Special_ACT_2, "912", 1, 1, 2, 2, 462, 295, 35);			//操作1 添加，图层1，颜色2绿色，线宽2，半径35
		Arc_Draw(&Special_ACT_3, "913", 1, 1, 0, 6, 135, 225, 462, 295, 35, 35 );	//操作1 添加，图层1，颜色0主色，线宽2，半径35
	}
}

/**
 * @brief 底盘运动模式显示
 * @note  display Action Mode
 * @param 
 */
Graph_Data_t Action_Mode_1;
void UI_Action_Mode(void)
{
	Circle_Draw(&Action_Mode_1, "411", 1, 3, 1, 8, 460, 150, 15);//操作1 添加，图层3，颜色1黄色，线宽8，, 半径5
	if(System.action_mode == SYS_ACT_FOLLOW)
		Circle_Draw(&Action_Mode_1, "411", 2, 3, 3, 8, 800, 150, 15);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
	if(System.action_mode == SYS_ACT_STAY)
		Circle_Draw(&Action_Mode_1, "411", 2, 3, 2, 8, 800, 150, 15);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
	if(System.action_mode == SYS_ACT_SPIRAL)
		Circle_Draw(&Action_Mode_1, "411", 2, 3, 6, 8, 800, 150, 15);//操作2 修改，图层3，颜色6青色，线宽8，, 半径5
}

/**
 * @brief 摩擦轮显示
 * @note  display Friction
 * @param 
 */
Graph_Data_t Fric_Mode_1;
void UI_Fric_Mode(void)
{
	Circle_Draw(&Fric_Mode_1, "511", 1, 4, 1, 8, 1113, 146, 15);//操作1 添加，图层4，颜色1黄色，线宽8，, 半径5
	if(fabs(Shoot.Friction.moto_Fric[0].speed_rpm) > 3000)								//摩擦轮开启
		Circle_Draw(&Fric_Mode_1, "511", 2, 4, 2, 8, 1113, 146, 15);//操作2 修改，图层4，颜色2绿色，线宽8，, 半径5
	if(fabs(Shoot.Friction.moto_Fric[0].speed_rpm) <= 3000)
		Circle_Draw(&Fric_Mode_1, "511", 2, 4, 3, 8,1113, 146, 15);//操作2 修改，图层4，颜色3橙色，线宽8，, 半径5
}

/**
 * @brief 发弹显示
 * @note  display Shoot
 * @param 
 */
Graph_Data_t Shoot_Mode_1;
void UI_Shoot_Mode(void)
{
		Circle_Draw(&Shoot_Mode_1, "611", 1, 5, 1, 8, 1120, 150, 15);//操作1 添加，图层6，颜色1黄色，线宽8，, 半径5
	if(change_Z>0&&come>480)						
		Circle_Draw(&Shoot_Mode_1, "611", 2, 5, 2, 8, 1120, 150, 15);//操作2 修改，图层6，颜色2绿色，线宽8，, 半径5
	else if(Vision_Data.VisionMode == CONST__OUTPOST)						
		Circle_Draw(&Shoot_Mode_1, "611", 2, 5, 6, 8, 1120, 150, 15);//操作2 修改，图层6，颜色6青色，线宽8，, 半径5
	else 
		Circle_Draw(&Shoot_Mode_1, "611", 2, 5, 3, 8, 1120, 150, 15);//操作2 修改，图层6，颜色6橙色，线宽8，, 半径5
}


Graph_Data_t Vision_Outpost_Mode;
void UI_Vision_Outpost_Mode()//视觉允许发弹
{
		Circle_Draw(&Vision_Outpost_Mode, "811", 1, 6, 1, 8, 1280, 150, 15);//操作1 添加，图层6，颜色1黄色，线宽8，, 半径5
	if(Rx_Vision_Message.Vision_Flag_Fire==1)	
		Circle_Draw(&Vision_Outpost_Mode, "811", 2, 6, 5, 8, 1280, 150, 15);//操作2 修改，图层6，颜色2绿色，线宽8，, 半径5
	else 
		Circle_Draw(&Vision_Outpost_Mode, "811", 2, 6, 3, 8, 1280, 150, 15);//操作2 修改，图层6，颜色6橙色，线宽8，, 半径5
}

Graph_Data_t Gimbal_connect_status;
void Gimbal_status_mode()
{
	Circle_Draw(&Gimbal_connect_status, "541", 1, 6, 1, 8, 960, 100, 15);//操作1 添加，图层6，颜色1黄色，线宽8，, 半径5
	if(toe_is_error(PITCH_GIMBAL_MOTOR_TOE)==true||toe_is_error(PITCH_GIMBAL_MOTOR_TOE)==true)	
		Circle_Draw(&Gimbal_connect_status, "541", 2, 6, 7, 8, 960, 100, 15);//操作2 修改，图层6，颜色2绿色，线宽8，, 半径5
	else 
		Circle_Draw(&Gimbal_connect_status, "541", 2, 6, 3, 8, 960, 100, 15);//操作2 修改，图层6，颜色6橙色，线宽8，, 半径5
}


Graph_Data_t SC_Outline_Arc_1;
Graph_Data_t SC_Outline_Arc_2;
Graph_Data_t SC_Outline_Line_1;
Graph_Data_t SC_Outline_Line_2;
Graph_Data_t SC_Vol_Arc;
float CAP_Vol_Angle;
void UI_Cap(void)
{
	CAP_Vol_Angle = 135.0f - (SuperCAP.C_Vol-8)/12.0f*90.0f;
	if(CAP_Vol_Angle>=134)
		CAP_Vol_Angle = 134;
	if(SuperCAP.C_Vol>=15)
		Arc_Draw(&SC_Vol_Arc, "715", 2, 6, 2, 5, CAP_Vol_Angle, 135, 960, 540, 298, 298);//操作2 更新，图层6，颜色2绿色，线宽5，起始角度，终点角度，圆心（960，540）,xy半径
	if(SuperCAP.C_Vol<15)
		Arc_Draw(&SC_Vol_Arc, "715", 2, 6, 5, 5, CAP_Vol_Angle, 135, 960, 540, 298, 298);//操作2 更新，图层6，颜色5粉色，线宽5，起始角度，终点角度，圆心（960，540）,xy半径
}


Graph_Data_t visual_field_1;
Graph_Data_t visual_field_2;
Graph_Data_t visual_field_3;
Graph_Data_t visual_field_4;
Graph_Data_t Vision_Mode_1;	//动态
void UI_Vision_Mode(void)
{
//	if(Vision_Data.VisionMode == CONST__SMALL_MECHANISM)
//		Circle_Draw(&Vision_Mode_1, "211", 2, 5, 2, 8, 1125, 150, 15);//操作2 修改，图层5，颜色2绿色，线宽8，, 半径5	//大符模式
//	if(Vision_Data.VisionMode == CONST__BIG_MECHANISM)
////		Circle_Draw(&Vision_Mode_1, "211", 2, 5, 6, 8, 1125, 150, 15);//操作2 修改，图层5，颜色6青色，线宽8，, 半径5	//小符模式
//	if(Vision_Data.VisionMode == CONST__ARMOR_PLATE)
//		Circle_Draw(&Vision_Mode_1, "211", 2, 5, 3, 8, 1125, 150, 15);//操作2 修改，图层5，颜色3橙色，线宽8，, 半径5	//装甲板模式
//	if(Vision_Data.VisionMode == CONST__OFF)				
//		Circle_Draw(&Vision_Mode_1, "211", 2, 5, 7, 8, 1125, 150, 15);//操作2 修改，图层5，颜色7黑色，线宽8，, 半径5	//关闭
	
//	//视觉识别区域，根据实测
//	Line_Draw( &visual_field_1, "212", 1, 5, 8, 1, 651, 715, 1343, 715 );//操作1(添加), 图层5, 颜色8(白色), 线宽2
//	Line_Draw( &visual_field_2, "213", 1, 5, 8, 1, 1343, 715, 1343, 250 );//操作1(添加), 图层5, 颜色8(白色), 线宽2
//	Line_Draw( &visual_field_3, "214", 1, 5, 8, 1, 1343, 250, 651, 250 );//操作1(添加), 图层5, 颜色8(白色), 线宽2
//	Line_Draw( &visual_field_4, "215", 1, 5, 8, 1, 651, 250, 651, 715 );//操作1(添加), 图层5, 颜色8(白色), 线宽2
}

/**
 * @brief 特殊运动模式显示
 * @note  Special ACT
 * @param 
 */
Graph_Data_t Special_ACT_1;//云台朝向
Graph_Data_t Special_ACT_2;//边框圆
Graph_Data_t Special_ACT_3;//灯条1/4圆
//以（1350，230）为圆心
void UI_Special_ACT(void)
{
	int angle_1;
	int angle_2;
	float ref;

	if (YAW_moto[0].Position <= SPECIAL_ACT_FRONT_ECD) 
	{
		ref = (float)(SPECIAL_ACT_FRONT_ECD - YAW_moto[0].Position) /pi  * 180;
	}
	else if (YAW_moto[0].Position > SPECIAL_ACT_FRONT_ECD)
	{
		ref = (float)(SPECIAL_ACT_FRONT_ECD + 2*pi - YAW_moto[0].Position) / pi* 180;
	}

	if (ref >= 180 && ref <= 360) // yaw_diff计算
	{
		ref = ref - 360;
	}
	else if (ref < 180 && ref >= 0)
	{
		ref = ref;
	}
	
	if(ref>=0&&ref<135){// 0<=ref<=180 时
		angle_1 = -ref+135;
		angle_2 = angle_1+90;
		Arc_Draw(&Special_ACT_3, "913", 2, 1, 0, 6, angle_1, angle_2, 462, 295, 35, 35 );	//操作2 更改，图层1，颜色0主色，线宽6，半径35
	}
	if(ref>=135&&ref<=180){// 0<=ref<=180 时
		angle_1 = -ref+45+15;
		angle_2 = angle_1-90;
		Arc_Draw(&Special_ACT_3, "913", 2, 1, 0, 6, angle_2, angle_1, 462, 295, 35, 35 );	//操作2 更改，图层1，颜色0主色，线宽6，半径35
	}
	if(ref<0&&ref>=-180){// -180<=ref<0 时
		ref = ref;
		angle_1 = -ref+135;
		angle_2 = angle_1+90;
		Arc_Draw(&Special_ACT_3, "913", 2, 1, 0, 6, angle_1, angle_2, 462, 295, 35, 35 );	//操作2 更改，图层1，颜色0主色，线宽6，半径35
	}

}


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

///*************
//添加车身位置
//pitch轴视觉角度校准




//车身校准占一位

//*************/
//#include "Steering_UI.h"
//#include "bsp_can.h"
//#include "referee.h"
//#include "INS_task.h"
//#include "Nautilus_Vision.h"
//#include "remote_control.h"
//#include "resolve.h"
//#include "shoot.h"
//#include "trigger.h"

//#include "math.h"
//#include "cmsis_os.h"

///**
// * @brief UI发送总函数
// * @note  UI send
// * @param 
// */
//int UI_cnt = 0;
//int cou = 1;
//void UI_Task(void)
//{
//	while(1)
//	{
//	UI_cnt++;
//	UI_ID_Set();
//	if(UI_cnt == 10)						//取值自行根据任务帧率调整
//	{
//		cou++;
//		UI_Gun_Sight();						//瞄准镜
////		UI_Cap_Voltage();					//电容电压
//		UI_Pitch_Angle();					//pitch轴角度
//		UI_Sentry_Angle(100.1, roll, 100.3, 100.4, 100.5, 100.6, 100.7);			//
//		UI_Reset();							//放在浮点型之后运行
//		UI_SendGraph(7, Sentry_Angle_1, GUN_Sight_LateralAxis_1, GUN_Sight_LateralAxis_2, GUN_Sight_LateralAxis_3, GUN_Sight_LateralAxis_4, GUN_Sight_LateralAxis_5, GUN_Sight_LateralAxis_6);
//		UI_SendGraph(7,GUN_Sight_LateralAxis_7,GUN_Sight_LateralAxis_8,GUN_Sight_LateralAxis_9,GUN_Sight_MeterAxis_1,GUN_Sight_LateralAxis_10,GUN_Sight_MeterAxis_2,GUN_Sight_MeterAxis_3);

//	}
//	else if(UI_cnt == 20)
//	{
//		UI_Reset();							//放在浮点型之后运行
//		UI_SendGraph(7, Pitch_Angle, Sentry_Angle_2, Sentry_Angle_3, Sentry_Angle_4, Sentry_Angle_5, Sentry_Angle_6, Sentry_Angle_7);
//	}
//	else if(UI_cnt == 30)
//	{
//		UI_MyRobot_Mode(cou/4);				//填入我方跟随机器人id
//		UI_Data_Disp(UI_cnt, cou);			//下方两个整形变量填入
//		UI_Shoot_Mode(1);
//		UI_Reset();							//放在浮点型之后运行
//		UI_SendGraph(7, Robot_StateID3, Robot_StateID4, Robot_StateID5, Int_Data1, Int_Data2, GUN_Sight_MainAxis, Shoot_Mode_1);
//		UI_SendGraph(7, MyRobot_StateID1, MyRobot_StateID2, MyRobot_StateID3, MyRobot_StateID4 ,MyRobot_StateID5, Robot_StateID1, Robot_StateID2);
//	}
//	else if(UI_cnt == 40)
//	{	
//		if(cou == 10)
//		{
//			UI_Robot1_Mode(1);				//敌方机器人目标标记圈，1为标记0为忽视
//			UI_Robot2_Mode(1);
//			UI_Robot3_Mode(1);
//			UI_Robot4_Mode(1);
//			UI_Robot5_Mode(1);
//			UI_Robot7_Mode(1);
//			UI_Robot10_Mode(1);
//		}
//		else if(cou == 20)
//		{
//			cou = 0;
//			UI_Robot1_Mode(0);
//			UI_Robot2_Mode(0);
//			UI_Robot3_Mode(0);
//			UI_Robot4_Mode(0);
//			UI_Robot5_Mode(0);
//			UI_Robot7_Mode(0);
//			UI_Robot10_Mode(0);
//		}
//		UI_Reset();							//放在浮点型之后运行
//		UI_SendGraph(7, MyRobot_Mode_1, MyRobot_Mode_2, MyRobot_Mode_3, MyRobot_Mode_4, MyRobot_Mode_5, MyRobot_Mode_7, MyRobot_Mode_10);
//		UI_SendGraph(7, Robot_Mode_1, Robot_Mode_2, Robot_Mode_3, Robot_Mode_4, Robot_Mode_5, Robot_Mode_7, Robot_Mode_10);
//		UI_cnt = 0;
//	}
//	}
//}

///**
// * @brief 根据裁判系统信息变更ui发送ID
// * @note  Change send ID
// * @param 
// */
//void UI_ID_Set(void)
//{
//	Robot_ID = robot_state.robot_id;
//	Cilent_ID = Robot_ID + 0x0100;
//}

///**
// * @brief 1--辅助瞄准
// * @note  auxiliary aim
// * @param 命名方式 第一位表示瞄准线类，第二位表示瞄准线分类（竖线1）（横线2），第三位从上到下，从左到右1234。。
// */
//Graph_Data_t GUN_Sight_MainAxis;//瞄准镜中轴
//Graph_Data_t GUN_Sight_LateralAxis_1;//横轴线 1
//Graph_Data_t GUN_Sight_LateralAxis_2;//横轴线 2
//Graph_Data_t GUN_Sight_LateralAxis_3;//横轴线 3
//Graph_Data_t GUN_Sight_LateralAxis_4;
//Graph_Data_t GUN_Sight_LateralAxis_5;
//Graph_Data_t GUN_Sight_LateralAxis_6;
//Graph_Data_t GUN_Sight_LateralAxis_7;
//Graph_Data_t GUN_Sight_LateralAxis_8;
//Graph_Data_t GUN_Sight_LateralAxis_9;
//Graph_Data_t GUN_Sight_LateralAxis_10;

//FloInt_Data_t GUN_Sight_MeterAxis_1;
//FloInt_Data_t GUN_Sight_MeterAxis_2;
//FloInt_Data_t GUN_Sight_MeterAxis_3;


//void UI_Gun_Sight(void)
//{
//	Line_Draw( &GUN_Sight_MainAxis, "111", 1, 0, 1, 1, 960, 600, 960, 0 );//操作1(添加), 图层0, 颜色1(黄色), 线宽2
//////	Line_Draw( &GUN_Sight_LateralAxis_1, "121", 1, 0, 1, 2, 900, 380, 1020, 380 );//中心点960*540（x*y）	零点为屏幕左下角	1m
//////	Line_Draw( &GUN_Sight_LateralAxis_2, "122", 1, 0, 1, 2, 925, 365, 995, 365 );//2m
//////	Line_Draw( &GUN_Sight_LateralAxis_3, "123", 1, 0, 1, 2, 940, 330, 980, 330 );//3m
//////	Line_Draw( &GUN_Sight_LateralAxis_4, "124", 1, 0, 1, 2, 945, 275, 975, 275 );//4m
//	Line_Draw( &GUN_Sight_LateralAxis_1, "121", 1, 0, 1, 1, 900, 420, 1020, 420 );//中心点960*540（x*y）	零点为屏幕左下角	4m环高下方
//	Line_Draw( &GUN_Sight_LateralAxis_2, "122", 1, 0, 1, 1, 925, 300, 995, 300 );//		8.2m狙击点
//	Line_Draw( &GUN_Sight_LateralAxis_3, "123", 1, 0, 1, 1, 940, 330, 980, 330 );//3m
//	Line_Draw( &GUN_Sight_LateralAxis_4, "124", 1, 0, 1, 1, 945, 275, 975, 275 );//4m
//	Line_Draw( &GUN_Sight_LateralAxis_5, "125", 1, 0, 1, 1, 900, 240, 1020, 240 );
//	Line_Draw( &GUN_Sight_LateralAxis_6, "126", 1, 0, 4, 1, 940, 360, 980, 360 );//身位姿态提醒
//	Line_Draw( &GUN_Sight_LateralAxis_7, "127", 1, 0, 4, 1, 940, 380, 980, 380 );//前哨站6m
//	Line_Draw( &GUN_Sight_LateralAxis_8, "128", 1, 0, 4, 1, 940, 319, 980, 319 );//
//	Line_Draw( &GUN_Sight_LateralAxis_9, "112", 1, 0, 4, 1, 925, 420, 925, 390 );//
//	Line_Draw( &GUN_Sight_LateralAxis_10,"113", 1, 0, 4, 1, 995, 420, 995, 390 );
//	
//	Float_Draw(&GUN_Sight_MeterAxis_1,"met6",1, 0, 4, 10,1, 3, 1030, 390, 6 );
//	Float_Draw(&GUN_Sight_MeterAxis_2,"met7",1, 1, 4, 10,1, 3, 1030 ,360 ,7 );
//	Float_Draw(&GUN_Sight_MeterAxis_3,"met9",1, 2, 4, 10,1, 3, 1030 ,320 ,9 );


//	
//}

///**
// * @brief 电容电压显示
// * @note  display SuperCap voltage
// * @param 
// */
////FloInt_Data_t SC_Vol;
////float C_Vol_Last = 0;
////void UI_Cap_Voltage(void)
////{
////		Float_Draw(&SC_Vol, "abc", 1, 6, 6, 40, 2, 3, 200, 700, C_Vol);//图层6 字号40 线宽3 颜色6(青色)
////		if((C_Vol-C_Vol_Last)<=0.2f || (C_Vol-C_Vol_Last)>=-0.2f){
////			Float_Draw(&SC_Vol, "abc", 2, 6, 6, 40, 2, 3, 200, 700, C_Vol);//修改
////		}
////		C_Vol_Last = C_Vol;
////} 

///**
// * @brief Pitch轴角度及摩擦轮转速
// * @note  display Pitch Angle
// * @param 
// */
//FloInt_Data_t Pitch_Angle;
//FloInt_Data_t Fricmotor_Speed;
//double Pitch_Last;
//void UI_Pitch_Angle(void)
//{
//	uint8_t decision = 1;
//	if(fabs(pitch - Rx_Vision_Message.Vision_Pitch_Angle ) < 0.3)
//		decision = 2;
//	Float_Draw(&Pitch_Angle, "700", 1, 7, 3, 15, 1, 3, 1050, 540, -pitch);//图层7 字号15 小数位数1 线宽3 颜色3(橙色)
//	Float_Draw(&Fricmotor_Speed, "adb", 1, 7, 1, 30, 1, 3, 200, 630, Shoot.Friction.L_friciont_num_target);//图层7 字号15 小数位数1 线宽3 颜色3(橙色)
//	if((pitch-Pitch_Last)<=0.1 || (pitch-Pitch_Last)>=-0.1)
//		Float_Draw(&Pitch_Angle, "700", 2, 7, decision, 15, 1, 3, 1050, 540, -pitch);//有变化则修改
//		Float_Draw(&Fricmotor_Speed, "adb", 2, 7, 1, 30, 1, 3, 200, 630, Shoot.Friction.L_friciont_num_target);

//	Pitch_Last = pitch;
//}

///**
// * @brief 两个整型数据展示及双方机器人状态圆圈ID显示
// * @note  
// * @param 
// */
//FloInt_Data_t Int_Data1;
//FloInt_Data_t Int_Data2;
//FloInt_Data_t MyRobot_StateID1;
//FloInt_Data_t MyRobot_StateID2;
//FloInt_Data_t MyRobot_StateID3;
//FloInt_Data_t MyRobot_StateID4;
//FloInt_Data_t MyRobot_StateID5;
//FloInt_Data_t Robot_StateID1;
//FloInt_Data_t Robot_StateID2;
//FloInt_Data_t Robot_StateID3;
//FloInt_Data_t Robot_StateID4;
//FloInt_Data_t Robot_StateID5;


//void UI_Data_Disp(int data1, int data2)
//{
//	Int32_Draw(&Int_Data1, "200", 1, 7, 1, 30, 8, 700, 880, 1);//图层7 字号15 小数位数1 线宽3 颜色3(橙色)
//	Int32_Draw(&Int_Data2, "201", 1, 7, 1, 30, 8, 1000, 880, 1);//图层7 字号15 小数位数1 线宽3 颜色3(橙色)
//	Int32_Draw(&MyRobot_StateID1, "202", 1, 7, 1, 30, 1, ROB1_LOCAT, ROBOT_LOC_Y - 50, 0);
//	Int32_Draw(&MyRobot_StateID2, "203", 1, 7, 1, 30, 1, ROB2_LOCAT, ROBOT_LOC_Y - 50, 1);
//	Int32_Draw(&MyRobot_StateID3, "204", 1, 7, 1, 30, 1, ROB3_LOCAT, ROBOT_LOC_Y - 50, 2);
//	Int32_Draw(&MyRobot_StateID4, "205", 1, 7, 1, 30, 1, ROB4_LOCAT, ROBOT_LOC_Y - 50, 3);
//	Int32_Draw(&MyRobot_StateID5, "206", 1, 7, 1, 30, 1, ROB5_LOCAT, ROBOT_LOC_Y - 50, 4);
//	Int32_Draw(&Robot_StateID1, "207", 1, 7, 1, 30, 1, 1920 - ROB1_LOCAT, ROBOT_LOC_Y - 50, 0);
//	Int32_Draw(&Robot_StateID2, "208", 1, 7, 1, 30, 1, 1920 - ROB2_LOCAT, ROBOT_LOC_Y - 50, 1);
//	Int32_Draw(&Robot_StateID3, "209", 1, 7, 1, 30, 1, 1920 - ROB3_LOCAT, ROBOT_LOC_Y - 50, 2);
//	Int32_Draw(&Robot_StateID4, "210", 1, 7, 1, 30, 1, 1920 - ROB4_LOCAT, ROBOT_LOC_Y - 50, 3);
//	Int32_Draw(&Robot_StateID5, "211", 1, 7, 1, 30, 1, 1920 - ROB5_LOCAT, ROBOT_LOC_Y - 50, 4);
//	Int32_Draw(&Int_Data1, "200", 2, 7, 1, 45, 8, 800, 200, data1);//图层7 字号15 小数位数1 线宽3 颜色3(橙色)
//	Int32_Draw(&Int_Data2, "201", 2, 7, 1, 45, 8, 1000, 200, data2);//图层7 字号15 小数位数1 线宽3 颜色3(橙色)
//}

///**
// * @brief 哨兵各角度数据显示
// * @note  display Sentry Angle,一组七个角度值，以第一个角度值的坐标为角度组左上角位置，两个一排呈四列
//位置示例：	1	2
//			3	4
//			5	6
//			7
// * @param 
// */
//FloInt_Data_t Sentry_Angle_1;
//FloInt_Data_t Sentry_Angle_2;
//FloInt_Data_t Sentry_Angle_3;
//FloInt_Data_t Sentry_Angle_4;
//FloInt_Data_t Sentry_Angle_5;
//FloInt_Data_t Sentry_Angle_6;
//FloInt_Data_t Sentry_Angle_7;
//void UI_Sentry_Angle(float Angle_1,float Angle_2,float Angle_3,float Angle_4,float Angle_5,float Angle_6,float Angle_7)
//{
//	uint16_t Start_X = 30, Start_Y= 1080 - 300;
//	Float_Draw(&Sentry_Angle_1, "701", 1, 7, 3, 18, 1, 3, Start_X + 0*FLOAT_DELTA_X, Start_Y - 0*FLOAT_DELTA_Y, 1);//图层7 字号15 小数位数1 线宽3 颜色3(橙色)
//	Float_Draw(&Sentry_Angle_2, "702", 1, 7, 3, 18, 1, 3, Start_X + 1*FLOAT_DELTA_X, Start_Y - 0*FLOAT_DELTA_Y, 1);//图层7 字号15 小数位数1 线宽3 颜色3(橙色)
//	Float_Draw(&Sentry_Angle_3, "703", 1, 7, 3, 18, 1, 3, Start_X + 0*FLOAT_DELTA_X, Start_Y - 1*FLOAT_DELTA_Y, 1);//图层7 字号15 小数位数1 线宽3 颜色3(橙色)
//	Float_Draw(&Sentry_Angle_4, "704", 1, 7, 3, 18, 1, 3, Start_X + 1*FLOAT_DELTA_X, Start_Y - 1*FLOAT_DELTA_Y, 1);//图层7 字号15 小数位数1 线宽3 颜色3(橙色)
//	Float_Draw(&Sentry_Angle_5, "705", 1, 7, 3, 18, 1, 3, Start_X + 0*FLOAT_DELTA_X, Start_Y - 2*FLOAT_DELTA_Y, 1);//图层7 字号15 小数位数1 线宽3 颜色3(橙色)
//	Float_Draw(&Sentry_Angle_6, "706", 1, 7, 3, 18, 1, 3, Start_X + 1*FLOAT_DELTA_X, Start_Y - 2*FLOAT_DELTA_Y, 1);//图层7 字号15 小数位数1 线宽3 颜色3(橙色)
//	Float_Draw(&Sentry_Angle_7, "707", 1, 7, 3, 18, 1, 3, Start_X + 0*FLOAT_DELTA_X, Start_Y - 3*FLOAT_DELTA_Y, 1);//图层7 字号15 小数位数1 线宽3 颜色3(橙色)
//	
//	Float_Draw(&Sentry_Angle_1, "701", 2, 7, 3, 18, 1, 3, Start_X + 0*FLOAT_DELTA_X, Start_Y - 0*FLOAT_DELTA_Y, Angle_1);//有变化则修改
//	Float_Draw(&Sentry_Angle_2, "702", 2, 7, 3, 18, 1, 3, Start_X + 1*FLOAT_DELTA_X, Start_Y - 0*FLOAT_DELTA_Y, Angle_2);//有变化则修改
//	Float_Draw(&Sentry_Angle_3, "703", 2, 7, 3, 18, 1, 3, Start_X + 0*FLOAT_DELTA_X, Start_Y - 1*FLOAT_DELTA_Y, Angle_3);//有变化则修改
//	Float_Draw(&Sentry_Angle_4, "704", 2, 7, 3, 18, 1, 3, Start_X + 1*FLOAT_DELTA_X, Start_Y - 1*FLOAT_DELTA_Y, Angle_4);//有变化则修改
//	Float_Draw(&Sentry_Angle_5, "705", 2, 7, 3, 18, 1, 3, Start_X + 0*FLOAT_DELTA_X, Start_Y - 2*FLOAT_DELTA_Y, Angle_5);//有变化则修改
//	Float_Draw(&Sentry_Angle_6, "706", 2, 7, 3, 18, 1, 3, Start_X + 1*FLOAT_DELTA_X, Start_Y - 2*FLOAT_DELTA_Y, Angle_6);//有变化则修改
//	Float_Draw(&Sentry_Angle_7, "707", 2, 7, 3, 18, 1, 3, Start_X + 0*FLOAT_DELTA_X, Start_Y - 3*FLOAT_DELTA_Y, Angle_7);//有变化则修改
//}
//	


///**
// * @brief 刷新UI
// * @note  防止浮点型在UI清除之后不显示
// * @param 
// */
//void UI_Reset(void)
//{
//	if(rc_ctrl.keyboard.key_E == 1)
//	{
//		UI_MyRobot_Mode(1);
//		UI_Robot1_Mode(1);
//		UI_Robot2_Mode(1);
//		UI_Robot3_Mode(1);
//		UI_Robot4_Mode(1);
//		UI_Robot5_Mode(1);
//		UI_Robot7_Mode(1);
//		UI_Robot10_Mode(1);
////		UI_Cap_Voltage();			//电容电压
//		UI_Pitch_Angle();			//pitch轴角度
//		UI_Data_Disp(UI_cnt, cou);	//两个整形数据显示
//		UI_Sentry_Angle(100.1, 100.2, 100.3, 100.4, 100.5, 100.6, 100.7);
//		UI_Gun_Sight();				//瞄准镜
//		UI_Shoot_Mode(1);
//		
//		
//		
//		
//		if(Robot_ID/10 == 0)		//红色方
//		{
//			Circle_Draw(&Robot_Mode_1, "121", 1, 3, 1, ROBOT_WIDE, 1920 - ROB1_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作1 添加，图层3，颜色1黄色，线宽8，, 半径5
//			
//			Circle_Draw(&Robot_Mode_2, "131", 1, 3, 1, ROBOT_WIDE, 1920 - ROB2_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作1 添加，图层3，颜色1黄色，线宽8，, 半径5
//			
//			Circle_Draw(&Robot_Mode_3, "141", 1, 3, 1, ROBOT_WIDE, 1920 - ROB3_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作1 添加，图层3，颜色1黄色，线宽8，, 半径5
//			
//			Circle_Draw(&Robot_Mode_4, "151", 1, 3, 1, ROBOT_WIDE, 1920 - ROB4_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作1 添加，图层3，颜色1黄色，线宽8，, 半径5
//			
//			Circle_Draw(&Robot_Mode_5, "161", 1, 3, 1, ROBOT_WIDE, 1920 - ROB5_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作1 添加，图层3，颜色1黄色，线宽8，, 半径5
//			
//			Circle_Draw(&Robot_Mode_7, "311", 1, 3, 1, ROBOT_WIDE, 1920 - ROB7_LOC_X, ROB7_LOC_Y, ROB7_RAD);//操作1 添加，图层3，颜色1黄色，线宽8，, 半径5
//			
//			Circle_Draw(&Robot_Mode_10, "312", 1, 3, 1, ROBOT_WIDE, 2040 - ROB7_LOC_X, ROB7_LOC_Y, ROB7_RAD);//操作1 添加，图层3，颜色1黄色，线宽8，, 半径5
//			
//			Circle_Draw(&MyRobot_Mode_1, "171", 1, 3, 1, ROBOT_WIDE, ROB1_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作1 添加，图层3，颜色1黄色，线宽8，, 半径5
//			Circle_Draw(&MyRobot_Mode_2, "172", 1, 3, 1, ROBOT_WIDE, ROB2_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//			Circle_Draw(&MyRobot_Mode_3, "173", 1, 3, 1, ROBOT_WIDE, ROB3_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//			Circle_Draw(&MyRobot_Mode_4, "174", 1, 3, 1, ROBOT_WIDE, ROB4_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//			Circle_Draw(&MyRobot_Mode_5, "175", 1, 3, 1, ROBOT_WIDE, ROB5_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//			Circle_Draw(&MyRobot_Mode_7, "176", 1, 3, 1, ROBOT_WIDE, ROB7_LOC_X, ROB7_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//			Circle_Draw(&MyRobot_Mode_10, "177", 1, 3, 1, ROBOT_WIDE, ROB7_LOC_X - 120, ROB7_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//		}
//		else
//		{
//			Circle_Draw(&Robot_Mode_1, "121", 1, 3, 1, ROBOT_WIDE, ROB1_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作1 添加，图层3，颜色1黄色，线宽8，, 半径5
//			
//			Circle_Draw(&Robot_Mode_2, "131", 1, 3, 1, ROBOT_WIDE, ROB2_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作1 添加，图层3，颜色1黄色，线宽8，, 半径5
//			
//			Circle_Draw(&Robot_Mode_3, "141", 1, 3, 1, ROBOT_WIDE, ROB3_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作1 添加，图层3，颜色1黄色，线宽8，, 半径5
//			
//			Circle_Draw(&Robot_Mode_4, "151", 1, 3, 1, ROBOT_WIDE, ROB4_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作1 添加，图层3，颜色1黄色，线宽8，, 半径5
//			
//			Circle_Draw(&Robot_Mode_5, "161", 1, 3, 1, ROBOT_WIDE, ROB5_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作1 添加，图层3，颜色1黄色，线宽8，, 半径5
//			
//			Circle_Draw(&Robot_Mode_7, "311", 1, 3, 1, ROBOT_WIDE, ROB7_LOC_X, ROB7_LOC_Y, ROB7_RAD);//操作1 添加，图层3，颜色1黄色，线宽8，, 半径5
//			
//			Circle_Draw(&Robot_Mode_10, "312", 1, 3, 1, ROBOT_WIDE, 2040 - ROB7_LOC_X, ROB7_LOC_Y, ROB7_RAD);//操作1 添加，图层3，颜色1黄色，线宽8，, 半径5
//			
//			Circle_Draw(&MyRobot_Mode_1, "171", 1, 3, 1, ROBOT_WIDE, 1920 - ROB1_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作1 添加，图层3，颜色1黄色，线宽8，, 半径5
//			Circle_Draw(&MyRobot_Mode_2, "172", 1, 3, 1, ROBOT_WIDE, 1920 - ROB2_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//			Circle_Draw(&MyRobot_Mode_3, "173", 1, 3, 1, ROBOT_WIDE, 1920 - ROB3_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//			Circle_Draw(&MyRobot_Mode_4, "174", 1, 3, 1, ROBOT_WIDE, 1920 - ROB4_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//			Circle_Draw(&MyRobot_Mode_5, "175", 1, 3, 1, ROBOT_WIDE, 1920 - ROB5_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//			Circle_Draw(&MyRobot_Mode_7, "176", 1, 3, 1, ROBOT_WIDE, 1920 - ROB7_LOC_X, ROB7_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//			Circle_Draw(&MyRobot_Mode_10, "177", 1, 3, 1, ROBOT_WIDE, 2040 - ROB7_LOC_X, ROB7_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//		}
//		
//		Circle_Draw(&Shoot_Mode_1, "611", 1, 6, 1, Circle_Wide, 960, 540, 30);//操作1 添加，图层6，颜色1黄色，线宽8，, 半径5
////		Float_Draw(&SC_Vol, "abc", 1, 6, 6, 40, 2, 3, 200, 700, C_Vol);//图层6 字号40 线宽3 颜色6(青色)
////		
//		Float_Draw(&Pitch_Angle, "700", 1, 7, 3, 15, 1, 3, 1050, 540, -pitch);//图层7 字号15 小数位数1 线宽3 颜色3(橙色)
//		Int32_Draw(&Int_Data1, "200", 1, 7, 1, 30, 8, 700, 880, 1);//图层7 字号15 小数位数1 线宽3 颜色3(橙色)
//		Int32_Draw(&Int_Data2, "201", 1, 7, 1, 30, 8, 1000, 880, 1);//图层7 字号15 小数位数1 线宽3 颜色3(橙色)
//		
//		
//		uint16_t Start_X = 30, Start_Y= 1080 - 200;
//		Float_Draw(&Sentry_Angle_1, "701", 1, 7, 3, 18, 1, 3, Start_X + 0*FLOAT_DELTA_X, Start_Y - 0*FLOAT_DELTA_Y, 0);//图层7 字号15 小数位数1 线宽3 颜色3(橙色)
//		Float_Draw(&Sentry_Angle_2, "702", 1, 7, 3, 18, 1, 3, Start_X + 1*FLOAT_DELTA_X, Start_Y - 0*FLOAT_DELTA_Y, 0);//图层7 字号15 小数位数1 线宽3 颜色3(橙色)
//		Float_Draw(&Sentry_Angle_3, "703", 1, 7, 3, 18, 1, 3, Start_X + 0*FLOAT_DELTA_X, Start_Y - 1*FLOAT_DELTA_Y, 0);//图层7 字号15 小数位数1 线宽3 颜色3(橙色)
//		Float_Draw(&Sentry_Angle_4, "704", 1, 7, 3, 18, 1, 3, Start_X + 1*FLOAT_DELTA_X, Start_Y - 1*FLOAT_DELTA_Y, 0);//图层7 字号15 小数位数1 线宽3 颜色3(橙色)
//		Float_Draw(&Sentry_Angle_5, "705", 1, 7, 3, 18, 1, 3, Start_X + 0*FLOAT_DELTA_X, Start_Y - 2*FLOAT_DELTA_Y, 0);//图层7 字号15 小数位数1 线宽3 颜色3(橙色)
//		Float_Draw(&Sentry_Angle_6, "706", 1, 7, 3, 18, 1, 3, Start_X + 1*FLOAT_DELTA_X, Start_Y - 2*FLOAT_DELTA_Y, 0);//图层7 字号15 小数位数1 线宽3 颜色3(橙色)
//		Float_Draw(&Sentry_Angle_7, "707", 1, 7, 3, 18, 1, 3, Start_X + 0*FLOAT_DELTA_X, Start_Y - 3*FLOAT_DELTA_Y, 0);//图层7 字号15 小数位数1 线宽3 颜色3(橙色)
//		
//		
//		
//		
//		
//	}
//}




///**
// * @brief 我方机器人哨兵跟随目标指示
// * @note  
// * @param 
// */
//Graph_Data_t MyRobot_Mode_1;
//Graph_Data_t MyRobot_Mode_2;
//Graph_Data_t MyRobot_Mode_3;
//Graph_Data_t MyRobot_Mode_4;
//Graph_Data_t MyRobot_Mode_5;
//Graph_Data_t MyRobot_Mode_7;
//Graph_Data_t MyRobot_Mode_10;
//void UI_MyRobot_Mode(uint8_t state)
//{
//	if(Robot_ID/10 == 0)		//红色方
//	{
//		Circle_Draw(&MyRobot_Mode_1, "171", 1, 3, 1, ROBOT_WIDE, ROB1_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作1 添加，图层3，颜色1黄色，线宽8，, 半径5
//		Circle_Draw(&MyRobot_Mode_2, "172", 1, 3, 1, ROBOT_WIDE, ROB2_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//		Circle_Draw(&MyRobot_Mode_3, "173", 1, 3, 1, ROBOT_WIDE, ROB3_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//		Circle_Draw(&MyRobot_Mode_4, "174", 1, 3, 1, ROBOT_WIDE, ROB4_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//		Circle_Draw(&MyRobot_Mode_5, "175", 1, 3, 1, ROBOT_WIDE, ROB5_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//		Circle_Draw(&MyRobot_Mode_7, "176", 1, 3, 1, ROBOT_WIDE, ROB7_LOC_X, ROB7_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//		Circle_Draw(&MyRobot_Mode_10, "177", 1, 3, 1, ROBOT_WIDE, ROB7_LOC_X - 120, ROB7_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//		switch(state)
//		{
//			case 1:
//				Circle_Draw(&MyRobot_Mode_1, "171", 2, 3, 3, ROBOT_WIDE, ROB1_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_2, "172", 2, 3, 2, ROBOT_WIDE, ROB2_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_3, "173", 2, 3, 2, ROBOT_WIDE, ROB3_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_4, "174", 2, 3, 2, ROBOT_WIDE, ROB4_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_5, "175", 2, 3, 2, ROBOT_WIDE, ROB5_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_7, "176", 2, 3, 2, ROBOT_WIDE, ROB7_LOC_X, ROB7_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_10, "177", 2, 3, 2, ROBOT_WIDE, ROB7_LOC_X - 120, ROB7_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				break;
//			case 2:
//				Circle_Draw(&MyRobot_Mode_1, "171", 2, 3, 2, ROBOT_WIDE, ROB1_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_2, "172", 2, 3, 3, ROBOT_WIDE, ROB2_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_3, "173", 2, 3, 2, ROBOT_WIDE, ROB3_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_4, "174", 2, 3, 2, ROBOT_WIDE, ROB4_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_5, "175", 2, 3, 2, ROBOT_WIDE, ROB5_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_7, "176", 2, 3, 2, ROBOT_WIDE, ROB7_LOC_X, ROB7_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_10, "177", 2, 3, 2, ROBOT_WIDE, ROB7_LOC_X - 120, ROB7_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				break;
//			case 3:
//				Circle_Draw(&MyRobot_Mode_1, "171", 2, 3, 2, ROBOT_WIDE, ROB1_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_2, "172", 2, 3, 2, ROBOT_WIDE, ROB2_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_3, "173", 2, 3, 3, ROBOT_WIDE, ROB3_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_4, "174", 2, 3, 2, ROBOT_WIDE, ROB4_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_5, "175", 2, 3, 2, ROBOT_WIDE, ROB5_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_7, "176", 2, 3, 2, ROBOT_WIDE, ROB7_LOC_X, ROB7_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_10, "177", 2, 3, 2, ROBOT_WIDE, ROB7_LOC_X - 120, ROB7_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				break;
//			case 4:
//				Circle_Draw(&MyRobot_Mode_1, "171", 2, 3, 2, ROBOT_WIDE, ROB1_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_2, "172", 2, 3, 2, ROBOT_WIDE, ROB2_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_3, "173", 2, 3, 2, ROBOT_WIDE, ROB3_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_4, "174", 2, 3, 3, ROBOT_WIDE, ROB4_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_5, "175", 2, 3, 2, ROBOT_WIDE, ROB5_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_7, "176", 2, 3, 2, ROBOT_WIDE, ROB7_LOC_X, ROB7_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_10, "177", 2, 3, 2, ROBOT_WIDE, ROB7_LOC_X - 120, ROB7_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				break;
//			case 5:
//				Circle_Draw(&MyRobot_Mode_1, "171", 2, 3, 2, ROBOT_WIDE, ROB1_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_2, "172", 2, 3, 2, ROBOT_WIDE, ROB2_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_3, "173", 2, 3, 2, ROBOT_WIDE, ROB3_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_4, "174", 2, 3, 2, ROBOT_WIDE, ROB4_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_5, "175", 2, 3, 3, ROBOT_WIDE, ROB5_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_7, "176", 2, 3, 2, ROBOT_WIDE, ROB7_LOC_X, ROB7_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_10, "177", 2, 3, 2, ROBOT_WIDE, ROB7_LOC_X - 120, ROB7_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				break;
//			case 7:
//				Circle_Draw(&MyRobot_Mode_1, "171", 2, 3, 2, ROBOT_WIDE, ROB1_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_2, "172", 2, 3, 2, ROBOT_WIDE, ROB2_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_3, "173", 2, 3, 2, ROBOT_WIDE, ROB3_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_4, "174", 2, 3, 2, ROBOT_WIDE, ROB4_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_5, "175", 2, 3, 2, ROBOT_WIDE, ROB5_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_7, "176", 2, 3, 3, ROBOT_WIDE, ROB7_LOC_X, ROB7_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_10, "177", 2, 3, 2, ROBOT_WIDE, ROB7_LOC_X - 120, ROB7_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				break;
//			case 10:
//				Circle_Draw(&MyRobot_Mode_1, "171", 2, 3, 2, ROBOT_WIDE, ROB1_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_2, "172", 2, 3, 2, ROBOT_WIDE, ROB2_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_3, "173", 2, 3, 2, ROBOT_WIDE, ROB3_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_4, "174", 2, 3, 2, ROBOT_WIDE, ROB4_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_5, "175", 2, 3, 2, ROBOT_WIDE, ROB5_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_7, "176", 2, 3, 2, ROBOT_WIDE, ROB7_LOC_X, ROB7_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_10, "177", 2, 3, 3, ROBOT_WIDE, ROB7_LOC_X - 120, ROB7_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				break;
//		}
//	}
//	else
//	{
//		Circle_Draw(&MyRobot_Mode_1, "171", 1, 3, 1, ROBOT_WIDE, 1920 - ROB1_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作1 添加，图层3，颜色1黄色，线宽8，, 半径5
//		Circle_Draw(&MyRobot_Mode_2, "172", 1, 3, 1, ROBOT_WIDE, 1920 - ROB2_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//		Circle_Draw(&MyRobot_Mode_3, "173", 1, 3, 1, ROBOT_WIDE, 1920 - ROB3_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//		Circle_Draw(&MyRobot_Mode_4, "174", 1, 3, 1, ROBOT_WIDE, 1920 - ROB4_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//		Circle_Draw(&MyRobot_Mode_5, "175", 1, 3, 1, ROBOT_WIDE, 1920 - ROB5_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//		Circle_Draw(&MyRobot_Mode_7, "176", 1, 3, 1, ROBOT_WIDE, 1920 - ROB7_LOC_X, ROB7_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//		Circle_Draw(&MyRobot_Mode_10, "177", 1, 3, 1, ROBOT_WIDE, 2040 - ROB7_LOC_X, ROB7_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//		switch(state)
//		{
//			case 1:
//				Circle_Draw(&MyRobot_Mode_1, "171", 2, 3, 3, ROBOT_WIDE, 1920 - ROB1_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_2, "172", 2, 3, 2, ROBOT_WIDE, 1920 - ROB2_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_3, "173", 2, 3, 2, ROBOT_WIDE, 1920 - ROB3_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_4, "174", 2, 3, 2, ROBOT_WIDE, 1920 - ROB4_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_5, "175", 2, 3, 2, ROBOT_WIDE, 1920 - ROB5_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_7, "176", 2, 3, 2, ROBOT_WIDE, 1920 - ROB7_LOC_X, ROB7_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_10, "177", 2, 3, 2, ROBOT_WIDE, 2040 - ROB7_LOC_X, ROB7_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				break;
//			case 2:
//				Circle_Draw(&MyRobot_Mode_1, "171", 2, 3, 2, ROBOT_WIDE, 1920 - ROB1_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_2, "172", 2, 3, 3, ROBOT_WIDE, 1920 - ROB2_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_3, "173", 2, 3, 2, ROBOT_WIDE, 1920 - ROB3_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_4, "174", 2, 3, 2, ROBOT_WIDE, 1920 - ROB4_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_5, "175", 2, 3, 2, ROBOT_WIDE, 1920 - ROB5_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_7, "176", 2, 3, 2, ROBOT_WIDE, 1920 - ROB7_LOC_X, ROB7_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_10, "177", 2, 3, 2, ROBOT_WIDE, 2040 - ROB7_LOC_X, ROB7_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				break;
//			case 3:
//				Circle_Draw(&MyRobot_Mode_1, "171", 2, 3, 2, ROBOT_WIDE, 1920 - ROB1_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_2, "172", 2, 3, 2, ROBOT_WIDE, 1920 - ROB2_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_3, "173", 2, 3, 3, ROBOT_WIDE, 1920 - ROB3_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_4, "174", 2, 3, 2, ROBOT_WIDE, 1920 - ROB4_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_5, "175", 2, 3, 2, ROBOT_WIDE, 1920 - ROB5_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_7, "176", 2, 3, 2, ROBOT_WIDE, 1920 - ROB7_LOC_X, ROB7_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_10, "177", 2, 3, 2, ROBOT_WIDE, 2040 - ROB7_LOC_X, ROB7_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				break;
//			case 4:
//				Circle_Draw(&MyRobot_Mode_1, "171", 2, 3, 2, ROBOT_WIDE, 1920 - ROB1_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_2, "172", 2, 3, 2, ROBOT_WIDE, 1920 - ROB2_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_3, "173", 2, 3, 2, ROBOT_WIDE, 1920 - ROB3_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_4, "174", 2, 3, 3, ROBOT_WIDE, 1920 - ROB4_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_5, "175", 2, 3, 2, ROBOT_WIDE, 1920 - ROB5_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_7, "176", 2, 3, 2, ROBOT_WIDE, 1920 - ROB7_LOC_X, ROB7_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_10, "177", 2, 3, 2, ROBOT_WIDE, 2040 - ROB7_LOC_X, ROB7_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				break;
//			case 5:
//				Circle_Draw(&MyRobot_Mode_1, "171", 2, 3, 2, ROBOT_WIDE, 1920 - ROB1_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_2, "172", 2, 3, 2, ROBOT_WIDE, 1920 - ROB2_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_3, "173", 2, 3, 2, ROBOT_WIDE, 1920 - ROB3_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_4, "174", 2, 3, 2, ROBOT_WIDE, 1920 - ROB4_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_5, "175", 2, 3, 3, ROBOT_WIDE, 1920 - ROB5_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_7, "176", 2, 3, 2, ROBOT_WIDE, 1920 - ROB7_LOC_X, ROB7_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_10, "177", 2, 3, 2, ROBOT_WIDE, 2040 - ROB7_LOC_X, ROB7_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				break;
//			case 7:
//				Circle_Draw(&MyRobot_Mode_1, "171", 2, 3, 2, ROBOT_WIDE, 1920 - ROB1_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_2, "172", 2, 3, 2, ROBOT_WIDE, 1920 - ROB2_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_3, "173", 2, 3, 2, ROBOT_WIDE, 1920 - ROB3_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_4, "174", 2, 3, 2, ROBOT_WIDE, 1920 - ROB4_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_5, "175", 2, 3, 2, ROBOT_WIDE, 1920 - ROB5_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_7, "176", 2, 3, 3, ROBOT_WIDE, 1920 - ROB7_LOC_X, ROB7_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_10, "177", 2, 3, 2, ROBOT_WIDE, 2040 - ROB7_LOC_X, ROB7_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				break;
//			case 10:
//				Circle_Draw(&MyRobot_Mode_1, "171", 2, 3, 2, ROBOT_WIDE, 1920 - ROB1_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_2, "172", 2, 3, 2, ROBOT_WIDE, 1920 - ROB2_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_3, "173", 2, 3, 2, ROBOT_WIDE, 1920 - ROB3_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_4, "174", 2, 3, 2, ROBOT_WIDE, 1920 - ROB4_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_5, "175", 2, 3, 2, ROBOT_WIDE, 1920 - ROB5_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_7, "176", 2, 3, 2, ROBOT_WIDE, 1920 - ROB7_LOC_X, ROB7_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//				Circle_Draw(&MyRobot_Mode_10, "177", 2, 3, 2, ROBOT_WIDE, 2040 - ROB7_LOC_X, ROB7_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//				break;
//		}
//	}
//}



///**
// * @brief 1号英雄机器人目标指示
// * @note  
// * @param 
// */
//Graph_Data_t Robot_Mode_1;
//void UI_Robot1_Mode(int8_t state)
//{
//	if(Robot_ID/10 == 0)		//红色方
//	{
//		Circle_Draw(&Robot_Mode_1, "121", 1, 3, 1, ROBOT_WIDE, 1920 - ROB1_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作1 添加，图层3，颜色1黄色，线宽8，, 半径5
//		if(state == 1)
//			Circle_Draw(&Robot_Mode_1, "121", 2, 3, 2, ROBOT_WIDE, 1920 - ROB1_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//		else
//			Circle_Draw(&Robot_Mode_1, "121", 2, 3, 3, ROBOT_WIDE, 1920 - ROB1_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//	}
//	else
//	{
//		Circle_Draw(&Robot_Mode_1, "121", 1, 3, 1, ROBOT_WIDE, ROB1_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作1 添加，图层3，颜色1黄色，线宽8，, 半径5
//		if(state == 1)
//			Circle_Draw(&Robot_Mode_1, "121", 2, 3, 2, ROBOT_WIDE, ROB1_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//		else
//			Circle_Draw(&Robot_Mode_1, "121", 2, 3, 3, ROBOT_WIDE, ROB1_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//	}
//}

///**
// * @brief 2号工程机器人目标指示
// * @note  
// * @param 
// */
//Graph_Data_t Robot_Mode_2;
//void UI_Robot2_Mode(int8_t state)
//{
//	if(Robot_ID/10 == 0)		//红色方
//	{
//		Circle_Draw(&Robot_Mode_2, "131", 1, 3, 1, ROBOT_WIDE, 1920 - ROB2_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作1 添加，图层3，颜色1黄色，线宽8，, 半径5
//		if(state == 1)
//			Circle_Draw(&Robot_Mode_2, "131", 2, 3, 2, ROBOT_WIDE, 1920 - ROB2_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//		else
//			Circle_Draw(&Robot_Mode_2, "131", 2, 3, 3, ROBOT_WIDE, 1920 - ROB2_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//	}
//	else
//	{
//		Circle_Draw(&Robot_Mode_2, "131", 1, 3, 1, ROBOT_WIDE, ROB2_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作1 添加，图层3，颜色1黄色，线宽8，, 半径5
//		if(state == 1)
//			Circle_Draw(&Robot_Mode_2, "131", 2, 3, 2, ROBOT_WIDE, ROB2_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//		else
//			Circle_Draw(&Robot_Mode_2, "131", 2, 3, 3, ROBOT_WIDE, ROB2_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//	}
//}

///**
// * @brief 3号步兵机器人目标指示
// * @note  
// * @param 
// */
//Graph_Data_t Robot_Mode_3;
//void UI_Robot3_Mode(int8_t state)
//{
//	if(Robot_ID/10 == 0)		//红色方
//	{
//		Circle_Draw(&Robot_Mode_3, "141", 1, 3, 1, ROBOT_WIDE, 1920 - ROB3_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作1 添加，图层3，颜色1黄色，线宽8，, 半径5
//		if(state == 1)
//			Circle_Draw(&Robot_Mode_3, "141", 2, 3, 2, ROBOT_WIDE, 1920 - ROB3_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//		else
//			Circle_Draw(&Robot_Mode_3, "141", 2, 3, 3, ROBOT_WIDE, 1920 - ROB3_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//	}
//	else
//	{
//		Circle_Draw(&Robot_Mode_3, "141", 1, 3, 1, ROBOT_WIDE, ROB3_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作1 添加，图层3，颜色1黄色，线宽8，, 半径5
//		if(state == 1)
//			Circle_Draw(&Robot_Mode_3, "141", 2, 3, 2, ROBOT_WIDE, ROB3_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//		else
//			Circle_Draw(&Robot_Mode_3, "141", 2, 3, 3, ROBOT_WIDE, ROB3_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//	}
//}

///**
// * @brief 4号步兵机器人目标指示
// * @note  
// * @param 
// */
//Graph_Data_t Robot_Mode_4;
//void UI_Robot4_Mode(int8_t state)
//{
//	if(Robot_ID/10 == 0)		//红色方
//	{
//		Circle_Draw(&Robot_Mode_4, "151", 1, 3, 1, ROBOT_WIDE, 1920 - ROB4_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作1 添加，图层3，颜色1黄色，线宽8，, 半径5
//		if(state == 1)
//			Circle_Draw(&Robot_Mode_4, "151", 2, 3, 2, ROBOT_WIDE, 1920 - ROB4_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//		else
//			Circle_Draw(&Robot_Mode_4, "151", 2, 3, 3, ROBOT_WIDE, 1920 - ROB4_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//	}
//	else
//	{
//		Circle_Draw(&Robot_Mode_4, "151", 1, 3, 1, ROBOT_WIDE, ROB4_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作1 添加，图层3，颜色1黄色，线宽8，, 半径5
//		if(state == 1)
//			Circle_Draw(&Robot_Mode_4, "151", 2, 3, 2, ROBOT_WIDE, ROB4_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//		else
//			Circle_Draw(&Robot_Mode_4, "151", 2, 3, 3, ROBOT_WIDE, ROB4_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//	}
//}

///**
// * @brief 5号步兵机器人目标指示
// * @note  
// * @param 
// */
//Graph_Data_t Robot_Mode_5;
//void UI_Robot5_Mode(int8_t state)
//{
//	if(Robot_ID/10 == 0)		//红色方
//	{
//		Circle_Draw(&Robot_Mode_5, "161", 1, 3, 1, ROBOT_WIDE, 1920 - ROB5_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作1 添加，图层3，颜色1黄色，线宽8，, 半径5
//		if(state == 1)
//			Circle_Draw(&Robot_Mode_5, "161", 2, 3, 2, ROBOT_WIDE, 1920 - ROB5_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//		else
//			Circle_Draw(&Robot_Mode_5, "161", 2, 3, 3, ROBOT_WIDE, 1920 - ROB5_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//	}
//	else
//	{
//		Circle_Draw(&Robot_Mode_5, "161", 1, 3, 1, ROBOT_WIDE, ROB5_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作1 添加，图层3，颜色1黄色，线宽8，, 半径5
//		if(state == 1)
//			Circle_Draw(&Robot_Mode_5, "161", 2, 3, 2, ROBOT_WIDE, ROB5_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//		else
//			Circle_Draw(&Robot_Mode_5, "161", 2, 3, 3, ROBOT_WIDE, ROB5_LOCAT, ROBOT_LOC_Y, ROBOT_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//	}
//}

///**
// * @brief 7号烧饼机器人目标指示
// * @note  
// * @param 
// */
//Graph_Data_t Robot_Mode_7;
//void UI_Robot7_Mode(int8_t state)
//{
//	if(Robot_ID/10 == 0)		//红色方
//	{
//		Circle_Draw(&Robot_Mode_7, "311", 1, 3, 1, ROBOT_WIDE, 1920 - ROB7_LOC_X, ROB7_LOC_Y, ROB7_RAD);//操作1 添加，图层3，颜色1黄色，线宽8，, 半径5
//		if(state == 1)
//			Circle_Draw(&Robot_Mode_7, "311", 2, 3, 2, ROBOT_WIDE, 1920 - ROB7_LOC_X, ROB7_LOC_Y, ROB7_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//		else
//			Circle_Draw(&Robot_Mode_7, "311", 2, 3, 3, ROBOT_WIDE, 1920 - ROB7_LOC_X, ROB7_LOC_Y, ROB7_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//	}
//	else
//	{
//		Circle_Draw(&Robot_Mode_7, "311", 1, 3, 1, ROBOT_WIDE, ROB7_LOC_X, ROB7_LOC_Y, ROB7_RAD);//操作1 添加，图层3，颜色1黄色，线宽8，, 半径5
//		if(state == 1)
//			Circle_Draw(&Robot_Mode_7, "311", 2, 3, 2, ROBOT_WIDE, ROB7_LOC_X, ROB7_LOC_Y, ROB7_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//		else
//			Circle_Draw(&Robot_Mode_7, "311", 2, 3, 3, ROBOT_WIDE, ROB7_LOC_X, ROB7_LOC_Y, ROB7_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//	}
//}

///**
// * @brief 10号敌方前哨战目标指示
// * @note  
// * @param 
// */
//Graph_Data_t Robot_Mode_10;
//void UI_Robot10_Mode(int8_t state)
//{
//	if(Robot_ID/10 == 0)		//红色方
//	{
//		Circle_Draw(&Robot_Mode_10, "312", 1, 3, 1, ROBOT_WIDE, 2040 - ROB7_LOC_X, ROB7_LOC_Y, ROB7_RAD);//操作1 添加，图层3，颜色1黄色，线宽8，, 半径5
//		if(state == 1)
//			Circle_Draw(&Robot_Mode_10, "312", 2, 3, 2, ROBOT_WIDE, 2040 - ROB7_LOC_X, ROB7_LOC_Y, ROB7_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//		else
//			Circle_Draw(&Robot_Mode_10, "312", 2, 3, 3, ROBOT_WIDE, 2040 - ROB7_LOC_X, ROB7_LOC_Y, ROB7_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//	}
//	else
//	{
//		Circle_Draw(&Robot_Mode_10, "312", 1, 3, 1, ROBOT_WIDE, ROB7_LOC_X - 120, ROB7_LOC_Y, ROB7_RAD);//操作1 添加，图层3，颜色1黄色，线宽8，, 半径5
//		if(state == 1)
//			Circle_Draw(&Robot_Mode_10, "312", 2, 3, 2, ROBOT_WIDE, ROB7_LOC_X - 120, ROB7_LOC_Y, ROB7_RAD);//操作2 修改，图层3，颜色2绿色，线宽8，, 半径5
//		else
//			Circle_Draw(&Robot_Mode_10, "312", 2, 3, 3, ROBOT_WIDE, ROB7_LOC_X - 120, ROB7_LOC_Y, ROB7_RAD);//操作2 修改，图层3，颜色3橙色，线宽8，, 半径5
//	}
//}


///**
// * @brief 发弹显示
// * @note  display Shoot
// * @param 
// */
//Graph_Data_t Shoot_Mode_1;
//void UI_Shoot_Mode(int8_t state)
//{
//	Circle_Draw(&Shoot_Mode_1, "611", 1, 6, 1, Circle_Wide, 960, 540, 30);//操作1 添加，图层6，颜色1黄色，线宽8，, 半径5
//	if(state == 1)						//当前可以发弹
//		Circle_Draw(&Shoot_Mode_1, "611", 2, 6, 2, Circle_Wide, 960, 540, 30);//操作2 修改，图层6，颜色2绿色，线宽8，, 半径5
//	else 
//		Circle_Draw(&Shoot_Mode_1, "611", 2, 6, 3, Circle_Wide, 960, 540, 30);//操作2 修改，图层6，颜色6橙色，线宽8，, 半径5
//}











