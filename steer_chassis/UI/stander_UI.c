#include "stander_UI.h"
#include "system.h"
#include "Nautilus_UI.h"
#include "system.h"
#include "referee.h"
#include "freertos.h"
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
#include "stander_UI.h"
#include "bsp_can.h"
#include "referee.h"
#include "ins_task.h"
#include "remote_control.h"
#include "cmsis_os.h"
#include "trig.h"
//#include "Nautilus_Vision.h"
#include "shoot.h"
#include "gimbal.h"
#include "motor.h"
#include "Nautilus_UI.h"
#include "CAN_receive.h"
#include "vision.h"
/*-----------------------------------------控制层-----------------------------------------*/
/**
 * @brief UI发送总函数
 * @note  UI send
 * @param 
 */
 int mmmmbmm;
int UI_cnt;
void UI_Task(void)
{
	for(;;)   
	{
	mmmmbmm++;
		UI_ID_Set();
		
//		UI_NAUTILUS_LOGO();//放这个位置显示比较流畅
		
		if(rc_ctrl.keyboard.key_G == 1)
		{
			UI_Reset();				//放在浮点型之后运行,按R手动刷新UI
		}
		else 
		{
			UI_Cap_Voltage();		//电容电压
			UI_Cap();				//弧形显示电容电压
			UI_Pitch_Angle();		//pitch轴角度
			UI_Action_Mode();		//运动模式标识
			UI_Shoot_Mode();		//拨弹轮状态
			UI_Fric_Mode();			//摩擦轮状态
			UI_Vision_Mode();		//视觉模式
			UI_Special_ACT();		//特殊运动模式，动态
			UI_Pitch_Mode();
		  UI_Shoot_Num();
			UI_Power_Mode();
			UI_Ctrl_Mode();
			UI_DM_Mode();
			UI_Shift_Mode();
      
			UI_SendGraph( 1, Special_ACT_3);//放在下一行（连续发7个）的前面就很丝滑，放后面就卡住
			vTaskDelay(30);			
			UI_SendGraph( 5,  Pitch_Angle, Action_Mode_1, Shoot_Mode_1,Fric_Mode_1 , Shoot_Mode_1);
		  vTaskDelay(30);
			UI_SendGraph( 5, UI_ShootNum_1, UI_ShootNum_3,UI_ShootNum_2,UI_ShootNum_4,UI_ShootNum_5);
			vTaskDelay(30);
			UI_SendGraph( 5, chassis_UI,gimbal_UI,shoot_UI,Ctrl_Mode_1,Shift_Mode);//放在下一行（连续发7个）的前面就很丝滑，放后面就卡住
			vTaskDelay(30);
			UI_SendGraph( 1, DM_Mode);//放在下一行（连续发7个）的前面就很丝滑，放后面就卡住
		}
		
		vTaskDelay(50);
	}
}


/**
 * @brief 刷新UI
 * @note  防止浮点型在UI清除之后不显示
 * @param 
 */
void UI_Reset(void)
{
		//静态UI创建操作
		UI_Gun_Sight();			//瞄准镜
		
		UI_SendGraph(2,GUN_Sight_LateralAxis_1,GUN_Sight_LateralAxis_3);
		vTaskDelay(30);
	UI_Anticollition();		//防撞提示
	
		UI_SendGraph(5, Anticollition_1, Anticollition_2, Anticollition_3, DM_Mode, Shift_Mode 
					);
		vTaskDelay(30);
		
		//视觉自瞄范围(静态)
		Circle_Draw(&chassis_UI, "851", 1, 3, 1, 8, 200, 600, 15);//操作2 修改，图层3，颜色7黑色，线宽8，, 半径5
		Circle_Draw(&gimbal_UI, "852", 1, 3, 1, 8, 275, 600, 15);//操作2 修改，图层3，颜色7黑色，线宽8，, 半径5
		Circle_Draw(&shoot_UI, "853", 1, 3, 1, 8, 350, 600, 15);//操作2 修改，图层3，颜色7黑色，线宽8，, 半径5
		Circle_Draw(&Ctrl_Mode_1, "888", 1, 4, 8, 8, 425, 600, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
//		
		//电容
//		Arc_Draw(&SC_Outline_Arc_1, "711", 1, 6, 1, 1, 45, 135, 960, 540, 301, 301);//操作1 创建，图层6，颜色1黄色，线宽1，起始角度，终点角度，圆心（960，540）,xy半径
//		Arc_Draw(&SC_Outline_Arc_2, "712", 1, 6, 1, 1, 45, 135, 960, 540, 295, 295);//操作1 创建，图层6，颜色1黄色，线宽1，起始角度，终点角度，圆心（960，540）,xy半径
//		Line_Draw(&SC_Outline_Line_1, "713", 1, 6, 1, 1, 1168, (1080-331), 1173, (1080-326) );//操作1 创建，图层6，颜色1黄色，线宽1，
//		Line_Draw(&SC_Outline_Line_2, "714", 1, 6, 1, 1, 1168, (1080-747), 1173, (1080-753) );//操作1 创建，图层6，颜色1黄色，线宽1，
//		Arc_Draw(&SC_Vol_Arc, "715", 1, 6, 5, 5, 45, 135, 960, 540, 298, 298);//操作1 创建，图层6，颜色5粉色，线宽5，起始角度，终点角度，圆心（960，540）,xy半径
		
		UI_SendGraph( 2, chassis_UI, gimbal_UI);
		vTaskDelay(30);
		
		//动态UI创建操作
//		Float_Draw(&SC_Vol, "abc", 1, 6, 6, 40, 2, 3, 200, 700, SuperCAP.C_Vol);//电容电压//图层6 字号40 线宽3 颜色6(青色)
		
		Float_Draw(&Pitch_Angle, "acb", 1, 7, 3, 15, 1, 3, 1050, 540, INS.Pitch);//PITCH角度//图层7 字号15 小数位数1 线宽3 颜色3(橙色)
		
		Circle_Draw(&Action_Mode_1, "411", 1, 3, 1, 8, 795, 150, 15);//运动模式//操作1 添加，图层3，颜色1黄色，线宽8，, 半径5
		
		Circle_Draw(&Fric_Mode_1, "511", 1, 4, 1, 8, 905, 150, 15);//摩擦轮状态//操作1 添加，图层4，颜色1黄色，线宽8，, 半径5
		
		Circle_Draw(&Shoot_Mode_1, "611", 1, 5, 1, 8, 1015, 150, 15);//拨弹轮状态//操作1 添加，图层5，颜色1黄色，线宽8，, 半径5
		
//		Circle_Draw(&Vision_Mode_1, "211", 1, 5, 1, 8, 1125, 150, 15);//视觉模式//操作1 添加，图层5，颜色1黄色，线宽8，, 半径5		//默认视觉关闭
		

		UI_SendGraph( 5, Pitch_Angle, Action_Mode_1, Fric_Mode_1, Shoot_Mode_1,Ctrl_Mode_1);
		vTaskDelay(40);
		
		Circle_Draw(&UI_ShootNum_1, "971", 1, 4, 0, 8, 1655, 450, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
		Circle_Draw(&UI_ShootNum_2, "972", 1, 4, 0, 8, 1655, 525, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
		Circle_Draw(&UI_ShootNum_3, "973", 1, 4, 0, 8, 1655, 600, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
		Circle_Draw(&UI_ShootNum_4, "974", 1, 4, 0, 8, 1655, 675, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
		Circle_Draw(&UI_ShootNum_5, "975", 1, 4, 0, 8, 1655, 750, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
		UI_SendGraph( 5, UI_ShootNum_1, UI_ShootNum_3,UI_ShootNum_2,UI_ShootNum_4,UI_ShootNum_5);
		vTaskDelay(40);

//		Circle_Draw(&Pitch_Mode, "814", 1, 4, 6, 8, 955, 75, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
		

		// UI_SendGraph( 1, UI_ShootNum);
		// vTaskDelay(20);
		//运动状态
		Line_Draw(&Special_ACT_1, "911", 1, 1, 6, 6, 570, 250, 570, 290 );	//操作1 添加，图层1，颜色0主色，线宽4
		Circle_Draw(&Special_ACT_2, "912", 1, 1, 0, 2, 570, 230, 35);			//操作1 添加，图层1，颜色2绿色，线宽2，半径35
		Arc_Draw(&Special_ACT_3, "913", 1, 1, 6, 6, 135, 225, 570, 230, 35, 35 );	//操作1 添加，图层1，颜色0主色，线宽2，半径35
		
		UI_SendGraph( 5, Special_ACT_2,Special_ACT_2, Special_ACT_1, Special_ACT_3,shoot_UI);
		
}

/*-----------------------------------------功能层-----------------------------------------*/
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

void UI_Gun_Sight(void)
{
	Line_Draw( &GUN_Sight_MainAxis, "111", 1, 0, 1, 2, 860, 800, 860, 280 );//操作1(添加), 图层0, 颜色1(黄色), 线宽2
	Line_Draw( &GUN_Sight_LateralAxis_1, "121", 1, 0, 1, 2, 970, 420, 970, 530 );
	Circle_Draw(&DM_Mode, "858", 1, 5, 8, 8, 1055, 75, 15);//操作2 修改，图层5，颜色5粉色，线宽8，, 半径5
	Line_Draw( &GUN_Sight_LateralAxis_2, "122", 1, 0, 1, 2, 910, 540, 1010, 540 );
	Line_Draw( &GUN_Sight_LateralAxis_3, "123", 1, 0, 1, 2, 915, 470, 1015, 470 );//三米
	Circle_Draw(&Shift_Mode, "857", 1, 5, 8, 8, 855, 75, 15);//操作2 修改，图层5，颜色7黑色，线宽8，, 半径5


}

/**
 * @brief 视觉模式显示
 * @note  display VISION_Mode
 * @param 
 */
Graph_Data_t visual_field_4;
Graph_Data_t Vision_Mode_1;	//动态
void UI_Vision_Mode(void)
{
	if(shoot_mode == version_small)
		Circle_Draw(&Vision_Mode_1, "211", 2, 5, 6, 8, 1125, 150, 15);//操作2 修改，图层5，颜色2绿色，线宽8，, 半径5	//大符模式
	if(shoot_mode == version_big)
		Circle_Draw(&Vision_Mode_1, "211", 2, 5, 5, 8, 1125, 150, 15);//操作2 修改，图层5，颜色5粉色，线宽8，, 半径5	//小符模式
	if(shoot_mode == version_armor)
		Circle_Draw(&Vision_Mode_1, "211", 2, 5, 0, 8, 1125, 150, 15);//操作2 修改，图层5，颜色6青色，线宽8，, 半径5	//装甲板模式
	if(shoot_mode == version_off)				
		Circle_Draw(&Vision_Mode_1, "211", 2, 5, 8, 8, 1125, 150, 15);//操作2 修改，图层5，颜色7黑色，线宽8，, 半径5	//关闭
}

/**
 * @brief 3--防撞提示
 * @note  
 * @param 命名方式 第一位表示防撞提示类，第二位表示...
 */
Graph_Data_t Anticollition_1;//防撞提示-横线
Graph_Data_t Anticollition_2;//左斜"\"
Graph_Data_t Anticollition_3;//右斜"/"
void UI_Anticollition(void)
{
	Line_Draw( &Anticollition_1, "311", 1, 2, 1, 2, 320, 200, 1600, 200 );//操作1(添加), 图层2, 颜色1(黄色), 线宽2
	Line_Draw( &Anticollition_2, "321", 1, 2, 1, 2, 1370, 540, 1570, 0 );
	Line_Draw( &Anticollition_3, "322", 1, 2, 1, 2, 350, 0, 550, 540 );
}

/**
 * @brief 电容电压显示
 * @note  display SuperCap voltage
 * @param 
 */
FloInt_Data_t SC_Vol;
float C_Vol_Last;
void UI_Cap_Voltage(void)
{
		if((SuperCAP.C_Vol-C_Vol_Last)<=0.2f || (SuperCAP.C_Vol-C_Vol_Last)>=-0.2f){
			Float_Draw(&SC_Vol, "abc", 2, 6, 6, 40, 2, 3, 200, 700, SuperCAP.C_Vol);//修改
		}
		C_Vol_Last = SuperCAP.C_Vol;
}

/**
 * @brief 发弹量显示
 * @note  display SuperCap voltage
 * @param 
 */
Graph_Data_t UI_ShootNum_1;
Graph_Data_t UI_ShootNum_2;
Graph_Data_t UI_ShootNum_3;
Graph_Data_t UI_ShootNum_4;
Graph_Data_t UI_ShootNum_5;
int32_t ShootNum = 0;
int32_t Last_Num;
void UI_Shoot_Num(void)
{
	ShootNum = Report_Shoot_NUM();
	if(ShootNum <= 100)
	{
		Circle_Draw(&UI_ShootNum_5, "975", 2, 4, 0, 8, 1655, 750, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
		Circle_Draw(&UI_ShootNum_4, "974", 2, 4, 0, 8, 1655, 675, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
		Circle_Draw(&UI_ShootNum_3, "973", 2, 4, 0, 8, 1655, 600, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
		Circle_Draw(&UI_ShootNum_2, "972", 2, 4, 0, 8, 1655, 525, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
		Circle_Draw(&UI_ShootNum_1, "971", 2, 4, 0, 8, 1655, 450, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5

	}
	if(ShootNum <=200 && ShootNum > 100)
	{
		Circle_Draw(&UI_ShootNum_5, "975", 2, 4, 8, 8, 1655, 750, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
		Circle_Draw(&UI_ShootNum_4, "974", 2, 4, 0, 8, 1655, 675, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
		Circle_Draw(&UI_ShootNum_3, "973", 2, 4, 0, 8, 1655, 600, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
		Circle_Draw(&UI_ShootNum_2, "972", 2, 4, 0, 8, 1655, 525, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
		Circle_Draw(&UI_ShootNum_1, "971", 2, 4, 0, 8, 1655, 450, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
	}
	else if(ShootNum <=300 && ShootNum > 200)
	{
		Circle_Draw(&UI_ShootNum_5, "975", 2, 4, 8, 8, 1655, 750, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
		Circle_Draw(&UI_ShootNum_4, "974", 2, 4, 8, 8, 1655, 675, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
		Circle_Draw(&UI_ShootNum_3, "973", 2, 4, 0, 8, 1655, 600, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
		Circle_Draw(&UI_ShootNum_2, "972", 2, 4, 0, 8, 1655, 525, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
		Circle_Draw(&UI_ShootNum_1, "971", 2, 4, 0, 8, 1655, 450, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
	}
	else if(ShootNum <=400 && ShootNum > 300)
	{
		Circle_Draw(&UI_ShootNum_5, "975", 2, 4, 8, 8, 1655, 750, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
		Circle_Draw(&UI_ShootNum_4, "974", 2, 4, 8, 8, 1655, 675, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
		Circle_Draw(&UI_ShootNum_3, "973", 2, 4, 8, 8, 1655, 600, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
		Circle_Draw(&UI_ShootNum_2, "972", 2, 4, 0, 8, 1655, 525, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
		Circle_Draw(&UI_ShootNum_1, "971", 2, 4, 0, 8, 1655, 450, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
	}
	else if(ShootNum <=500 && ShootNum > 400)
	{
		Circle_Draw(&UI_ShootNum_5, "975", 2, 4, 8, 8, 1655, 750, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
		Circle_Draw(&UI_ShootNum_4, "974", 2, 4, 8, 8, 1655, 675, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
		Circle_Draw(&UI_ShootNum_3, "973", 2, 4, 8, 8, 1655, 600, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
		Circle_Draw(&UI_ShootNum_2, "972", 2, 4, 8, 8, 1655, 525, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
		Circle_Draw(&UI_ShootNum_1, "971", 2, 4, 0, 8, 1655, 450, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
	}
	else if(ShootNum > 500)
	{
		Circle_Draw(&UI_ShootNum_5, "975", 2, 4, 8, 8, 1655, 750, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
		Circle_Draw(&UI_ShootNum_4, "974", 2, 4, 8, 8, 1655, 675, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
		Circle_Draw(&UI_ShootNum_3, "973", 2, 4, 8, 8, 1655, 600, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
		Circle_Draw(&UI_ShootNum_2, "972", 2, 4, 8, 8, 1655, 525, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
		Circle_Draw(&UI_ShootNum_1, "971", 2, 4, 8, 8, 1655, 450, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
	}
	Last_Num = ShootNum;
}

/**
 * @brief Pitch轴角度
 * @note  display Pitch Angle
 * @param 
 */
FloInt_Data_t Pitch_Angle;
double Pitch_Last;
void UI_Pitch_Angle(void)
{
	if((INS.Pitch-Pitch_Last)<=0.1 || (INS.Pitch-Pitch_Last)>=-0.1){
		if(stander_system.auto_shoot_mode==0)
			Float_Draw(&Pitch_Angle, "acb", 2, 7, 6, 15, 1, 3, 1050, 540, INS.Pitch);//有变化则修改,青色
		else Float_Draw(&Pitch_Angle, "acb", 2, 7, 5, 15, 1, 3, 1050, 540, INS.Pitch);//有变化则修改,粉色
	}

	Pitch_Last = INS.Pitch;
}


/**
 * @brief 底盘运动模式显示
 * @note  display Action Mode
 * @param 
 */
Graph_Data_t Action_Mode_1;
void UI_Action_Mode(void)
{
	if(stander_system.chassis_mode == no_move)
		Circle_Draw(&Action_Mode_1, "411", 2, 3, 8, 7, 795, 150, 15);//操作2 修改，图层3，颜色7黑色，线宽8，, 半径5
	if(stander_system.chassis_mode == follow_mode)
		Circle_Draw(&Action_Mode_1, "411", 2, 3, 0, 6, 795, 150, 15);//操作2 修改，图层3，颜色6青色，线宽8，, 半径5
	if(stander_system.chassis_mode== around_mode)
		Circle_Draw(&Action_Mode_1, "411", 2, 3, 6, 5, 795, 150, 15);//操作2 修改，图层3，颜色5粉色，线宽8，, 半径5
}


/**
 * @brief power显示
 * @note  display Action Mode
 * @param 
 */
Graph_Data_t shoot_UI;
Graph_Data_t gimbal_UI;
Graph_Data_t chassis_UI;
void UI_Power_Mode(void)
{
	if(Report_IF_Chassis_work() == false)
		Circle_Draw(&chassis_UI, "851", 2, 3, 8, 8, 200, 600, 15);//操作2 修改，图层3，颜色7黑色，线宽8，, 半径5
	else
		Circle_Draw(&chassis_UI, "852", 2, 3, 0, 8, 200, 600, 15);//操作2 修改，图层3，颜色7黑色，线宽8，, 半径5
	if(Report_IF_Gimbal_work() == false)
		Circle_Draw(&gimbal_UI, "852", 2, 3, 8, 8, 275, 600, 15);//操作2 修改，图层3，颜色6青色，线宽8，, 半径5
	else
		Circle_Draw(&gimbal_UI, "851", 2, 3, 0, 8, 275, 600, 15);//操作2 修改，图层3，颜色7黑色，线宽8，, 半径5
	if(Report_IF_ArmorBooster_work() == false)
		Circle_Draw(&shoot_UI, "853", 2, 3, 8, 8, 350, 600, 15);//操作2 修改，图层3，颜色5粉色，线宽8，, 半径5
	else
		Circle_Draw(&shoot_UI, "853", 2, 3, 0, 8, 350, 600, 15);//操作2 修改，图层3，颜色7黑色，线宽8，, 半径5
}
/**
 * @brief 摩擦轮显示
 * @note  display Friction
 * @param 
 */
Graph_Data_t Fric_Mode_1;
void UI_Fric_Mode(void)
{
	if(Report_IF_Fric3508_SetSpeed() == YES && stander_system.fire_mode == 1)
		Circle_Draw(&Fric_Mode_1, "511", 2, 4, 0, 8, 905, 150, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
	if(Report_IF_Fric3508_SetSpeed() == NO || stander_system.fire_mode == 0)
		Circle_Draw(&Fric_Mode_1, "511", 2, 4, 8, 8, 905, 150, 15);//操作2 修改，图层4，颜色7黑色，线宽8，, 半径5
}

/**
 * @brief 控制模式
 * @note  display Friction
 * @param 
 */
Graph_Data_t Ctrl_Mode_1;
void UI_Ctrl_Mode(void)
{
	if(stander_system.control_mode == rc_mode)
		Circle_Draw(&Ctrl_Mode_1, "888", 2, 4, 8, 8, 425, 600, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
	if(stander_system.control_mode == pc_mode)
		Circle_Draw(&Ctrl_Mode_1, "888", 2, 4, 0, 8, 425, 600, 15);//操作2 修改，图层4，颜色7黑色，线宽8，, 半径5
}

/**
 * @brief 摩擦轮显示
 * @note  display Friction
 * @param 
 */
Graph_Data_t Pitch_Mode;
void UI_Pitch_Mode(void)
{
	if(stander_system.gimbal_back == 0)
		Circle_Draw(&Pitch_Mode, "814", 2, 4, 0, 8, 955, 75, 15);//操作2 修改，图层4，颜色6青色，线宽8，, 半径5
	if(stander_system.gimbal_back == 1)
		Circle_Draw(&Pitch_Mode, "814", 2, 4, 8, 8, 955, 75, 15);//操作2 修改，图层4，颜色7黑色，线宽8，, 半径5
}

/**
 * @brief 发弹显示
 * @note  display Shoot
 * @param 
 */
#define SinFire 0	//单发
#define ConFire 1//连发
Graph_Data_t Shoot_Mode_1;
void UI_Shoot_Mode(void)
{
	if(trig.fire_state == fire_sin&&trig.if_block ==0)
		Circle_Draw(&Shoot_Mode_1, "611", 2, 5, 1, 8, 1015, 150, 15);//操作2 修改，图层5，颜色6青色，线宽8，, 半径5
	if(trig.fire_state == fire_con&&trig.if_block ==0)
		Circle_Draw(&Shoot_Mode_1, "611", 2, 5, 5, 8, 1015, 150, 15);//操作2 修改，图层5，颜色5粉色，线宽8，, 半径5
	if(trig.fire_state == fire_no&&trig.if_block ==0)
		Circle_Draw(&Shoot_Mode_1, "611", 2, 5, 8, 8, 1015, 150, 15);//操作2 修改，图层5，颜色6青色，线宽8，, 半径5
  if(trig.if_block == 1)
		Circle_Draw(&Shoot_Mode_1, "611", 2, 5, 7, 8, 1015, 150, 15);//操作2 修改，图层5，颜色7黑色，线宽8，, 半径5
}

/**
 * @brief 加速显示
 * @note  display Shoot
 * @param 
 */
Graph_Data_t Shift_Mode;
void UI_Shift_Mode(void)
{
	if(rc_ctrl.keyboard.key_Shift == 1)
		Circle_Draw(&Shift_Mode, "857", 2, 5, 0, 8, 855, 75, 15);//操作2 修改，图层5，颜色6青色，线宽8，, 半径5
	if(rc_ctrl.keyboard.key_F == 1)
		Circle_Draw(&Shift_Mode, "857", 2, 5, 8, 8, 855, 75, 15);//操作2 修改，图层5，颜色5粉色，线宽8，, 半径5
	if(rc_ctrl.keyboard.key_F == 0 && rc_ctrl.keyboard.key_Shift == 0)
		Circle_Draw(&Shift_Mode, "857", 2, 5, 6, 8, 855, 75, 15);//操作2 修改，图层5，颜色7黑色，线宽8，, 半径5
}

/**
 * @brief pitch使能显示
 * @note  display Shoot
 * @param 
 */
Graph_Data_t DM_Mode;
void UI_DM_Mode(void)
{
	if(pitch_motor.Err==1)
		Circle_Draw(&DM_Mode, "858", 2, 5, 0, 8, 1055, 75, 15);//操作2 修改，图层5，颜色6青色，线宽8，, 半径5
	else
		Circle_Draw(&DM_Mode, "858", 2, 5, 6, 8, 1055, 75, 15);//操作2 修改，图层5，颜色6青色，线宽8，, 半径5
}

/**
 * @brief 电容电压显示
 * @note  display SuperCap voltage
 * @param 
 */
Graph_Data_t SC_Outline_Arc_1;
Graph_Data_t SC_Outline_Arc_2;
Graph_Data_t SC_Outline_Line_1;
Graph_Data_t SC_Outline_Line_2;
Graph_Data_t SC_Vol_Arc;
float CAP_Vol_Angle;
void UI_Cap(void)
{
	
	CAP_Vol_Angle = 135.0f - SuperCAP.C_Vol/23.0f*90.0f;
	if(CAP_Vol_Angle>=134)
		CAP_Vol_Angle = 134;//等于135时ui会变成一个整圆，防止这种情况
	if(SuperCAP.C_Vol>=16)
		Arc_Draw(&SC_Vol_Arc, "715", 2, 6, 6, 5, CAP_Vol_Angle, 135, 960, 540, 298, 298);//操作2 更新，图层6，颜色2绿色，线宽5，起始角度，终点角度，圆心（960，540）,xy半径
	if(SuperCAP.C_Vol<16)
		Arc_Draw(&SC_Vol_Arc, "715", 2, 6, 5, 5, CAP_Vol_Angle, 135, 960, 540, 298, 298);//操作2 更新，图层6，颜色5粉色，线宽5，起始角度，终点角度，圆心（960，540）,xy半径
}
	


/**
 * @brief 特殊运动模式显示
 * @note  Special ACT
 * @param 
 */
 int angle_1;
	int angle_2;
	float ref;
Graph_Data_t Special_ACT_1;//云台朝向
Graph_Data_t Special_ACT_2;//边框圆
Graph_Data_t Special_ACT_3;//灯条1/4圆
//以（1350，230）为圆心
void UI_Special_ACT(void)
{
	
	
	if(stander_system.yaw_diff_radin>=-90&&stander_system.yaw_diff_radin<=180)
		ref = stander_system.yaw_diff_radin - 90.0f;
	else if(stander_system.yaw_diff_radin<-90&&stander_system.yaw_diff_radin>=-180)
		ref = 270.0f + stander_system.yaw_diff_radin;
	
	if(back_stata==forwad)
	{
	ref-=90.0f;
	}
	else if(back_stata==turn)
	{
	ref+=90.0f;
	}
	if(ref>=0&&ref<135){// 0<=ref<=180 时
		angle_1 = -ref+135;
		angle_2 = angle_1+90;
		Arc_Draw(&Special_ACT_3, "913", 2, 1, 6, 6, angle_1, angle_2, 570, 230, 35, 35 );	//操作2 更改，图层1，颜色0主色，线宽6，半径35
	}
	if(ref>=135&&ref<=180){// 0<=ref<=180 时
		angle_1 = -ref+45+15;
		angle_2 = angle_1-90;
		Arc_Draw(&Special_ACT_3, "913", 2, 1, 6, 6, angle_2, angle_1, 570, 230, 35, 35 );	//操作2 更改，图层1，颜色0主色，线宽6，半径35
	}
	if(ref<0){// -180<=ref<0 时&&ref>=-180
		ref = ref;
		angle_1 = -ref+135;
		angle_2 = angle_1+90;
		Arc_Draw(&Special_ACT_3, "913", 2, 1, 6, 6, angle_1, angle_2, 570, 230, 35, 35 );	//操作2 更改，图层1，颜色0主色，线宽6，半径35
	}
}



/*-----------------------------------------整活层-----------------------------------------*/
/**
 * @brief NAUTILUS之LOGO显示
 * @note  
 * @param 
 */
int LOGO_cnt;
void UI_NAUTILUS_LOGO(void)
{
	if(rc_ctrl.keyboard.flag_Z == 1)
	{
		if(LOGO_cnt == 6){
			UI_LOGO_1();
			UI_SendGraph(5, LOGO_1_1, LOGO_1_2, LOGO_1_3, LOGO_1_1, LOGO_1_1);
		}
		if(LOGO_cnt == 5){
			UI_LOGO_2();
			UI_SendGraph(5, LOGO_2_1, LOGO_2_2, LOGO_2_3, LOGO_2_4, LOGO_2_1);
		}
		if(LOGO_cnt == 4){
			UI_LOGO_3();
			UI_SendGraph(5, LOGO_3_1, LOGO_3_2, LOGO_3_3, LOGO_3_4, LOGO_3_1);
		}
		if(LOGO_cnt == 3){
			UI_LOGO_4();
			UI_SendGraph(5, LOGO_4_1, LOGO_4_2, LOGO_4_3, LOGO_4_4, LOGO_4_1);
		}
		if(LOGO_cnt == 2){
			UI_LOGO_5();
			UI_SendGraph(5, LOGO_5_1, LOGO_5_2, LOGO_5_3, LOGO_5_4, LOGO_5_1);
		}
		if(LOGO_cnt == 1){
			UI_LOGO_6();
			UI_SendGraph(5, LOGO_6_1, LOGO_6_2, LOGO_6_3, LOGO_6_4, LOGO_6_1);
		}
		if(LOGO_cnt == 0){
			UI_LOGO_7();
			UI_SendGraph(5, LOGO_7_1, LOGO_7_2, LOGO_7_3, LOGO_7_4, LOGO_7_1);
		}
		if(LOGO_cnt == 7){
			UI_LOGO_8();
			UI_SendGraph(1, LOGO_8_1);
		}
		if(LOGO_cnt >= 8){
			UI_LOGO_9();
		}
		LOGO_cnt++;
	}
	else
	{
		UI_Delete( 1, 8 );
		UI_Delete( 1, 9 );
		LOGO_cnt = 0;
	}
}


/**
 * @brief LOGO的第一部分
 * @note  LOGO_1
 * @param 
 */
Graph_Data_t LOGO_1_1;//直线      
Graph_Data_t LOGO_1_2;//直线
Graph_Data_t LOGO_1_3;//直线
void UI_LOGO_1(void)
{
	Line_Draw( &LOGO_1_1, "811", 1, 8, 6, 4, 802, 440, 848, 672 );//操作1(添加), 图层8, 颜色6(青色), 线宽2
	Line_Draw( &LOGO_1_2, "812", 1, 8, 6, 4, 848, 672, 913, 623 );
	Line_Draw( &LOGO_1_3, "813", 1, 8, 6, 4, 913, 623, 802, 440 );
}

/**
 * @brief LOGO的第二部分
 * @note  LOGO_2
 * @param 
 */
Graph_Data_t LOGO_2_1;//直线
Graph_Data_t LOGO_2_2;//直线
Graph_Data_t LOGO_2_3;//直线
Graph_Data_t LOGO_2_4;//圆弧
void UI_LOGO_2(void)
{
	Line_Draw( &LOGO_2_1, "821", 1, 8, 6, 4, 919, 634, 858, 683 );//操作1(添加), 图层8, 颜色6(青色), 线宽2
	Line_Draw( &LOGO_2_2, "822", 1, 8, 6, 4, 858, 683, 934, 742 );
	Line_Draw( &LOGO_2_3, "823", 1, 8, 6, 4, 934, 742, 952, 652 );
	Arc_Draw( &LOGO_2_4, "824", 1, 8, 6, 4, 310, 344, 971, 588, 66, 66 );//310度-344度
}

/**
 * @brief LOGO的第三部分
 * @note  LOGO_3
 * @param 
 */
Graph_Data_t LOGO_3_1;//直线
Graph_Data_t LOGO_3_2;//直线
Graph_Data_t LOGO_3_3;//直线
Graph_Data_t LOGO_3_4;//圆弧
void UI_LOGO_3(void)
{
	Line_Draw( &LOGO_3_1, "831", 1, 8, 6, 4, 962, 655, 948, 732 );//操作1(添加), 图层8, 颜色6(青色), 线宽2
	Line_Draw( &LOGO_3_2, "832", 1, 8, 6, 4, 948, 732, 1047, 730 );
	Line_Draw( &LOGO_3_3, "833", 1, 8, 6, 4, 1047, 730, 1000, 648 );
	Arc_Draw( &LOGO_3_4, "834", 1, 8, 6, 4, 352, 386, 971, 588, 66, 66 );//352, 386
}

/**
 * @brief LOGO的第四部分
 * @note  LOGO_4
 * @param 
 */
Graph_Data_t LOGO_4_1;//直线
Graph_Data_t LOGO_4_2;//直线
Graph_Data_t LOGO_4_3;//直线
Graph_Data_t LOGO_4_4;//圆弧
void UI_LOGO_4(void)
{
	Line_Draw( &LOGO_4_1, "841", 1, 8, 6, 4, 1008, 641, 1049, 709 );//操作1(添加), 图层8, 颜色6(青色), 线宽2
	Line_Draw( &LOGO_4_2, "842", 1, 8, 6, 4, 1049, 709, 1121, 638 );
	Line_Draw( &LOGO_4_3, "843", 1, 8, 6, 4, 1121, 638, 1030, 613 );
	Arc_Draw( &LOGO_4_4, "844", 1, 8, 6, 4, 34, 68, 971, 588, 66, 66 );//34, 68
}

/**
 * @brief LOGO的第五部分
 * @note  LOGO_5
 * @param 
 */
Graph_Data_t LOGO_5_1;//直线
Graph_Data_t LOGO_5_2;//直线
Graph_Data_t LOGO_5_3;//直线
Graph_Data_t LOGO_5_4;//圆弧
void UI_LOGO_5(void)
{
	Line_Draw( &LOGO_5_1, "851", 1, 8, 6, 4, 1034, 602, 1111, 624 );//操作1(添加), 图层8, 颜色6(青色), 线宽2
	Line_Draw( &LOGO_5_2, "852", 1, 8, 6, 4, 1111, 624, 1115, 523 );
	Line_Draw( &LOGO_5_3, "853", 1, 8, 6, 4, 1115, 523, 1031, 563 );
	Arc_Draw( &LOGO_5_4, "854", 1, 8, 6, 4, 78, 112, 971, 588, 66, 66 );//78, 112
}

/**
 * @brief LOGO的第六部分
 * @note  LOGO_6
 * @param 
 */
Graph_Data_t LOGO_6_1;//直线
Graph_Data_t LOGO_6_2;//直线
Graph_Data_t LOGO_6_3;//直线
Graph_Data_t LOGO_6_4;//圆弧
void UI_LOGO_6(void)
{
	Line_Draw( &LOGO_6_1, "861", 1, 8, 6, 4, 1025, 554, 1101, 520 );//操作1(添加), 图层8, 颜色6(青色), 线宽2
	Line_Draw( &LOGO_6_2, "862", 1, 8, 6, 4, 1101, 520, 1037, 440 );
	Line_Draw( &LOGO_6_3, "863", 1, 8, 6, 4, 1037, 440, 999, 529 );
	Arc_Draw( &LOGO_6_4, "864", 1, 8, 6, 4, 122, 155, 971, 588, 66, 66 );//122, 155
}

/**
 * @brief LOGO的第七部分
 * @note  LOGO_7
 * @param 
 */
Graph_Data_t LOGO_7_1;//直线
Graph_Data_t LOGO_7_2;//直线
Graph_Data_t LOGO_7_3;//直线
Graph_Data_t LOGO_7_4;//圆弧
void UI_LOGO_7(void)
{
	Line_Draw( &LOGO_7_1, "871", 1, 8, 6, 4, 991, 524, 1024, 436 );//操作1(添加), 图层8, 颜色6(青色), 线宽2
	Line_Draw( &LOGO_7_2, "872", 1, 8, 6, 4, 1024, 436, 913, 436 );
	Line_Draw( &LOGO_7_3, "873", 1, 8, 6, 4, 913, 436, 965, 522 );
	Arc_Draw( &LOGO_7_4, "874", 1, 8, 6, 4, 164, 186, 971, 588, 66, 66 );//78, 112
}

/**
 * @brief LOGO的第八部分
 * @note  LOGO_8
 * @param 
 */
Graph_Data_t LOGO_8_1;//正圆
void UI_LOGO_8(void)
{
	Circle_Draw( &LOGO_8_1, "881", 1, 8, 6, 4, 971, 588, 53 );//操作1(添加), 图层8, 颜色6(青色), 线宽2
}


/**
 * @brief LOGO的第九部分
 * @note  LOGO_9
 * @param 
 */
String_Data_t LOGO_9_1;
String_Data_t LOGO_9_2;
String_Data_t LOGO_9_3;
String_Data_t LOGO_9_4;
String_Data_t LOGO_9_5;
String_Data_t LOGO_9_6;
String_Data_t LOGO_9_7;
String_Data_t LOGO_9_8;
char *Name;
void UI_LOGO_9(void)
{
	if(LOGO_cnt == 9){
		Name = "N";
		Char_Draw( &LOGO_9_1, "891", 1, 9, 8, 8, 60, 1, 732, 418, Name );
		UI_SendChars(&LOGO_9_1);
	}
	if(LOGO_cnt == 10){
		Name = "A";
		Char_Draw( &LOGO_9_2, "892", 1, 9, 8, 8, 60, 1, 792, 418, Name );
		UI_SendChars(&LOGO_9_2);
	}
	if(LOGO_cnt == 11){
		Name = "U";
		Char_Draw( &LOGO_9_3, "893", 1, 9, 8, 8, 60, 1, 852, 418, Name );
		UI_SendChars(&LOGO_9_3);
	}
	if(LOGO_cnt == 12){
		Name = "T";
		Char_Draw( &LOGO_9_4, "894", 1, 9, 8, 8, 60, 1, 912, 418, Name );
		UI_SendChars(&LOGO_9_4);
	}
	if(LOGO_cnt == 13){
		Name = "I";
		Char_Draw( &LOGO_9_5, "895", 1, 9, 8, 8, 60, 1, 972, 418, Name );
		UI_SendChars(&LOGO_9_5);
	}
	if(LOGO_cnt == 14){
		Name = "L";
		Char_Draw( &LOGO_9_6, "896", 1, 9, 8, 8, 60, 1, 1032, 418, Name );
		UI_SendChars(&LOGO_9_6);
	}
	if(LOGO_cnt == 15){
		Name = "U";
		Char_Draw( &LOGO_9_7, "897", 1, 9, 8, 8, 60, 1, 1092, 418, Name );
		UI_SendChars(&LOGO_9_7);
	}
	if(LOGO_cnt == 16){
		Name = "S";
		Char_Draw( &LOGO_9_8, "898", 1, 9, 8, 8, 60, 1, 1152, 418, Name );
		UI_SendChars(&LOGO_9_8);
		LOGO_cnt = 0;
	}
}