/**
  ************************************* Copyright ****************************** 
  * FileName   : Nautilus_Vision.c   
  * Version    : v1.0		
  * Author     : 王子佩
  * last_Author: 周灿
  * Number     : 18602780430 	
  * Date       : 2023-12-26         
  * Description:    
  * Function List:  
  	1. ....
  	   <version>: 		
  <modify staff>:
  		  <data>:
   <description>:  
  	2. ...
  ******************************************************************************
 *
*******************************************************************/

#include "Nautilus_Vision.h"
#include  "ins_task.h"
#include "usart.h"
#include "referee.h"
#include "gimbal.h"
#include <stdlib.h>
#include "usbd_cdc_if.h"
#include "cmsis_os.h"
#include "usb_device.h"
#include "user_lib.h"
#include "CAN_receive.h"
#include "crc8_crc16.h"
#include "kalman.h"
#include "bsp_dwt.h"
#include "system.h"
#include <math.h>

//Tx_Data_t Tx_Data;
//Rx_Data_t Rx_Data;

Vision_Send_Data_t Send_Vision;
Vision_rx_Data_t Rx_Vision;
Vision_state_t Vision_state;

first_order_filter_type_t vision_filter_yaw;
first_order_filter_type_t vision_filter_pitch;

pid_struct_t pid_Vision_Pitch_angle;   //Pitch轴云台角度PID结构体
pid_struct_t pid_Vision_Pitch_speed;   //Pitch轴云台速度PID结构体
pid_struct_t pid_Vision_Yaw_angle;     //Yaw轴云台角度PID结构体
pid_struct_t pid_Vision_Yaw_speed;     //Yaw轴云台速度PID结构体

pid_struct_t pid_outpost_angle;
pid_struct_t pid_outpost_speed;

pid_struct_t pid_Curise_Pitch_angle;   //Pitch轴云台角度PID结构体
pid_struct_t pid_Curise_Pitch_speed;   //Pitch轴云台速度PID结构体
pid_struct_t pid_Curise_Yaw_angle;     //Yaw轴云台角度PID结构体
pid_struct_t pid_Curise_Yaw_speed;     //Yaw轴云台速度PID结构体

/**
  * @Name    Vision_Gimbal_Init
  * @brief   视觉PID初始化
  * @param   : [输入/出] 
  * @Data    2024-04-10
*/

void Vision_Gimbal_Init()
{


//    pid_init( &pid_Vision_Pitch_angle,0.5,0.001,0.00,0.08,5);      //6.0,0,0 ,0,0 ,500//1.2,0.01,0,0,1,200//1.1,0.01,0,0,1,100
//    pid_init( &pid_Vision_Pitch_speed,0.5,0.003,0.0,0.07, 8);      //0.5,0.00 ,0 ,0, 0 ,10//0.7,0.00,0,0, 0, 10//0.8,0.001,0,0,1, 10





//   pid_init( &pid_Vision_Yaw_angle,5.5, 0.01, 0.0,4,400);       //2.8, 0.0009, 5.0 ,
//   pid_init( &pid_Vision_Yaw_speed,440, 5, 0,700,29000 );        //2600	
//		pid_init( &pid_Vision_Yaw_speed,500, 3, 0,700,29000 );
    
    
    
    pid_init( &pid_Vision_Pitch_angle,0.5,0.001,0.00,0.15,5);      //6.0,0,0 ,0,0 ,500//1.2,0.01,0,0,1,200//1.1,0.01,0,0,1,100
    pid_init( &pid_Vision_Pitch_speed,0.5,0.01,0.0,0.3, 8);      //0.5,0.00 ,0 ,0, 0 ,10//0.7,0.00,0,0, 0, 10//0.8,0.001,0,0,1, 10


   pid_init( &pid_Vision_Yaw_angle,6.3, 0.001, 0.0,1,40);       //2.8, 0.0009, 5.0 ,
   pid_init( &pid_Vision_Yaw_speed,500, 2, 0,700,29000 );        //2600	
    
    
    
    
    
    
    
//    pid_init( &pid_Vision_Pitch_angle,0.4,0.001,0.00,0.08,5);      //6.0,0,0 ,0,0 ,500//1.2,0.01,0,0,1,200//1.1,0.01,0,0,1,100
//    pid_init( &pid_Vision_Pitch_speed,0.5,0.007,0.0,0.1, 8);      //0.5,0.00 ,0 ,0, 0 ,10//0.7,0.00,0,0, 0, 10//0.8,0.001,0,0,1, 10


   pid_init( &pid_outpost_angle,4.0, 0.09, 0.0,2,400);       //2.8, 0.0009, 5.0 ,
   pid_init( &pid_outpost_speed,400, 8, 0,1500,29000 );        //2600	
    
   float first_kp=1.3f;

	  first_order_filter_init( &vision_filter_yaw,  0.24f ,   &first_kp );
	  first_order_filter_init( &vision_filter_pitch, 0.08f ,   &first_kp );
	
		KalmanCreate(&test_yaw,0.25f,11.0f);
		KalmanCreate(&test_pitch,0.1f,30.0f);
}	



/**
  * @Name    Vision_Tx_Handle
  * @brief   发送数据处理
  * @param   data: [输入/出] 
  * @Data    2024-03-11
*/
uint8_t uuuuu;
void Vision_Tx_Handle(Vision_Send_Data_t *data)
{
	data->head = VISION_TX_HEAD;
	
//	data->robot_color = Recive_data.robot_color ;
//	data->Vision_Mode = Auto.Vision ;
	
  data->robot_color=control_data.robot_color;
  data->Vision_Mode=1;
  
	data->roll = INS.Pitch  * 1000.0;
	data->pitch = INS.Roll * 1000.0;
	data->yaw = INS.Yaw  * 1000.0;
//	data->roll = 0;
//	data->pitch = 0;
//	data->yaw = 0;
 data->shoot_speed=0;
 
 
 
//  data->shoot_all      = 0;
//  data->shoot_Hero     = 1;
//  data->shoot_Engineer = 1;
//  data->shoot_balance  = 1;
//  data->shoot_infantr4 = 1;
//  data->shoot_infantr5 = 1;
//  data->shoot_Sentry   = 1;
//  data->shoot_outpost=1;
//  data->shoot_base   =1;
    data->if_outpost_lock=control_data.other_data.outpost_state;
//    data->top_senior=control_data.other_data.top_senior_priority;
    
       
    
    //测试用
//    data->if_outpost_lock=0;
    data->top_senior=ARMOR_ENGINEER;
    data->shoot_all=0;
    data->shoot_Hero=control_data.Vision_ByteBits.shoot_Hero;
    data->shoot_Engineer=control_data.Vision_ByteBits.shoot_Engineer;
    data->shoot_balance=control_data.Vision_ByteBits.shoot_infantr3;
    data->shoot_infantr4=control_data.Vision_ByteBits.shoot_infantr4;
    data->shoot_Sentry=control_data.Vision_ByteBits.shoot_Sentry;
    data->shoot_outpost=control_data.Vision_ByteBits.shoot_outpost;
    data->shoot_base   =control_data.Vision_ByteBits.shoot_base;
  	data->tail = VISION_TX_TAIL;
}
int bbbb;
int nnnn;
/**
  * @Name    Vision_Sand_Data
* @brief     发送数据给视觉
  * @param   data: [输入/出] 
  * @Data    2024-03-11
*/

uint8_t buff1[VISION_TX_COUNT];
void Vision_Send_Data(Vision_Send_Data_t *data)
{
	Vision_Tx_Handle(data);

	memcpy( buff1  , &data->head , 1);
	memcpy( buff1+1, &data->robot_color , 1);
	memcpy( buff1+2, &data->top_senior , 1);//&data->pitch
	memcpy( buff1+3, &data->roll  , 4);
	memcpy( buff1+7, &data->pitch , 4);
	memcpy( buff1+11, &data->yaw  , 4);
	memcpy( buff1+15, &data->shoot_Hero , 1);
	memcpy( buff1+16, &data->shoot_Engineer , 1);
	memcpy( buff1+17, &data->shoot_balance , 1);
	memcpy( buff1+18, &data->shoot_infantr4 , 1);
	memcpy( buff1+19, &data->shoot_Sentry , 1);
	memcpy( buff1+20, &data->shoot_outpost , 1);
  memcpy( buff1+21, &data->if_outpost_lock,1);
	memcpy( buff1+22, &data->tail , 1);
  
uuuuu++;
  
	bbbb=CDC_Transmit_FS(buff1, sizeof(buff1));
}













/********************同济视觉通信************************/

uint8_t Tj_Tx_data[43];
uint8_t Tj_Rx_data[29];
TJ_Vision_Rx_t TJ_Vision_Rx;
TJ_Vision_Tx_t TJ_Vision_Tx;
/**
  * @Name    TJ_Vision_Tx
  * @brief   发送数据处理
  * @param   data: [输入/出] 
  * @Data    2024-03-11
*/
uint8_t uuuuu;
void TJ_Tx_Handle(TJ_Vision_Tx_t *data)
{
  data->head[0]='S';
  data->head[1]='P';
  data->Vision_Tx_gimbal_mode=gimbal_armor;
  data->q[0]=INS.q[0];
  data->q[1]=INS.q[1];
  data->q[2]=INS.q[2];
  data->q[3]=INS.q[3];
  data->yaw=INS.Yaw*0.01745;
  data->yaw_vel=INS.Gyro[2];
  data->pitch=INS.Roll*0.01745;
  data->pitch_vel=INS.Gyro[1];
  data->fire_speed=25;
  data->fire_count=control_data.shoot_num;
  
}
USBD_StatusTypeDef USBD_StatusTypeDef_t;

void Tj_Send_Data(TJ_Vision_Tx_t *data)
{
  TJ_Tx_Handle(data);
  
  
  memcpy(Tj_Tx_data    ,&data->head,2);
  memcpy(Tj_Tx_data + 2 , &data->Vision_Tx_gimbal_mode,1);
  
  memcpy(Tj_Tx_data + 3 , &data->q[0] ,4);
  memcpy(Tj_Tx_data + 7 , &data->q[1] ,4);
  memcpy(Tj_Tx_data + 11 ,&data->q[2] ,4);
  memcpy(Tj_Tx_data + 15 ,&data->q[3] ,4);
  memcpy(Tj_Tx_data + 19 ,&data->yaw  ,4 );
  memcpy(Tj_Tx_data + 23 ,&data->yaw_vel  ,4 );
  memcpy(Tj_Tx_data + 27 ,&data->pitch  , 4);
  memcpy(Tj_Tx_data + 31 ,&data->pitch_vel  ,4 );
  memcpy(Tj_Tx_data + 35 ,&data->fire_speed  ,4 );
  memcpy(Tj_Tx_data + 39 ,&data->fire_count  ,2 );
  
  append_CRC16_check_sum(Tj_Tx_data,43);
  
 USBD_StatusTypeDef_t= CDC_Transmit_FS(Tj_Tx_data, sizeof(Tj_Tx_data));
}


void Tj_Receive_Data(TJ_Vision_Rx_t *data,uint8_t *buff,uint32_t Len)
{

	data->head[0]=buff[0];
  data->head[1]=buff[1];
  
  if(data->head[0]=='S'&&data->head[1]=='P')
  {
     communication_frame_rate_update(VISION_TOE);
     
     
     
     
     Algorithm_float_u yaw,yaw_speed,yaw_acc,pitch,pitch_speed,pitch_acc;
     
     for(int i=0;i<4;i++)
     {
     
       yaw.d[i]=buff[i+3];
       yaw_speed.d[i]=buff[i+7];
       yaw_acc.d[i]=buff[i+11];
       pitch.d[i]=buff[i+15];
       pitch_speed.d[i]=buff[i+19];
       pitch_acc.d[i]=buff[i+23];
       
     }
     data->Vision_gimbal_mode=buff[2];
     data->yaw=yaw.data;
     data->yaw_vel=yaw_speed.data;
     data->yaw_acc=yaw_acc.data;
     data->pitch=pitch.data;
     data->pitch_vel=pitch_speed.data;
     data->pitch_acc=pitch_acc.data;
     data->yaw_add = data->yaw - INS.Yaw;
    
    
    if(data->yaw_add>180)
    {
      data->yaw_add-=360.0;
    }
    else if(data->yaw_add<-180.0)
    {
     data->yaw_add+=360.0f;
    }
     
     data->total_yaw=data->yaw_add+INS.YawTotalAngle;
     
  }
	

		
}


///**
//  * @Name    Vision_Tx_Handle
//  * @brief   发送数据处理
//  * @param   data: [输入/出] 
//  * @Data    2024-03-11
//*/
//uint8_t uuuuu;
//void Vision_Tx_Handle(Vision_Send_Data_t *data)
//{
//	data->head = VISION_TX_HEAD;
//	
////	data->robot_color = Recive_data.robot_color ;
////	data->Vision_Mode = Auto.Vision ;
//	
//  data->robot_color=control_data.robot_color;
//  data->Vision_Mode=1;
//  
//	data->roll = INS.Pitch * 1000.0;
//	data->pitch = INS.Roll * 1000.0;
//	data->yaw = INS.Yaw  * 1000.0;
////	data->roll = 0;
////	data->pitch = 0;
////	data->yaw = 0;
// data->shoot_speed=(control_data.shoot_speed-20)*10;
// 
////  data->shoot_all      = 0;
////  data->shoot_Hero     = 1;
////  data->shoot_Engineer = 1;
////  data->shoot_balance  = 1;
////  data->shoot_infantr4 = 1;
////  data->shoot_infantr5 = 1;
////  data->shoot_Sentry   = 1;
////  data->shoot_outpost=0;
////  data->shoot_base   =1;

//    data->shoot_all=0;
//    data->shoot_Hero=control_data.Vision_ByteBits.shoot_Hero;
//    data->shoot_Engineer=control_data.Vision_ByteBits.shoot_Engineer;
//    data->shoot_balance=control_data.Vision_ByteBits.shoot_infantr3;
//    data->shoot_infantr4=control_data.Vision_ByteBits.shoot_infantr3;
//    data->shoot_Sentry=control_data.Vision_ByteBits.shoot_Sentry;
//    data->shoot_outpost=1;
//    data->shoot_base   =control_data.Vision_ByteBits.shoot_base;
//  	data->tail = VISION_TX_TAIL;
//}
//int bbbb;
//int nnnn;
///**
//  * @Name    Vision_Sand_Data
//* @brief     发送数据给视觉
//  * @param   data: [输入/出] 
//  * @Data    2024-03-11
//*/

//uint8_t buff1[VISION_TX_COUNT];
//void Vision_Send_Data(Vision_Send_Data_t *data)
//{
//	Vision_Tx_Handle(data);

//	memcpy( buff1  , &data->head , 1);
//	memcpy( buff1+1, &data->robot_color , 1);
//	memcpy( buff1+2, &data->Vision_Mode , 1);//&data->pitch
//	memcpy( buff1+3, &data->roll  , 4);
//	memcpy( buff1+7, &data->pitch , 4);
//	memcpy( buff1+11, &data->yaw  , 4);
//	memcpy( buff1+15, &data->shoot_all , 1);
//	memcpy( buff1+16, &data->shoot_Hero , 1);
//	memcpy( buff1+17, &data->shoot_Engineer , 1);
//	memcpy( buff1+18, &data->shoot_balance , 1);
//	memcpy( buff1+19, &data->shoot_infantr4 , 1);
//	memcpy( buff1+20, &data->shoot_Sentry , 1);
//	memcpy( buff1+21, &data->shoot_outpost , 1);
//  memcpy( buff1+22, &data->shoot_speed,1);
//	memcpy( buff1+23, &data->tail , 1);
//  
//uuuuu++;
//  
//	bbbb=CDC_Transmit_FS(buff1, sizeof(buff1));
//}

/**
  * @Name    Rx_Vision_Handle
  * @brief   接收视觉数据处理
  * @param   data: [输入/出] 
**			 buff: [输入/出] 
**			 Len: [输入/出] 
  * @Data    2024-03-11
*/
uint8_t buff0[VISION_RX_COUNT];
uint8_t Last_Vision_seq;
float last_pitch;

float diff_time;
uint32_t time;
void Rx_Vision_Handle(Vision_rx_Data_t *data,uint8_t *buff,uint32_t Len)
{nnnn++;
//	for(int i = 0;i < VISION_RX_COUNT; i++)
//	{
//		buff0[i] = buff[i];
//	}
	
	data->head = buff[0];
	data->tail = buff[Len-1];
	
	if(data->head == VISION_RX_HEAD && data->tail == VISION_RX_TAIL)
	{
		Algorithm_float_u _pitch,_yaw;
		
		for(int i = 0; i < 4; i++)
		{
			_pitch.d[i] = buff[i+1];
			_yaw.d[i]   = buff[i+5];
		}
		data->pitch_obj = _pitch.data /1000.0 ;///1000.0 *0.2
		data->yaw_obj = _yaw.data /1000.0;
		data->Flag_Found = buff[9] ;
		data->armor_id = buff[10] ;
		data->fire = buff[11] ;
		data->Fire_Mode = buff[12];
		data->Vision_seq = buff[Len-5];
//		data->IF_stay = buff[Len-4];
    data->IF_stay = 0;
    data->Back_Found = buff[Len-3];
    data->armor_dist=buff[Len-2];
    data->yaw_add = data->yaw_obj - INS.Yaw;
    
    
    if(data->yaw_add>180)
    {
      data->yaw_add-=360.0;
    }
    else if(data->yaw_add<-180.0)
    {
     data->yaw_add+=360.0f;
    }
     
     data->total_yaw=data->yaw_add+INS.YawTotalAngle;
     if(data->Vision_seq!=Last_Vision_seq)
     {
       communication_frame_rate_update(VISION_TOE);
     }
     
     
     

	}
	Last_Vision_seq = data->Vision_seq;
		
}




//struct SolveTrajectoryParams st;
//struct tar_pos tar_position[4]; //最多只有四块装甲板
//float t = 0.5f; // 飞行时间


///**
//@brief 单方向空气阻力弹道模型
//@param s:m 距离
//@param v:m/s 速度
//@param angle:rad 角度
//@return z:m
//*/
//float monoDirectionalAirResistanceModel(float s, float v, float angle)
//{
//    float z;
//    //t为给定v与angle时的飞行时间
//    t = (float)((exp(st.k * s) - 1) / (st.k * v * cos(angle)));
//    //z为给定v与angle时的高度
//    z = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
//    return z;
//}


///**
//@brief 完整弹道模型
//@param s:m 距离
//@param v:m/s 速度
//@param angle:rad 角度
//@return z:m
//*/
////TODO 完整弹道模型
////float completeAirResistanceModel(float s, float v, float angle)
////{
////    continue;


////}



///**
//@brief pitch轴解算
//@param s:m 距离
//@param z:m 高度
//@param v:m/s
//@return angle_pitch:rad
//*/
//float pitchTrajectoryCompensation(float s, float z, float v)
//{
//    float z_temp, z_actual, dz;
//    float angle_pitch;
//    int i = 0;
//    z_temp = z;
//    // iteration
//    for (i = 0; i < 20; i++)
//    {
//        angle_pitch = atan2(z_temp, s); // rad
//        z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch);
//        dz = 0.3*(z - z_actual);
//        z_temp = z_temp + dz;
//        if (fabsf(dz) < 0.00001)
//        {
//            break;
//        }
//    }
//    return angle_pitch;
//}

///**
//@brief 根据最优决策得出被击打装甲板 自动解算弹道
//@param pitch:rad  传出pitch
//@param yaw:rad    传出yaw
//@param aim_x:传出aim_x  打击目标的x
//@param aim_y:传出aim_y  打击目标的y
//@param aim_z:传出aim_z  打击目标的z
//*/
//void autoSolveTrajectory(float *pitch, float *yaw, float *aim_x, float *aim_y, float *aim_z)
//{

//    // 线性预测
//    float timeDelay = st.bias_time/1000.0 + t;
//    st.tar_yaw += st.v_yaw * timeDelay;

//    //计算四块装甲板的位置
//    //装甲板id顺序，以四块装甲板为例，逆时针编号
//    //      2
//    //   3     1
//    //      0
//	int use_1 = 1;
//	int i = 0;
//    int idx = 0; // 选择的装甲板
//    //armor_num = ARMOR_NUM_BALANCE 为平衡步兵
//    if (st.armor_num == ARMOR_NUM_BALANCE) {
//        for (i = 0; i<2; i++) {
//            float tmp_yaw = st.tar_yaw + i * PI;
//            float r = st.r1;
//            tar_position[i].x = st.xw - r*cos(tmp_yaw);
//            tar_position[i].y = st.yw - r*sin(tmp_yaw);
//            tar_position[i].z = st.zw;
//            tar_position[i].yaw = tmp_yaw;
//        }

//        float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);

//        //因为是平衡步兵 只需判断两块装甲板即可
//        float temp_yaw_diff = fabsf(*yaw - tar_position[1].yaw);
//        if (temp_yaw_diff < yaw_diff_min)
//        {
//            yaw_diff_min = temp_yaw_diff;
//            idx = 1;
//        }


//    } else if (st.armor_num == ARMOR_NUM_OUTPOST) {  //前哨站
//        for (i = 0; i<3; i++) {
//            float tmp_yaw = st.tar_yaw + i * 2.0 * PI/3.0;  // 2/3PI
//            float r =  (st.r1 + st.r2)/2;   //理论上r1=r2 这里取个平均值
//            tar_position[i].x = st.xw - r*cos(tmp_yaw);
//            tar_position[i].y = st.yw - r*sin(tmp_yaw);
//            tar_position[i].z = st.zw;
//            tar_position[i].yaw = tmp_yaw;
//        }

//        //TODO 选择最优装甲板 选板逻辑你们自己写，这个一般给英雄用


//    } else {

//        for (i = 0; i<4; i++) {
//            float tmp_yaw = st.tar_yaw + i * PI/2.0;
//            float r = use_1 ? st.r1 : st.r2;
//            tar_position[i].x = st.xw - r*cos(tmp_yaw);
//            tar_position[i].y = st.yw - r*sin(tmp_yaw);
//            tar_position[i].z = use_1 ? st.zw : st.zw + st.dz;
//            tar_position[i].yaw = tmp_yaw;
//            use_1 = !use_1;
//        }

//            //2种常见决策方案：
//            //1.计算枪管到目标装甲板yaw最小的那个装甲板
//            //2.计算距离最近的装甲板

//            //计算距离最近的装甲板
//        //	float dis_diff_min = sqrt(tar_position[0].x * tar_position[0].x + tar_position[0].y * tar_position[0].y);
//        //	int idx = 0;
//        //	for (i = 1; i<4; i++)
//        //	{
//        //		float temp_dis_diff = sqrt(tar_position[i].x * tar_position[0].x + tar_position[i].y * tar_position[0].y);
//        //		if (temp_dis_diff < dis_diff_min)
//        //		{
//        //			dis_diff_min = temp_dis_diff;
//        //			idx = i;
//        //		}
//        //	}
//        //

//            //计算枪管到目标装甲板yaw最小的那个装甲板
//        float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);
//        for (i = 1; i<4; i++) {
//            float temp_yaw_diff = fabsf(*yaw - tar_position[i].yaw);
//            if (temp_yaw_diff < yaw_diff_min)
//            {
//                yaw_diff_min = temp_yaw_diff;
//                idx = i;
//            }
//        }

//    }


//    *aim_z = tar_position[idx].z + st.vzw * timeDelay;
//    *aim_x = tar_position[idx].x + st.vxw * timeDelay;
//    *aim_y = tar_position[idx].y + st.vyw * timeDelay;
//    //这里符号给错了
//    *pitch = -pitchTrajectoryCompensation(sqrt((*aim_x) * (*aim_x) + (*aim_y) * (*aim_y)) - st.s_bias,
//            *aim_z + st.z_bias, st.current_v);
//    *yaw = (float)(atan2(*aim_y, *aim_x));

//}








