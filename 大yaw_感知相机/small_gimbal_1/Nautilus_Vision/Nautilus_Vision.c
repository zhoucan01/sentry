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
	  pid_init( &pid_Vision_Pitch_angle,0.7,0.0,0.0,0,30);      //6.0,0,0 ,0,0 ,500//1.2,0.01,0,0,1,200//1.1,0.01,0,0,1,100
    pid_init( &pid_Vision_Pitch_speed,0.7,0.0,0.0,0.03, 8);      //0.5,0.00 ,0 ,0, 0 ,10//0.7,0.00,0,0, 0, 10//0.8,0.001,0,0,1, 10
//	
//	
    pid_init( &pid_Vision_Yaw_angle,5.85, 0.0, 0.0,0,400);       //2.8, 0.0009, 5.0 ,
   pid_init( &pid_Vision_Yaw_speed,1000, 0.000, 1,0,26000 );        //2600
//	
//		
//	  pid_init( &pid_Curise_Pitch_angle,0.8,0.01,0,0,1,100);      //6.0,0,0 ,0,0 ,500//1.2,0.01,0,0,1,200//1.1,0.01,0,0,1,100
//    pid_init( &pid_Curise_Pitch_speed,0.5,0.00,0,0, 0,5);      //0.5,0.00 ,0 ,0, 0 ,10//0.7,0.00,0,0, 0, 10//0.8,0.001,0,0,1, 10

//    pid_init( &pid_Curise_Yaw_angle,2.5, 0.000, 0,0,1,300);       //2.8, 0.0009, 5.0 ,
//    pid_init( &pid_Curise_Yaw_speed,800, 0.000, 0.0,0,100,26000 );        //2600
//	
   float first_kp=1.8f;

	  first_order_filter_init( &vision_filter_yaw,  0.22f ,   &first_kp );
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
  
	data->roll = INS.Pitch * 1000.0;
	data->pitch = INS.Roll * 1000.0;
	data->yaw = INS.Yaw  * 1000.0;

	data->shoot_all      = 0;
	data->shoot_Hero     = 1;
	data->shoot_Engineer = 1;
	data->shoot_balance  = 1;
	data->shoot_infantr4 = 1;
	data->shoot_infantr5 = 1;
	data->shoot_Sentry   = 1;
  data->shoot_outpost=1;
    data->shoot_base   =1;

//    data->shoot_all=control_data.Vision_ByteBits.shoot_all;
//    data->shoot_Hero=control_data.Vision_ByteBits.shoot_Hero;
//    data->shoot_Engineer=control_data.Vision_ByteBits.shoot_Engineer;
//    data->shoot_balance=control_data.Vision_ByteBits.shoot_infantr3;
//    data->shoot_infantr4=control_data.Vision_ByteBits.shoot_infantr4;
//    data->shoot_Sentry=control_data.Vision_ByteBits.shoot_Sentry;
//    data->shoot_outpost=control_data.Vision_ByteBits.shoot_outpost;
//    data->shoot_base   =control_data.Vision_ByteBits.shoot_base;
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
	memcpy( buff1+2, &data->Vision_Mode , 1);//&data->pitch
	memcpy( buff1+3, &data->roll  , 4);
	memcpy( buff1+7, &data->pitch , 4);
	memcpy( buff1+11, &data->yaw  , 4);
	memcpy( buff1+15, &data->shoot_all , 1);
	memcpy( buff1+16, &data->shoot_Hero , 1);
	memcpy( buff1+17, &data->shoot_Engineer , 1);
	memcpy( buff1+18, &data->shoot_balance , 1);
	memcpy( buff1+19, &data->shoot_infantr4 , 1);
	memcpy( buff1+20, &data->shoot_infantr5 , 1);
	memcpy( buff1+21, &data->shoot_Sentry , 1);
	memcpy( buff1+22, &data->shoot_outpost , 1);
	memcpy( buff1+23, &data->shoot_base , 1);
	memcpy( buff1+24, &data->tail , 1);
uuuuu++;
  
	bbbb=CDC_Transmit_FS(buff1, sizeof(buff1));
}

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
void Rx_Vision_Handle(Vision_rx_Data_t *data,uint8_t *buff,uint32_t Len)
{nnnn++;
	for(int i = 0;i < VISION_RX_COUNT; i++)
	{
		buff0[i] = buff[i];
	}
	
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
		data->Vision_seq = buff[Len-3];
		data->IF_stay = buff[Len-2];
		
    data->yaw_add=data->yaw_obj-INS.Yaw;
    if(data->yaw_add>180.0f)
    {
    data->yaw_add-=360.0f;
    }
    else if(data->yaw_add<-180.0f)
    {
     data->yaw_add+=360.0f;
    }
    
    if(data->yaw_add>5)
    {
      data->yaw_add=5;
    }
       else if(data->yaw_add<-5)
    {
      data->yaw_add=-5;
    }
    data->total_yaw=data->yaw_add+INS.YawTotalAngle;
    
    communication_frame_rate_update(VISION_TOE);
    
    
//		if( data->Vision_seq == Last_Vision_seq )
//			data->Flag_Found = 0;
	}
	Last_Vision_seq = data->Vision_seq;
		
}

///**
//  * @Name    Recive_Data_Handle
//  * @brief   
//  * @param   buff: [输入/出] 
//  * @Data    2023-12-26
//*/

//void Recive_Data_Handle(uint8_t *buff,uint32_t Len,Rx_Data_t *data)
//{
//	if (buff == NULL || data == NULL)
//    {
//        return ;
//    }
//	
//		if(buff[0] == CONST_HEAD0 && buff[Len-1] == CONST_END0)
//		{
//			Algorithm_float_u Vx, Vy, Wz;
//			for(int i = 0; i < 4; i++)
//			{
//				Vx.d[i]  = buff[1+i];
//				Vy.d[i]  = buff[5+i];
//				Wz.d[i]  = buff[9+i];
//			}
//			data->PC_Move.vx_obj = Vx.data ;
//			data->PC_Move.vy_obj = Vy.data ;
//			data->PC_Move.wz_obj = Wz.data ;
//		}
//		else if(buff[Len-1] == CONST_END1)
//		{
//				if (buff[0] == CONST_HEAD1)
//			{
//				for (int i = 0; i < 49; i++)
//					data->PC_Patth.delta_x[i] = buff[1+i];  // 1 - 49
//			}
//			else if (buff[0] == CONST_HEAD2)
//			{
//				for (int i = 0; i < 49; i++)
//					data->PC_Patth.delta_y[i] = buff[1+i];  // 1 - 49
//			}
//		}
//}



///**
//  * @Name    Serial_Sand_Data
//* @brief     数据发送
//  * @param   None
//  * @Data    2023-12-26
//*/
// 
//void Serial_Send_Data(Auto_t*auto_data,Chassis_Move_t*Move,Tx_Data_t *data)
//{
//		Serial_Data_Handle(auto_data,Move,&Tx_Data);
//		
//		uint8_t buff[SEND_DATA_COUNT];
//	  uint8_t Serial_Seq; 
//	
//	buff[0] = CONST_HEAD0;
//	
//	memcpy( buff+1,  &data->Navi.IF_nav, 1 );
//	memcpy( buff+2,  &data->Navi.pos_x_set, 4 );
//	memcpy( buff+6,  &data->Navi.pos_y_set, 4 );
//	memcpy( buff+10, &data->Navi.yaw_set, 4 );
//	memcpy( buff+14, &data->odom.IF_location, 1 );
//	memcpy( buff+15, &data->odom.pos_x, 4 );
//	memcpy( buff+19, &data->odom.pos_y, 4 );
//	memcpy( buff+23, &data->odom.yaw, 4 );
//	buff[SEND_DATA_COUNT-2] = Serial_Seq;
//	buff[SEND_DATA_COUNT-1] = CONST_END0;
//	
//	CDC_Transmit_FS(buff, SEND_DATA_COUNT);
//	
//	Serial_Seq++;
//}

///**
//  * @Name    Serial_Data_Handle
//  * @brief   串口数据处理
//  * @param   Chassis_Move_t*Move: [输入/出] 
//  * @Data    2023-12-26
//*/

//static void Serial_Data_Handle(Auto_t*auto_data,Chassis_Move_t*Move,Tx_Data_t*data)
//{
//    if (data == NULL)
//    {
//        return;
//    }
//		
//		data->Navi.IF_nav = auto_data->Navi.IF_nav ;//auto_data->Navi.IF_nav
//		data->Navi.pos_x_set = auto_data->Navi.pos_x_set  ;
//		data->Navi.pos_y_set = auto_data->Navi.pos_y_set ;
//		data->Navi.yaw_set = auto_data->Navi.yaw_set ;
//		
//		data->odom.IF_location = auto_data->Odome.IF_location ;
//		data->odom.pos_x = Move->position_x ;
//		data->odom.pos_y = Move->position_y ;
//		data->odom.yaw = Move->turn_angle ;
//		
//}


struct SolveTrajectoryParams st;
struct tar_pos tar_position[4]; //最多只有四块装甲板
float t = 0.5f; // 飞行时间


/**
@brief 单方向空气阻力弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/
float monoDirectionalAirResistanceModel(float s, float v, float angle)
{
    float z;
    //t为给定v与angle时的飞行时间
    t = (float)((exp(st.k * s) - 1) / (st.k * v * cos(angle)));
    //z为给定v与angle时的高度
    z = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
    return z;
}


/**
@brief 完整弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/
//TODO 完整弹道模型
//float completeAirResistanceModel(float s, float v, float angle)
//{
//    continue;


//}



/**
@brief pitch轴解算
@param s:m 距离
@param z:m 高度
@param v:m/s
@return angle_pitch:rad
*/
float pitchTrajectoryCompensation(float s, float z, float v)
{
    float z_temp, z_actual, dz;
    float angle_pitch;
    int i = 0;
    z_temp = z;
    // iteration
    for (i = 0; i < 20; i++)
    {
        angle_pitch = atan2(z_temp, s); // rad
        z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch);
        dz = 0.3*(z - z_actual);
        z_temp = z_temp + dz;
        if (fabsf(dz) < 0.00001)
        {
            break;
        }
    }
    return angle_pitch;
}

/**
@brief 根据最优决策得出被击打装甲板 自动解算弹道
@param pitch:rad  传出pitch
@param yaw:rad    传出yaw
@param aim_x:传出aim_x  打击目标的x
@param aim_y:传出aim_y  打击目标的y
@param aim_z:传出aim_z  打击目标的z
*/
void autoSolveTrajectory(float *pitch, float *yaw, float *aim_x, float *aim_y, float *aim_z)
{

    // 线性预测
    float timeDelay = st.bias_time/1000.0 + t;
    st.tar_yaw += st.v_yaw * timeDelay;

    //计算四块装甲板的位置
    //装甲板id顺序，以四块装甲板为例，逆时针编号
    //      2
    //   3     1
    //      0
	int use_1 = 1;
	int i = 0;
    int idx = 0; // 选择的装甲板
    //armor_num = ARMOR_NUM_BALANCE 为平衡步兵
    if (st.armor_num == ARMOR_NUM_BALANCE) {
        for (i = 0; i<2; i++) {
            float tmp_yaw = st.tar_yaw + i * PI;
            float r = st.r1;
            tar_position[i].x = st.xw - r*cos(tmp_yaw);
            tar_position[i].y = st.yw - r*sin(tmp_yaw);
            tar_position[i].z = st.zw;
            tar_position[i].yaw = tmp_yaw;
        }

        float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);

        //因为是平衡步兵 只需判断两块装甲板即可
        float temp_yaw_diff = fabsf(*yaw - tar_position[1].yaw);
        if (temp_yaw_diff < yaw_diff_min)
        {
            yaw_diff_min = temp_yaw_diff;
            idx = 1;
        }


    } else if (st.armor_num == ARMOR_NUM_OUTPOST) {  //前哨站
        for (i = 0; i<3; i++) {
            float tmp_yaw = st.tar_yaw + i * 2.0 * PI/3.0;  // 2/3PI
            float r =  (st.r1 + st.r2)/2;   //理论上r1=r2 这里取个平均值
            tar_position[i].x = st.xw - r*cos(tmp_yaw);
            tar_position[i].y = st.yw - r*sin(tmp_yaw);
            tar_position[i].z = st.zw;
            tar_position[i].yaw = tmp_yaw;
        }

        //TODO 选择最优装甲板 选板逻辑你们自己写，这个一般给英雄用


    } else {

        for (i = 0; i<4; i++) {
            float tmp_yaw = st.tar_yaw + i * PI/2.0;
            float r = use_1 ? st.r1 : st.r2;
            tar_position[i].x = st.xw - r*cos(tmp_yaw);
            tar_position[i].y = st.yw - r*sin(tmp_yaw);
            tar_position[i].z = use_1 ? st.zw : st.zw + st.dz;
            tar_position[i].yaw = tmp_yaw;
            use_1 = !use_1;
        }

            //2种常见决策方案：
            //1.计算枪管到目标装甲板yaw最小的那个装甲板
            //2.计算距离最近的装甲板

            //计算距离最近的装甲板
        //	float dis_diff_min = sqrt(tar_position[0].x * tar_position[0].x + tar_position[0].y * tar_position[0].y);
        //	int idx = 0;
        //	for (i = 1; i<4; i++)
        //	{
        //		float temp_dis_diff = sqrt(tar_position[i].x * tar_position[0].x + tar_position[i].y * tar_position[0].y);
        //		if (temp_dis_diff < dis_diff_min)
        //		{
        //			dis_diff_min = temp_dis_diff;
        //			idx = i;
        //		}
        //	}
        //

            //计算枪管到目标装甲板yaw最小的那个装甲板
        float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);
        for (i = 1; i<4; i++) {
            float temp_yaw_diff = fabsf(*yaw - tar_position[i].yaw);
            if (temp_yaw_diff < yaw_diff_min)
            {
                yaw_diff_min = temp_yaw_diff;
                idx = i;
            }
        }

    }


    *aim_z = tar_position[idx].z + st.vzw * timeDelay;
    *aim_x = tar_position[idx].x + st.vxw * timeDelay;
    *aim_y = tar_position[idx].y + st.vyw * timeDelay;
    //这里符号给错了
    *pitch = -pitchTrajectoryCompensation(sqrt((*aim_x) * (*aim_x) + (*aim_y) * (*aim_y)) - st.s_bias,
            *aim_z + st.z_bias, st.current_v);
    *yaw = (float)(atan2(*aim_y, *aim_x));

}



