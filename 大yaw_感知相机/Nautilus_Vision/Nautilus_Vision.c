/**
  ************************************* Copyright ****************************** 
  * FileName   : Nautilus_Vision.c   
  * Version    : v1.0		
  * Author     : 王子佩
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
#include "system.h"
#include <stdlib.h>
#include "usbd_cdc_if.h"
#include "usb_device.h"
//#include "Chassis.h"
//#include "Decision.h"
#include "bsp_transmit.h"
#include "referee.h"
#include "stdbool.h"
Tx_Data_t Tx_Data;
Rx_Data_t Rx_Data;

/**
  * @Name    Recive_Data_Handle
  * @brief   算法接收信息处理
  * @param   buff: [输入/出] 
  * @Data    2023-12-26
*/
uint8_t buffr[60];
uint8_t Last_naiv_seq;
int ijn;
void Recive_Data_Handle(Auto_t*ptr,uint8_t *buff,uint32_t Len,Rx_Data_t *data)
{
	for( int i = 0; i < 50;i++)
	{
		buffr[i] = buff[i];
	}
	
	if (buff == NULL || data == NULL)
    {
        return ;
    }
	
		if(buff[0] == CONST_HEAD0 && buff[Len-1] == CONST_END0)
		{
			Algorithm_float_u Vx, Vy, Wz,robot_x,robot_y,robot_yaw;
			for(int i = 0; i < 4; i++)
			{
				Vx.d[i]  = buff[1+i];
				Vy.d[i]  = buff[5+i];
				Wz.d[i]  = buff[9+i];
				
				robot_x.d[i]   = buff[13+i];
				robot_y.d[i]   = buff[17+i];
				robot_yaw.d[i] = buff[21+i];
			}
			data->PC_Move.vx_obj = Vx.data ;
			data->PC_Move.vy_obj = Vy.data ;
			data->PC_Move.wz_obj = Wz.data ;
      
			data->Navi_coord.map_x   = robot_x.data ;
			data->Navi_coord.map_y   = robot_y.data ;
			data->Navi_coord.map_yaw = robot_yaw.data ;
			
			data->navi_state = buff[Len-3];
			data->seq = buff[Len-2];
		}
		else if(buff[Len-1] == CONST_END1)
		{
			Algorithm_16_u _Start_x,_Start_y;
			
				if (buff[0] == CONST_HEAD1)
			{
				for( int i = 0;i < 2;i++ )
				_Start_x.d[i] = buff[1+i];
				data->PC_Path.Start_x =  _Start_x.data ;
				
				if( decision.robot_data.robot_color == blue )
				{
					for (int i = 0; i < 49; i++)
						data->PC_Path.delta_x[i] = - buff[3+i];  // 1 - 49
				}					
				else
				{
					for (int i = 0; i < 49; i++)
						data->PC_Path.delta_x[i] = buff[3+i];  // 1 - 49
				}
				data->seq_path_x = buff[Len-2];

			}
			else if (buff[0] == CONST_HEAD2)
			{
				for( int i = 0;i < 2;i++ )
				_Start_y.d[i] = buff[1+i];
				data->PC_Path.Start_y =  _Start_y.data ;
				
				if( decision.robot_data.robot_color == blue )
				{
					for (int i = 0; i < 49; i++)
						data->PC_Path.delta_y[i] = - buff[3+i];  // 1 - 49
				}
				else
				{
					for (int i = 0; i < 49; i++)
						data->PC_Path.delta_y[i] = buff[3+i];  // 1 - 49					
				}
			  	data->seq_path_y = buff[Len-2];
			}
		}
		
	ptr->Navi.pos_x_set  = data->PC_Move.vx_obj ;
	ptr->Navi.pos_y_set  = data->PC_Move.vy_obj ;
	ptr->Navi.yaw_set    = data->PC_Move.wz_obj ;
		
//		if( data->seq == Last_naiv_seq || !ptr->Navi.IF_nav )
//		{
//			ptr->Navi.pos_x_set  = 0 ;
//			ptr->Navi.pos_y_set  = 0 ;
//			ptr->Navi.yaw_set    = 0 ;
//		}
		
		if( data->PC_Move.vx_obj == 0 && data->PC_Move.vy_obj == 0 && data->PC_Move.wz_obj == 0 )
			USART_TX_data.IF_STOP = YES;
		else 
			USART_TX_data.IF_STOP = NO;

		if( data->PC_Move.vx_obj == 0 && data->PC_Move.vy_obj == 0 && data->PC_Move.wz_obj == 0 )	
			ptr->IF_MOVE = 0;
		else
			ptr->IF_MOVE = 1;
		
		if( decision.robot_data.robot_color == blue )
		{
			ptr->Navi_coord.map_x   = 28 - data->Navi_coord.map_x;
		  ptr->Navi_coord.map_y   = 15 - data->Navi_coord.map_y;
			ptr->Navi_coord.map_yaw = data->Navi_coord.map_yaw;
			
			ptr->Map_Start_x = 28 - data->PC_Path.Start_x ;
			ptr->Map_Start_y = 15 - data->PC_Path.Start_y ;
		}
		else
		{
			ptr->Navi_coord.map_x   = data->Navi_coord.map_x;
		  ptr->Navi_coord.map_y   = data->Navi_coord.map_y;
			ptr->Navi_coord.map_yaw = data->Navi_coord.map_yaw;
			
			ptr->Map_Start_x = data->PC_Path.Start_x ;
			ptr->Map_Start_y = data->PC_Path.Start_y ;
		}
			
			
		Last_naiv_seq = data->seq;
}

int8_t Last_point = -2;       // TODO
int8_t Last_navi_point = -2;  // TODO


/**
  * @Name    Serial_Sand_Data
* @brief     数据发送
  * @param   None
  * @Data    2023-12-26
*/
uint8_t Serial_Seq; 
void Serial_Send_Data(Auto_t*auto_data,Chassis_Move_t*Move,Tx_Data_t *data)
{
	
	Serial_Data_Handle(auto_data,Move,data);
	
  uint8_t buff[SEND_DATA_COUNT];
	
	buff[0] = CONST_HEAD0;
	
	memcpy( buff+1,  &data->Navi.IF_nav, 1 );
	memcpy( buff+2,  &data->Navi.pos_x_set, 4 );
	memcpy( buff+6,  &data->Navi.pos_y_set, 4 );
	memcpy( buff+10, &data->Navi.yaw_set, 4 );
	memcpy( buff+14, &data->odom.IF_location, 1 );
	memcpy( buff+15, &data->odom.pos_x, 4 );
	memcpy( buff+19, &data->odom.pos_y, 4 );
	memcpy( buff+23, &data->odom.yaw, 4 );
	buff[SEND_DATA_COUNT-2] = Serial_Seq;
	buff[SEND_DATA_COUNT-1] = CONST_END0;
	
	CDC_Transmit_FS(buff, SEND_DATA_COUNT);
	
	Serial_Seq++;
}

/**
  * @Name    Serial_Data_Handle
  * @brief   串口数据处理
  * @param   Chassis_Move_t*Move: [输入/出] 
  * @Data    2023-12-26
*/
float X_test,Y_test;
static void Serial_Data_Handle(Auto_t*auto_data,Chassis_Move_t*Move,Tx_Data_t*data)
{
    if (data == NULL)
    {
        return;
    }
		
			data->odom.IF_location = if_update ;//auto_data->Odome.IF_location ;
			data->odom.pos_x = X_test ; 
			data->odom.pos_y = Y_test ;
			data->odom.yaw = robot_pos.angle  ;
		
		  
		
}

void my_first_order_filter_init(my_first_order_filter_type_t *first_order_filter_type, float frame_period, const float num)
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num = num;
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

void my_first_order_filter_cali(my_first_order_filter_type_t *first_order_filter_type, float input)
{
    first_order_filter_type->last_out = first_order_filter_type->out;
    first_order_filter_type->input = input;
    first_order_filter_type->out =
    first_order_filter_type->num / (first_order_filter_type->num + first_order_filter_type->frame_period) * first_order_filter_type->out +
    first_order_filter_type->frame_period / (first_order_filter_type->num + first_order_filter_type->frame_period) * first_order_filter_type->input;
    //	first_order_filter_type->input * first_order_filter_type->num[0]+(1-first_order_filter_type->num[0])*first_order_filter_type->last_out;
}





