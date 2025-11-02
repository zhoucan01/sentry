/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       该任务是通过串口中断启动，不是freeRTOS任务
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-01-2019     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "remote_control.h"
#include "main.h"
//#include "detect.h"
#include "bsp_rc.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"

#define RC_CHANNAL_ERROR_VALUE                           700

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

uint8_t remote_source = REMOTE_MODE;

/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

//remote control data 
//遥控器控制变量
RC_ctrl_t rc_ctrl;
static int16_t RC_abs(int16_t value);
//receive data, 18 bytes one frame, but set 36 bytes 
//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
void remote_control_init(void)
{
    RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
//		rc_ctrl.keyboard.flag_G=1;
}
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

/**
  * @brief          遥控器数据错误处理,将遥控器所有数据清零
  * @param[in]      none
  * @retval         none
  */
bool_t RC_data_is_error_handle(void)
{
    if (RC_abs(rc_ctrl.rc.ch0) > RC_CHANNAL_ERROR_VALUE)
    {
        goto rc_error_handle;
    }
    if (RC_abs(rc_ctrl.rc.ch1) > RC_CHANNAL_ERROR_VALUE)
    {
        goto rc_error_handle;
    }
    if (RC_abs(rc_ctrl.rc.ch2) > RC_CHANNAL_ERROR_VALUE)
    {
        goto rc_error_handle;
    }
    if (RC_abs(rc_ctrl.rc.ch3) > RC_CHANNAL_ERROR_VALUE)
    {
        goto rc_error_handle;
    }
    if (0 == rc_ctrl.rc.s_l)
    {
        goto rc_error_handle;
    }
    if (0 == rc_ctrl.rc.s_r)
    {
        goto rc_error_handle;
    }
    return 0;

rc_error_handle:
    memset(&rc_ctrl, 0, sizeof(RC_ctrl_t));
    return 1;
}



/**
  * @brief          遥控器掉线处理,将遥控器所有数据清零
  * @param[in]      none
  * @retval         none
  */
bool_t RC_offline_data_handle(void)
{
	memset(&rc_ctrl, 0, sizeof(RC_ctrl_t));
	return 0;
}



/**
  * @brief          遥控器掉线处理,软件重连
  * @param[in]      none
  * @retval         none
  */
void RC_connect_soft_restart(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}

//uint32_t REMOTE_Time = 0;
uint8_t res = 0;
uint32_t REMOTE_Time = 0;
uint32_t last_Time = 0;	
uint32_t time_1=0;
uint32_t diff_t=0;
int get_if_communite_broke(void)	
{
	
	time_1=HAL_GetTick();
	diff_t= time_1- REMOTE_Time;
	if((diff_t> 1500))
	{res = 1;}
	else
	{res = 0;}
	return res;
}





//串口中断
int ggggg;
void USART3_IRQHandler(void)
{
	REMOTE_Time = HAL_GetTick();//记录接收遥控器数据的时间
	ggggg++;
	
    if(huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */
    
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc      (sbus_rx_buf[0], &rc_ctrl);
							  #ifdef USE_DETECT
									communication_frame_rate_update(DBUS_TOE);
								#endif
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
							
							  #ifdef USE_DETECT
									communication_frame_rate_update(DBUS_TOE);
								#endif
            }
        }
    }
}

static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}


/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch0 = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch1 = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch2 = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch3 = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s_r = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s_l = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->rc.wheel = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL
		
    rc_ctrl->mouse.vx = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.vy = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.vz = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
		
    rc_ctrl->keyboard.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
		rc_ctrl->keyboard.key_W  = (sbus_buf[14] >> 0) & 0x01;
		rc_ctrl->keyboard.key_S     = (sbus_buf[14] >> 1) & 0x01;
		rc_ctrl->keyboard.key_A     = (sbus_buf[14] >> 2) & 0x01;
		rc_ctrl->keyboard.key_D     = (sbus_buf[14] >> 3) & 0x01;
		rc_ctrl->keyboard.key_Shift = (sbus_buf[14] >> 4) & 0x01;
		rc_ctrl->keyboard.key_CTRL  = (sbus_buf[14] >> 5) & 0x01;
		rc_ctrl->keyboard.key_Q     = (sbus_buf[14] >> 6) & 0x01;
		rc_ctrl->keyboard.key_E     = (sbus_buf[14] >> 7) & 0x01;
		
		rc_ctrl->keyboard.key_R    = (sbus_buf[15] >> 0) & 0x01;
		rc_ctrl->keyboard.key_F    = (sbus_buf[15] >> 1) & 0x01;
		rc_ctrl->keyboard.key_G     = (sbus_buf[15] >> 2) & 0x01;
		rc_ctrl->keyboard.key_Z     = (sbus_buf[15] >> 3) & 0x01;
		rc_ctrl->keyboard.key_X     = (sbus_buf[15] >> 4) & 0x01;
		rc_ctrl->keyboard.key_C    = (sbus_buf[15] >> 5) & 0x01;
		rc_ctrl->keyboard.key_V     = (sbus_buf[15] >> 6) & 0x01;
		rc_ctrl->keyboard.key_B     = (sbus_buf[15] >> 7) & 0x01;

	  rc_ctrl->rc.ch0 -=RC_CH_VALUE_OFFSET;
		rc_ctrl->rc.ch1 -=RC_CH_VALUE_OFFSET;
		rc_ctrl->rc.ch2 -=RC_CH_VALUE_OFFSET;
		rc_ctrl->rc.ch3 -=RC_CH_VALUE_OFFSET;
		rc_ctrl->rc.wheel -=RC_CH_VALUE_OFFSET;		
		
		if((rc_ctrl->keyboard.key_Z == 1) && (rc_ctrl->keyboard.last_Z == 0))                            //flag Z 切换模式的按键
		rc_ctrl->keyboard.flag_Z =! rc_ctrl->keyboard.flag_Z;
		rc_ctrl->keyboard.last_Z =rc_ctrl->keyboard.key_Z;
		
		if((rc_ctrl->keyboard.key_X == 1) && (rc_ctrl->keyboard.last_X == 0))                            //flag X 切换模式的按键
		rc_ctrl->keyboard.flag_X =! rc_ctrl->keyboard.flag_X;
		rc_ctrl->keyboard.last_X =rc_ctrl->keyboard.key_X;
		
		if((rc_ctrl->keyboard.key_C == 1) && (rc_ctrl->keyboard.last_C == 0))                            //flag C 切换模式的按键
		rc_ctrl->keyboard.flag_C =! rc_ctrl->keyboard.flag_C;
		rc_ctrl->keyboard.last_C =rc_ctrl->keyboard.key_C;
		
		if((rc_ctrl->keyboard.key_V == 1) && (rc_ctrl->keyboard.last_V == 0))                            //flag V 切换模式的按键
		rc_ctrl->keyboard.flag_V =! rc_ctrl->keyboard.flag_V;
		rc_ctrl->keyboard.last_V =rc_ctrl->keyboard.key_V;
		
		if((rc_ctrl->keyboard.key_CTRL == 1) && (rc_ctrl->keyboard.last_CTRL == 0))                      //flag Ctrl 切换模式的按键
		rc_ctrl->keyboard.flag_CTRL =! rc_ctrl->keyboard.flag_CTRL;
		rc_ctrl->keyboard.last_CTRL =rc_ctrl->keyboard.key_CTRL;
		
		if((rc_ctrl->keyboard.key_Q == 1) && (rc_ctrl->keyboard.last_Q == 0))                            //flag Q 切换模式的按键
		rc_ctrl->keyboard.flag_Q =! rc_ctrl->keyboard.flag_Q;
		rc_ctrl->keyboard.last_Q =rc_ctrl->keyboard.key_Q;
		
		if((rc_ctrl->keyboard.key_E == 1) && (rc_ctrl->keyboard.last_E == 0))                            //flag E 切换模式的按键
		rc_ctrl->keyboard.flag_E =! rc_ctrl->keyboard.flag_E;
		rc_ctrl->keyboard.last_E =rc_ctrl->keyboard.key_E;
		
		if((rc_ctrl->keyboard.key_F == 1) && (rc_ctrl->keyboard.last_F == 0))                            //flag F 切换模式的按键
		rc_ctrl->keyboard.flag_F =! rc_ctrl->keyboard.flag_F;
		rc_ctrl->keyboard.last_F =rc_ctrl->keyboard.key_F;
		
		if((rc_ctrl->keyboard.key_G == 1) && (rc_ctrl->keyboard.last_G == 0))                            //flag G 切换模式的按键
		rc_ctrl->keyboard.flag_G =! rc_ctrl->keyboard.flag_G;
		rc_ctrl->keyboard.last_G =rc_ctrl->keyboard.key_G;
		
		if((rc_ctrl->keyboard.key_B == 1) && (rc_ctrl->keyboard.last_B == 0))                            //flag G 切换模式的按键
		rc_ctrl->keyboard.flag_B =! rc_ctrl->keyboard.flag_B;
		rc_ctrl->keyboard.last_B =rc_ctrl->keyboard.key_B;
		
    if((rc_ctrl->keyboard.key_R == 1) && (rc_ctrl->keyboard.last_R == 0))                            //flag G 切换模式的按键
		rc_ctrl->keyboard.flag_R =! rc_ctrl->keyboard.flag_R;
		rc_ctrl->keyboard.last_R =rc_ctrl->keyboard.key_R;
		
		if((rc_ctrl->mouse.press_r == 1) && (rc_ctrl->mouse.last_press_r  == 0))                            //flag G 切换模式的按键
		rc_ctrl->mouse.flag_press_r =! rc_ctrl->mouse.flag_press_r;
		rc_ctrl->mouse.last_press_r=rc_ctrl->mouse.press_r;
    
		WHEEL_STATE_Ctrl();//拨轮状态辨识
		KEY_STATE_Ctrl();//鼠标左键状态辨识
}

/**
 * @brief 拨轮状态辨识
 * @param 
 */
int WHEEL_UP_TIME;
int WHEEL_DOWN_TIME;
int WHEEL_LAST;
Wheel_State_t Wheel_State;
//int receive_data=rc_ctrl.rc.wheel;
int receive_data=0;
void WHEEL_STATE_Ctrl(void)
{
 
receive_data=rc_ctrl.rc.wheel;
	if ( receive_data<-600 )
	{
		WHEEL_UP_TIME ++;
	}
	else if ( receive_data>600)
	{
		WHEEL_DOWN_TIME ++;
	}
	
	//判断为非动作位
	if ( receive_data>=-600 && receive_data<=600 && WHEEL_LAST>=-600 && WHEEL_LAST<=600 )
	{
		Wheel_State = ZERO_rc;
		WHEEL_DOWN_TIME = 0;
		WHEEL_UP_TIME = 0;
	}
	
	//UP--松开时进行判断SHORT or LONG
	if ( receive_data>=-600 && receive_data<=600 && WHEEL_LAST<-600 && WHEEL_UP_TIME<10 )
		Wheel_State = UP_SHORT_rc;
	else if(receive_data<-600 && WHEEL_UP_TIME>=20 )
		Wheel_State = UP_LONG_rc;
	
	//DOWN--松开时进行判断SHORT or LONG
	if ( receive_data>=-600 && receive_data<=600 && WHEEL_LAST>600 && WHEEL_DOWN_TIME<30 )
  {
  Wheel_State = DOWN_SHORT_rc;

  }
		
	else if( receive_data>600 && WHEEL_DOWN_TIME>=30 )
		Wheel_State = DOWN_LONG_rc;
	
	WHEEL_LAST = receive_data;
}

/**
 * @brief 鼠标左键状态辨识
 * @param 
 */
int KEY_LEFT_TIME;
int KEY_LAST;
void KEY_STATE_Ctrl(void)
{
	if ( rc_ctrl.mouse.press_l == 1 )
	{
		KEY_LEFT_TIME ++;
	}
	else
	{
		KEY_LEFT_TIME = 0;
		rc_ctrl.mouse .KEY_L_State = NOP_rc;
	}
	
	//松开时进行判断SHORT or LONG
	if ( rc_ctrl.mouse.press_l==1 && KEY_LEFT_TIME<12 )
		rc_ctrl.mouse .KEY_L_State  = PUSH_SHORT_rc;
	else if( rc_ctrl.mouse.press_l==1 && KEY_LEFT_TIME>=12 )
		rc_ctrl.mouse .KEY_L_State = PUSH_LONG_rc;
	
	KEY_LAST = rc_ctrl.mouse.press_l;
}

/*遥控器参数初始化*/
void Remote_Data_Init(void)
{
	rc_ctrl.rc.ch0 = 0;
	rc_ctrl.rc.ch1 = 0;
	rc_ctrl.rc.ch2 = 0;
	rc_ctrl.rc.ch3 = 0;
	rc_ctrl.rc.s_l = 2;
	rc_ctrl.rc.s_r = 2;
	rc_ctrl.rc.wheel = 0;
	
	rc_ctrl.keyboard.flag_C = 0;
	rc_ctrl.keyboard.flag_CTRL = 0;
	rc_ctrl.keyboard.flag_V = 0;
	rc_ctrl.keyboard.flag_X = 0;
	rc_ctrl.keyboard.flag_Z = 0;
	rc_ctrl.keyboard.key_A = 0;
	rc_ctrl.keyboard.key_B = 0;
	rc_ctrl.keyboard.key_C = 0;
	rc_ctrl.keyboard.key_CTRL = 0;
	rc_ctrl.keyboard.key_D = 0;
	rc_ctrl.keyboard.key_E = 0;
	rc_ctrl.keyboard.key_F = 0;
	rc_ctrl.keyboard.key_G = 0;
	rc_ctrl.keyboard.key_Q = 0;
	rc_ctrl.keyboard.key_R = 0;
	rc_ctrl.keyboard.key_S = 0;
	rc_ctrl.keyboard.key_Shift = 0;
	rc_ctrl.keyboard.key_V = 0;
	rc_ctrl.keyboard.key_W = 0;
	rc_ctrl.keyboard.key_X = 0;
	rc_ctrl.keyboard.key_Z = 0;
	rc_ctrl.keyboard.last_C = 0;
	rc_ctrl.keyboard.last_CTRL = 0;
	rc_ctrl.keyboard.last_V = 0;
	rc_ctrl.keyboard.last_X = 0;
	rc_ctrl.keyboard.last_Z = 0;
	rc_ctrl.keyboard.v = 0;
	rc_ctrl.keyboard.vx = 0;
	rc_ctrl.keyboard.vy = 0;
//	rc_ctrl.rc.WHEEL_State = ZERO_rc;
	rc_ctrl.mouse .KEY_L_State = NOP_rc;
}



