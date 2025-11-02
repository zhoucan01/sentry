/**
  ************************************* Copyright ****************************** 
  * FileName   : Sentry_cmd.c   
  * Version    : v1.0		
  * Author     : ç‹å­ä½©
  * Number     : 18602780430 	
  * Date       : 2024-05-12         
  * Description:    
  * Function List:  
  	1. ....
  	   <version>: 		
  <modify staff>:
  		  <data>:
   <description>:  
  	2. ...
  ******************************************************************************
 */
#include "Sentry_cmd.h"
#include "referee.h"
#include "navigation.h"
#include "cmsis_os.h"
Sentry_cmd_t Sentry_cmd;
Sentry_cmd_Fill_t Sentry_cmd_Fill;
uint8_t IF_revive = 1;


Map_data_t Map_data;
/**
 * @description: ÉÚ±ø»úÆ÷ÈËÏò¼º·½¿ÕÖĞ»úÆ÷ÈËÑ¡ÊÖ¶Ë·¢ËÍÂ·¾¶×ø±êÊı¾İ£¬¸ÃÂ·¾¶»áÔÚĞ¡µØÍ¼ÉÏÏÔÊ¾
 * @param {uint8_t} intention
 * @param {uint16_t} start_position_x
 * @param {uint16_t} start_position_y
 * @param {int8_t} delta_x
 * @param {int8_t} delta_y
 * @return {*}
 */
int8_t Map_diff_x,Map_diff_y;
float Map_diff_x_f,Map_diff_y_f;
uint16_t final_x,final_y;
void Path_display()
{

		
		static uint8_t data_pack[Map_Data_byte];
		uint8_t Map_Seq;
    

   
    /* æ•°æ®å¡«å…… */
    Map_data.path_data.data.intention = 1;
//    Map_data.path_data.data.start_position_x =  navigation_rx.current_x;
//    Map_data.path_data.data.start_position_y =  navigation_rx.current_y ;
    
    
//    if(decision.robot_data.robot_color==red)
//    {
//      Map_data.path_data.data.start_position_x =  Red_Navi_position[0][0]*10;
//      Map_data.path_data.data.start_position_y =  Red_Navi_position[0][1]*10;
//    }
//    else 
//    {
//      Map_data.path_data.data.start_position_x =  (28-Red_Navi_position[0][0])*10;
//      Map_data.path_data.data.start_position_y =  (15-Red_Navi_position[0][1])*10;
//    }
    
    
    
//    Map_diff_x_f=(Red_Navi_position[decision.point][0]*10-Map_data.path_data.data.start_position_x)/2;
//    Map_diff_y_f=(Red_Navi_position[decision.point][1]*10-Map_data.path_data.data.start_position_y)/2;


  
    if(decision.robot_data.robot_color==red)
    {
      Map_data.path_data.data.start_position_x =  (navigation_rx.current_x+Red_Navi_position[0][0])*10;
      Map_data.path_data.data.start_position_y =  (navigation_rx.current_y+Red_Navi_position[0][1])*10;
      
      final_x=Red_Navi_position[decision.point][0]*10;
      final_y=Red_Navi_position[decision.point][1]*10;
      
      
    }
    else 
    {
      Map_data.path_data.data.start_position_x =  (28-(navigation_rx.current_x+Red_Navi_position[0][0]))*10;
      Map_data.path_data.data.start_position_y =  (15-(navigation_rx.current_y+Red_Navi_position[0][1]))*10;
      
      final_x=(28-Red_Navi_position[decision.point][0])*10;
      final_y=(15-Red_Navi_position[decision.point][1])*10;
    }
    
//    if(decision.robot_data.robot_color==red)
//    { 
      if(decision.point==MANUAL_POINT)
      {
        final_x=map_command.target_position_x*10;
        final_y=map_command.target_position_y*10;
      }
//    }
//    else 
//    {
//      if(decision.point==MANUAL_POINT)
//      {
//        final_x=Red_Navi_position[decision.point][0];
//        final_y=Red_Navi_position[decision.point][1];
//      }
//    }
    
    
    
//    Map_data.path_data.data.start_position_x =  (navigation_rx.current_x+Red_Navi_position[0][0])*10;
//    Map_data.path_data.data.start_position_y =  (navigation_rx.current_y+Red_Navi_position[0][1])*10;
    
    Map_diff_x_f=(final_x-Map_data.path_data.data.start_position_x)/2;
    Map_diff_y_f=(final_y-Map_data.path_data.data.start_position_y)/2;
    
    
    Map_diff_x=(int8_t)Map_diff_x_f;
    Map_diff_y=(int8_t)Map_diff_y_f;
    
    
    for (int i = 0; i < 49; i++)
    {
        Map_data.path_data.data.delta_x[i] = 0;
        Map_data.path_data.data.delta_y[i] = 0;
    }
		Map_data.path_data.data.delta_x[47] = Map_diff_x;
    Map_data.path_data.data.delta_y[47] = Map_diff_y;
    
    Map_data.path_data.data.delta_x[48] = Map_diff_x;
    Map_data.path_data.data.delta_y[48] = Map_diff_y;
    
    
    
		Map_data.path_data.data.sender_id = robot_status.robot_id;

    Map_data.path_data.UI_Packhead.SOF = UI_SOF;
    Map_data.path_data.UI_Packhead.Seq = Map_Seq;
    Map_data.path_data.UI_Packhead.Data_Length = Map_Data_byte - 9;
		
    /* å‘é€æ•°æ® */
		
    memcpy(data_pack, &Map_data.path_data.UI_Packhead, 5);

    append_CRC8_check_sum(data_pack, 5);

    uint16_t CMD_ID = CONST__PATH_SEND_CMD_ID;
    memcpy(data_pack + 5, &CMD_ID, 2);

    memcpy(data_pack + 7, &Map_data.path_data.data , 105);

    append_CRC16_check_sum(data_pack, Map_Data_byte);

    HAL_UART_Transmit_DMA(&huart6, data_pack, Map_Data_byte);

    Map_Seq++;
}

//void Sentry_cmd_Ctl( Sentry_cmd_Fill_t * Sentry_cmd_Fill,Decision_t * decision)
//{
//  Sentry_cmd_Fill->IF_Revive                = 1;
//	Sentry_cmd_Fill->IF_Remote_Exchange_HP    = 0;
//	Sentry_cmd_Fill->IF_Immediately_Revive    = 0;
//	Sentry_cmd_Fill->Exchange_Projectile_Num  = 0;
////	if( map_command.cmd_keyboard == 'B' )
////		Sentry_cmd_Fill->Exchange_Projectile_Num = 150;
//	
////	if( game_state.game_progress == 4 )
//	
//	if( System.Robot_color == RED )
//    Robot_ID = UI_Data_RobotID_RSentry;
//	else
//		Robot_ID = UI_Data_RobotID_BSentry;
//		
////	Sentry_Cmd_Fill( &Sentry_cmd,Sentry_cmd_Fill->IF_Revive,Sentry_cmd_Fill->IF_Immediately_Revive ,Sentry_cmd_Fill->Exchange_Projectile_Num,Sentry_cmd_Fill->IF_Remote_Exchange_HP );
//	
//	if( game_state.game_progress == 4 )
//	if( IF_revive  )
//	UI_SendSentry( &Sentry_cmd );
//	
//}


uint16_t ooo;
void referee_transmit()
{

vTaskDelay(500);
  for(;;)
  {
  
    Path_display();
    vTaskDelay(300);
    ooo++;
      if(sentry_info.sentry_if_free_revive==1&decision.Cmd_condition.If_revive==1)
    {
      UI_SendSentry(&Sentry_cmd_send);
      vTaskDelay(100);
    }
    
  }
}

