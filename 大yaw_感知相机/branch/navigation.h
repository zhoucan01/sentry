#ifndef __NAVIGATION_H
#define __NAVIGATION_H

#define CONST_HEAD0             0XA5     // 帧头
#define CONST_END0              0XAA     // 帧尾

#define CONST_HEAD1             0XB0     // 帧头
#define CONST_END1              0XBB    // 帧尾

#define CONST_HEAD2             0XB1    // 帧头
#define CONST_END2              0XBB    // 帧尾

#define navigation_tx_len   18
#include "stdbool.h"
#include "struct_typedef.h"
#include "bsp_transmit.h"
//typedef union 
//{
//	float data;
//	uint8_t d[4];
//}u8_to_float;


//typedef union
//{
//  uint16_t data;
//  uint8_t d[2];
//}u8_to_u16;

//typedef union
//{
//  
//  int16_t data;
//  uint8_t d[2];
//}int16_to_8;

typedef enum
{
  navi_state_off=0,
  navi_state_on,
  
}Navi_state_t;


typedef struct
{
  float navi_vx;
  float navi_vy;
  float navi_wz;
  
  float current_x;
  float current_y;
  
  uint8_t if_arrived;
  uint8_t if_lost_navi;
  
  Navi_state_t Navi_state;
  uint8_t If_get_path;
  
//  uint8_t seq;//包序号
}navigation_rx_t;
typedef struct
{
  float yaw_diff;
  float yaw_init_ecd;
  float relative_ecd;
  
}odom_navi_t;

typedef struct 
{
  float navi_set_x_pos;
  float navi_set_y_pos;
  float current_yaw;
  float current_pitch;
  float current_roll;
  int if_navi; 
  float Odom_Vx;
  float Odom_Vy;
  float Odom_Sx;
  float Odom_Sy;
  float yaw_Wz;
  
  odom_navi_t odom_navi;
  
}navigation_tx_t;

typedef struct
{
 float Sx;
 float Sy;
 float Sx_set;
 float Sy_set;
 float steer_real_angle[4];
 float steer_init_ecd[4];
 float real_Vx,real_Vy;
 float real_Vx_c,real_Vy_c;
 float vx,vy;
 float Vx_c,Vy_c;
 float vx_all[4];
 float init_yaw;  // 检测上电那一刻的陀螺仪值，只是为了里程计计算用,后续考虑
 float diff_yaw;
 float diff_angle;
 bool yaw_update;
 float yaw_set;
}location_t;
//typedef struct
//{
//  float nav_vx;
//  float nav_vy;
//  float Wz_speed;
//  
//  
//  
//}navigation_rx_t
extern int navi_tx_count;
extern navigation_tx_t navigation_tx;
extern navigation_rx_t navigation_rx;
extern  uint8_t ninin;
extern uint8_t navi_state_get;
void navigation_rx_handle(uint8_t *buff,uint32_t Len,navigation_rx_t *data);
void Navigation_Tx_Send(navigation_tx_t *data);
#endif