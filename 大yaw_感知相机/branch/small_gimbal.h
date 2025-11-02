#ifndef __SMALL_GIMBAL_H
#define __SMALL_GIMBAL_H
#include "referee.h"
#include "system.h"
typedef struct
{
  int16_t gimbal_yaw_in;
  int16_t gimbal_pitch_in;
  int pitch_can;
}small_gimbal_t;

/**small_gimbal_send_t
*@note弹速一般为float类型变量，但是考虑到只有一位小数，在这里将数据*10转换成uint16_t的数据
为了节省资源，接收解析除以10即可
*@note将云台的增量也以原数据发过去，在解析中*0.0001即可

*/
typedef __packed struct
{
 uint8_t gimbal_mode :2;
 uint8_t robot_color :2;
 uint8_t pitch_can  :1;
 uint8_t shoot_mode :1;
 uint8_t vision_mode :1;
 uint8_t trig_mode  :1;
}small_control_data_t;
//小云台控制杂项

typedef __packed struct
{
  uint8_t outpost_state :1;//前哨站状态，是否停转
  uint8_t wheel_state : 3; //波轮状态
  uint8_t if_arrived :4;//是否到达目标位置
  
}other_data_t;


enum ARMOR_ID
{
    ARMOR_HERO = 1,
    ARMOR_ENGINEER = 2,
    ARMOR_INFANTRY3 = 3,
    ARMOR_INFANTRY4 = 4,
    ARMOR_INFANTRY5 = 5,
    ARMOR_SENTRY = 6,
    ARMOR_OUTPOST= 7,
    ARMOR_BASE= 8,
     
};

typedef enum
{
 curise_stop=0,
 curise_add,
 curise_minus,
 
}curise_mode_t;
typedef struct
{
 int gimbal_mode;
  int16_t shoot_speed_1;
  
  int16_t shoot_speed_2;
 uint16_t shooter_17mm_1_barrel_heat;
    uint16_t shooter_17mm_2_barrel_heat;
    robot_color_t robot_color;
    uint16_t shoot_num_1;
    uint16_t shoot_num_2;
    uint8_t if_pitch_can;
    curise_mode_t curise_mode;
    small_control_data_t small_control_data;
    other_data_t other_data;
}small_gimbal_send_t;


void get_gimbal_com();
void system_mode_to_small(small_gimbal_t *mode);

void send_gimbal_mode_1_gimbal(CAN_HandleTypeDef *hcan,uint32_t id_range,small_gimbal_send_t*data);
void send_gimbal_mode_2_gimbal(CAN_HandleTypeDef *hcan,uint32_t id_range,small_gimbal_send_t*data);
void send_gimbal_mode_1_shoot(CAN_HandleTypeDef *hcan,uint32_t id_range,small_gimbal_send_t*data);
void send_gimbal_mode_2_shoot(CAN_HandleTypeDef *hcan,uint32_t id_range,small_gimbal_send_t*data);
extern small_gimbal_t small_gimbal;
#endif