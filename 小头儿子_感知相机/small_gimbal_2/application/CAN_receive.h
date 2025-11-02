#ifndef _CAN_RECEIVE_H
#define _CAN_RECEIVE_H

#include "main.h"

//#include "system.h"
#include "struct_typedef.h"

/***电机id***/
typedef enum
{
/**
*@note can1���
*/

	yaw_motor_id = 0x205,
  pitch_motor_id = 0x00,
  trig_motor_id = 0x204,
  
	//BARREL_MOTOR_ID = 0X204, 
/**
*@note can2���
*/
   FRICR_R_ID = 0x203,
   FRICR_L_ID = 0x202,
   

	/************************************云台yaw轴id*******************************/
	
	//PITCH_MOTOR_ID = 0x206,


    /***********************************超级电�?�id*******************************/

} CAN_MotorID_e;


enum
{
	YAW = 0,
	PITCH,
};

enum
{
  RIGHT = 0,
  LEFT = 1,
};

enum
{
	FR = 0,
  BR,
	BL,
	FL,
};

typedef struct 
{
	    /* data */
    int32_t round_cnt;
    int32_t total_ecd;
    int32_t total_angle;
    uint32_t msg_cnt;

    int16_t ecd;              
    int16_t last_ecd;
    int16_t ECD_angle;	
    int16_t speed_rpm;        
    int16_t feedback_current; 
    uint8_t temperate;    

    uint16_t offset_ecd; 
}motor_measure_t;

typedef struct
{
	/* data */
    fp32 sp_tar;
    fp32 corder_tar;
    fp32 ins_tar;
    int16_t set_current;
	fp32 dm_current;
}motor_tar_t;

typedef struct
{
 motor_measure_t motor_measure;
 motor_tar_t motor_tar;
}motor_data_t;

typedef struct
{
	float C_Vol;	//电�?�电�?
	float C_Power;	//电�?�输出功�?
	float Mode_C;	//电�?�状态码
	float C_Current;	//电�?�输出电�?
	float PowerData[4];
} SuperCap_t;

/***************************************4310电机**************************************/
//Master_ID 接收�?
//#define YAW_MOTOR_ID		0x010	
#define PITCH_MOTOR_ID		0x000
//Slave_ID 发送用
//#define YAW_MOTOR_slaveID	0x000
#define PITCH_MOTOR_slaveID	0x001


//根据电调�?的参数进行�?�置
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

//关节电机数据结构�?
typedef struct
{
int8_t Err;
	//�?换出的实际�?
	float Position;		//位置_弧度
	float Angle;		//位置_角度
	float Velocity;		//速度
	float Torque;		//�?�?
	//电调传回的�?
	int Position_int;
	int Velocity_int;
	int Torque_int;
	uint8_t T_MOS;		//驱动MOS温度
	uint8_t T_Rotor;	//电机内部线圈的平均温�?
	//控制
	float Torque_SET;
} DM_motor_measure_t;

#define SEND_ID_HEAD1            0x301
#define SHOOT_SEND_ID_HEAD1      0x303
#define SEND_ID_HEAD_REMOTE1     0x305

#define SEND_ID_HEAD2            0x302
#define SHOOT_SEND_ID_HEAD2      0x304
#define SEND_ID_HEAD_REMOTE2     0x306

typedef enum
{
  small_gimbal_off=0,
  small_gimbal_rc,
  small_gimbal_pc,
  
}gimbal_mode_t;

 typedef enum
{
  NO_CONTACT=0,
  red ,
  blue,
}robot_color_t ;
typedef enum
{
shoot_no=0,
shoot_on,

}shoot_mode_t;


typedef __packed struct
{
 uint8_t gimbal_mode :2;
 uint8_t robot_color :2;
 uint8_t pitch_can  :1;
 uint8_t shoot_mode :1;
 uint8_t vision_mode :1;
 uint8_t trig_mode :1;
}small_control_pack_t;


typedef __packed struct 
{  
	uint8_t shoot_all      : 1;  
	uint8_t shoot_Hero     : 1;  
	uint8_t shoot_Engineer : 1;  
	uint8_t shoot_infantr3  : 1;  
	uint8_t shoot_infantr4 : 1;  
	uint8_t shoot_Sentry   : 1;  
	uint8_t shoot_outpost  : 1; 
	uint8_t shoot_base     : 1;  

}Vision_ByteBits_t;

typedef __packed struct
{
  uint8_t outpost_state :1;//ǰ��վ״̬���Ƿ�ͣת
  uint8_t wheel_state : 3; //����״̬
  uint8_t if_arrvied :4;//����Ŀ���е�������ȼ�����
  
}other_data_t;

typedef enum
{
  trig_off=0,
  trig_on=1,
}trig_limit_t;
typedef enum
{
 curise_stop=0,
 curise_add,
 curise_minus,
 
}curise_mode_t;
typedef struct
{
 uint16_t shoot_num;
 float shoot_speed;
 uint16_t shoot_power;
shoot_mode_t shoot_mode;
small_control_pack_t small_control_pack;
Vision_ByteBits_t Vision_ByteBits;
robot_color_t robot_color;
 int16_t gimbal_yaw_in;
 int16_t gimbal_pitch_in;
 int16_t data_wheel;
 uint8_t  if_gimbal_can;
 gimbal_mode_t gimbal_mode;
  trig_limit_t trig_limit;
  curise_mode_t curise_mode;
  uint8_t if_arrvied;
  other_data_t other_data;
  uint8_t top_senior_priority;
}control_data_t;


typedef union
{
    float data;
    uint8_t d[4];
} Algorithm_float_u;

typedef union
{
    int16_t  data;
    uint8_t d[2];
} Algorithm_16_u;

typedef union
{
    uint16_t  data;
    uint8_t d[2];
} Algorithm_8_u;

typedef union
{
  
  int16_t data;
  uint8_t d[2];
}int16_to_8;

/**
*@note ��һ��ͷ����can2��
*@note �ڶ���ͷ����can1��
*/
#define SEND_ID_HEAD1            0x301
#define SHOOT_SEND_ID_HEAD1      0x303
#define SEND_ID_HEAD_REMOTE1     0x305

#define SEND_ID_HEAD2            0x302
#define SHOOT_SEND_ID_HEAD2      0x304
#define SEND_ID_HEAD_REMOTE2     0x306


#define RECEIVE_ID_2     0x402
#define RECEIVE_ID_1     0x401
void get_dm4310_motor_measure(DM_motor_measure_t *ptr, uint8_t rx_data[]);
const SuperCap_t *get_supercup_pointer(void);
 
void get_gimbal_data(uint8_t rx_data[8],control_data_t *data);
void get_shoot_data(uint8_t rx_data[8],control_data_t *data);
void com_data_reset(control_data_t *mode);
//extern DM_motor_measure_t yaw_motor;


extern motor_data_t yaw_motor;
extern DM_motor_measure_t pitch_motor;
extern motor_data_t trig_motor;
extern motor_data_t trig_motor;
extern control_data_t control_data;
extern DM_motor_measure_t pitch_motor;
extern motor_data_t shoot_motor[2];

#endif
