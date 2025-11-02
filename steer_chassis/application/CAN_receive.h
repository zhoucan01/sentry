#ifndef _CAN_RECEIVE_H
#define _CAN_RECEIVE_H

#include "main.h"
#include "struct_typedef.h"

/***电机id***/
typedef enum
{
	/*����3508�н����*/
	FR_3508_ID = 0X204,
	BR_3508_ID = 0X203,
	BL_3508_ID = 0X202,
	FL_3508_ID = 0X201,
	
	
	
	/*���̶�����*/
	
	
	FR_6020_ID = 0x208,
	BR_6020_ID = 0x207,
	BL_6020_ID = 0x206,
	FL_6020_ID = 0x205,
	//BARREL_MOTOR_ID = 0X204, 

  pm_id = 0x212,
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
	FR = 3,
  BR = 2,
	BL = 1,
	FL = 0,
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
  float Real_Power;
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


void get_dm4310_motor_measure(DM_motor_measure_t *ptr, uint8_t rx_data[]);
const SuperCap_t *get_supercup_pointer(void);
extern motor_data_t chassis_motor[4];
extern motor_data_t yaw_motor;
extern motor_data_t shoot_motor[2];
extern motor_data_t trig_motor;
extern SuperCap_t SuperCAP;
extern DM_motor_measure_t pitch_motor;
extern float pm_power;
extern motor_data_t steer_motor[4];
#endif
