#ifndef __BSP_PID_H
#define __BSP_PID_H

typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

#ifndef NULL
#define NULL 0
#endif

typedef struct _pid_struct_t
{
  float kp;
  float ki;
  float kd;
  float i_max;
  float out_max;
  
  float ref;      // 目标值
  float fdb;      // 实际值
  float err[2];   // 误差和上次误差

	float set;
  float get;
	
  float p_out;
  float i_out;
  float d_out;
  float output;
}pid_struct_t;

enum
{
  LAST  = 0,
  NOW   = 1,
};


void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max);
float pid_calc(pid_struct_t *pid, float get, float set);
			

							
							
							
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次

} pid_type_def;


	typedef struct 
{
    /* data */
    float disturb;       //????
    float last_disturb;  //????
    float pre_disturb;   //????
    float alpha;         //a
    float belta;         //b
    float output;
    float outmax;
}feedforward_control_t;


/** 
  * @brief        ?????
  * @param        {feedforward_control_t} *str  ???????
  * @param        {fp32} alpha  
  * @param        {fp32} belta
  * @param        {fp32} outmax
  * @return       {*}
  */
void feedforward_control_init(feedforward_control_t *str, float alpha, float belta, float outmax);


/** 
  * @brief        ????
  * @param        {feedforward_control_t} *str ???????
  * @param        {fp32} disturb  ??
  * @return       {fp32} str->output  ??
  */
float feedforward_control_calc(feedforward_control_t *str, float disturb);		
extern void bsp_PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);
extern fp32 bsp_PID_calc(pid_type_def *pid, fp32 ref, fp32 set);
extern void bsp_PID_clear(pid_type_def *pid);



#endif

