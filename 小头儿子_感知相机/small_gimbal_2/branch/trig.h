/**
 ******************************************************************************
 * @file    trig.h
 * @brief   拨盘触发控制 - 单发/连发/视觉射击 + 卡弹检测与反卡
 ******************************************************************************
 */

#ifndef __TRIG_H
#define __TRIG_H

#include "string.h"
#include <stdlib.h>
#include "math.h"
#include "stdbool.h"
#include "bsp_pid.h"

/* ---- 拨盘机械参数 ---- */
#define TRIG_SIN_ECD        32700
#define TRIG_1RPS_SPEED     2160
#define TRIG_SLOT_NUM       8
#define TRIG_1FPS_SPEED     (TRIG_1RPS_SPEED / TRIG_SLOT_NUM)

/* ---- 射频档位 ---- */
#define CON_FREQ_22  22
#define CON_FREQ_20  20
#define CON_FREQ_15  15
#define CON_FREQ_12  12
#define CON_FREQ_10  10
#define CON_FREQ_5    5
#define CON_FREQ_1    1

/* ---- 功率限制 ---- */
#define TRIG_MAX_POWER          400.0f
#define TRIG_OUTPOST_POWER_LIMIT 250.0f
#define TRIG_NORMOL_POWER_LIMIT  80.0f

/* ---- 卡弹检�?---- */
#define BLOCK_CURRENT_THRESH    7000
#define BLOCK_SPEED_THRESH      50
#define BLOCK_CNT_THRESH        70
#define BLOCK_REACT_TIME        50

/* ---- 类型 ---- */
typedef enum { FIRE_NO = 0, FIRE_SIN, FIRE_CON, FIRE_VISION } fire_state_t;
typedef enum { NO_BLOCK = 0, SIN_BLOCK, CON_BLOCK } block_type_t;

typedef struct {
    block_type_t block_type;
    bool if_fire_block, if_block_over;
    int  cnt;
} block_state_t;

typedef struct {
    bool if_sin_over, if_con_over;
    bool if_sin_request, if_con_request;
    bool if_block_react, if_block_react_over;
} trig_flag_t;

typedef struct {
    fire_state_t  fire_state, last_state;
    bool          if_block;
    block_state_t block_state;
    trig_flag_t   trig_flag;
    int           trig_freq;
} trig_t;

/* ---- 全局变量 ---- */
extern trig_t        trig;
extern pid_struct_t  pid_trig_sin_ecd;
extern pid_struct_t  pid_trig_sin_speed;
extern pid_struct_t  pid_trig_con;
extern int           trig_ecd_set;
extern float         trig_speed_set;

/* ---- 函数声明 ---- */
void trig_run(void const *argument);
void trig_task_run(trig_t *mode);
void trig_mode_chose(trig_t *mode);
void tirg_control(trig_t *mode);
void trig_motor_run(trig_t *mode);
void get_trig_com(void);
void trig_chose_freq(trig_t *mode);
void trig_sin(trig_t *mode);
void trig_con(trig_t *mode);
void trig_vision(trig_t *mode);
void shoot_rc_mode(trig_t *mode);
void shoot_pc_mode(trig_t *mode);
void get_trig_motor_state(trig_t *mode);
void judge_if_block(void);
void block_react(trig_t *mode);
void sin_block_react(trig_t *mode);
void con_block_react(trig_t *mode);
void reset_block_flag(trig_t *mode);
bool judge_if_sin_over(void);
void trig_pid_init(void);

#endif
