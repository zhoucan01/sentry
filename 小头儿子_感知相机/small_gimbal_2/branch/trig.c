/**
 ******************************************************************************
 * @file    trig.c
 * @brief   拨盘触发控制 - 单发/连发/视觉 + 卡弹检测反卡 + 射频自适应
 *
 * 状态机: FIRE_NO ↔ FIRE_SIN ↔ FIRE_CON ↔ FIRE_VISION
 * 反卡逻辑: 检测堵转 → block_react → 卸力 → reset
 ******************************************************************************
 */

#include "trig.h"
#include "shoot.h"
#include "system.h"
#include "CAN_receive.h"
#include "cmsis_os.h"
#include "struct_typedef.h"
#include "stm32f4xx_hal.h"
#include "remote_control.h"
#include "Nautilus_Vision.h"
#include "gimbal.h"
#include "motor.h"

/* ---- 全局 ---- */
com_mode_t trig_com;
pid_struct_t pid_trig_sin_ecd;
pid_struct_t pid_trig_sin_speed;
pid_struct_t pid_trig_con;

int   trig_ecd_set;
float trig_speed_set;

trig_t trig = {
    .fire_state = FIRE_NO,
    .last_state = FIRE_NO,
    .trig_flag.if_sin_over        = 1,
    .trig_flag.if_con_over        = 1,
    .trig_flag.if_block_react     = 1,
    .trig_flag.if_block_react_over = 1,
};

/* ---- 内部状态 ---- */
static uint16_t react_time;

/* ---- 内部函数 ---- */
static void receive_wheel_state(void);

/* ======================== 初始化 + 主任务 ======================== */

void trig_pid_init(void)
{
    pid_init(&pid_trig_sin_ecd,   1.0f, 0.0f,    0.1f,  10.0f,  4000.0f);
    pid_init(&pid_trig_sin_speed, 8.0f, 0.0005f, 0.0f, 100.0f, 10000.0f);
    pid_init(&pid_trig_con,      25.0f, 0.0f,    0.0f,   0.0f, 10000.0f);
}

void trig_run(void const *argument)
{
    (void)argument;
    vTaskDelay(2);
    trig_pid_init();
    for (;;) {
        get_trig_com();
        trig_task_run(&trig);
        vTaskDelay(1);
    }
}

/* ======================== 通信检测 ======================== */

void get_trig_com(void)
{
    trig_com = (control_data.shoot_mode != shoot_no
             && gimbal_system.control_com == com_nom)
             ? com_nom : com_err;
}

/* ======================== 主状态机 ======================== */

void trig_task_run(trig_t *mode)
{
    receive_wheel_state();
    trig_mode_chose(mode);
    get_trig_motor_state(mode);
    tirg_control(mode);
    trig_motor_run(mode);
}

static void receive_wheel_state(void)
{
    Wheel_State = control_data.other_data.wheel_state;
}

/* ---- 模式选择 ---- */

void trig_mode_chose(trig_t *mode)
{
    if (trig_com != com_nom) {
        mode->fire_state = FIRE_NO;
        trig_ecd_set = trig_motor.motor_measure.total_ecd;
        trig_motor.motor_tar.set_current = 0;
        return;
    }

    if (control_data.gimbal_mode == small_gimbal_rc) {
        shoot_rc_mode(mode);
    } else {
        shoot_pc_mode(mode);
    }
}

/* ---- 遥控器射击模式 ---- */

void shoot_rc_mode(trig_t *mode)
{
    switch (Wheel_State) {
    case ZERO_rc:
        mode->fire_state = FIRE_NO;
        mode->trig_flag.if_sin_request = 1;
        break;
    case DOWN_SHORT_rc:
        mode->fire_state = FIRE_SIN;
        break;
    case DOWN_LONG_rc:
        /* 单发→连发过渡：需先转够一发的角度 */
        if (mode->last_state == FIRE_SIN
            && trig_motor.motor_measure.total_ecd - (float)trig_ecd_set > -1400) {
            mode->fire_state = FIRE_CON;
        } else {
            mode->fire_state = (mode->last_state == FIRE_SIN)
                             ? FIRE_SIN : FIRE_CON;
        }
        break;
    default:
        mode->fire_state = FIRE_NO;
        break;
    }
}

/* ---- PC视觉射击模式 ---- */

void shoot_pc_mode(trig_t *mode)
{
    if (gimbal.vision_on == 1) {
        mode->fire_state = FIRE_CON;
    } else {
        shoot_rc_mode(mode);  /* 无视觉时退化到遥控模式 */
    }
}

/* ---- 射击控制调度 ---- */

void tirg_control(trig_t *mode)
{
    if (control_data.shoot_mode != shoot_on) {
        trig_ecd_set = trig_motor.motor_measure.total_ecd;
        return;
    }

    trig_chose_freq(mode);

    /* 卡弹处理优先 */
    if (mode->trig_flag.if_block_react_over == 0) {
        block_react(mode);
        return;
    }

    switch (mode->fire_state) {
    case FIRE_SIN:
        if (mode->last_state == FIRE_CON) {
            trig_ecd_set = trig_motor.motor_measure.total_ecd;
        }
        trig_sin(mode);
        break;
    case FIRE_CON:
        trig_ecd_set = trig_motor.motor_measure.total_ecd;
        trig_con(mode);
        break;
    case FIRE_VISION:
        trig_vision(mode);
        break;
    case FIRE_NO:
    default:
        if (judge_if_sin_over() == YES) {
            mode->trig_flag.if_sin_request = 1;
            trig_ecd_set = trig_motor.motor_measure.total_ecd;
        }
        break;
    }

    mode->last_state = mode->fire_state;
}

/* ---- 单发 ---- */

void trig_sin(trig_t *mode)
{
    if (mode->trig_flag.if_sin_request != 1) return;
    mode->trig_flag.if_sin_request = 0;

    if (control_data.shoot_power >= TRIG_MAX_POWER - TRIG_NORMOL_POWER_LIMIT) {
        trig_ecd_set = trig_motor.motor_measure.total_ecd;  /* 功率不足，等待 */
    } else {
        trig_ecd_set += TRIG_SIN_ECD;
    }
}

/* ---- 连发 ---- */

void trig_con(trig_t *mode)
{
    if (gimbal.vision_on == 0) {
        trig_speed_set = TRIG_1FPS_SPEED * (float)mode->trig_freq;
    } else {
        /* 视觉模式：根据目标类型选择射频 */
        if (Rx_Vision.fire && Rx_Vision.Fire_Mode > 0) {
            mode->trig_freq = (Rx_Vision.armor_id == ARMOR_OUTPOST)
                            ? CON_FREQ_12 : CON_FREQ_20;
        } else {
            mode->trig_freq = 0;
        }
        trig_speed_set = TRIG_1FPS_SPEED * (float)mode->trig_freq;
    }
}

/* ---- 视觉单发 ---- */

void trig_vision(trig_t *mode)
{
    if (TJ_Vision_Rx.Vision_gimbal_mode == control_fire) {
        if (judge_if_sin_over() == YES) {
            trig_ecd_set += (TRIG_SIN_ECD * 4);
        }
    } else {
        trig_ecd_set = trig_motor.motor_measure.total_ecd;
    }
}

/* ---- 射频自适应 ---- */

void trig_chose_freq(trig_t *mode)
{
    if (control_data.gimbal_mode != small_gimbal_pc) {
        mode->trig_freq = CON_FREQ_20;
        return;
    }

    float limit = (Rx_Vision.armor_id == ARMOR_OUTPOST)
                ? TRIG_OUTPOST_POWER_LIMIT : TRIG_NORMOL_POWER_LIMIT;

    mode->trig_freq = (control_data.shoot_power >= TRIG_MAX_POWER - limit)
                    ? 0 : CON_FREQ_20;
}

/* ======================== 卡弹检测 ======================== */

void get_trig_motor_state(trig_t *mode)
{
    (void)mode;
    judge_if_block();
}

void judge_if_block(void)
{
    /* 堵转条件：大电流 + 低转速 + 未标记卡弹 */
    if (trig_motor.motor_measure.feedback_current > BLOCK_CURRENT_THRESH
        && trig_motor.motor_measure.speed_rpm < BLOCK_SPEED_THRESH
        && trig_motor.motor_measure.speed_rpm >= 0
        && trig.if_block == 0) {
        trig.block_state.cnt++;
    } else {
        trig.block_state.cnt = 0;
    }

    if (trig.block_state.cnt > BLOCK_CNT_THRESH) {
        trig.if_block = 1;
        trig.block_state.block_type = (trig.fire_state == FIRE_CON)
                                    ? CON_BLOCK : SIN_BLOCK;
        trig.trig_flag.if_block_react_over = 0;
    }

    if (trig.if_block == 0 && trig.trig_flag.if_block_react == 1) {
        trig.trig_flag.if_block_react = 0;
    }
}

/* ---- 反卡处理 ---- */

void block_react(trig_t *mode)
{
    if (mode->block_state.block_type == SIN_BLOCK)  sin_block_react(mode);
    if (mode->block_state.block_type == CON_BLOCK)  con_block_react(mode);
}

void sin_block_react(trig_t *mode)
{
    if (mode->trig_flag.if_block_react == 0) {
        trig_ecd_set = trig_motor.motor_measure.total_ecd;
        mode->trig_flag.if_block_react = 1;
    }

    if (mode->trig_flag.if_block_react == 1
        && abs(trig_ecd_set - trig_motor.motor_measure.total_ecd) < 1000) {
        react_time++;
    }
    if (react_time > BLOCK_REACT_TIME) {
        reset_block_flag(mode);
        react_time = 0;
    }
}

void con_block_react(trig_t *mode)
{
    if (mode->trig_flag.if_block_react == 0) {
        trig_ecd_set = trig_motor.motor_measure.total_ecd;
        mode->trig_flag.if_block_react = 1;
    }

    if (mode->trig_flag.if_block_react == 1
        && abs(trig_ecd_set - trig_motor.motor_measure.total_ecd) < 1000) {
        reset_block_flag(mode);
    }
}

void reset_block_flag(trig_t *mode)
{
    mode->block_state.block_type = NO_BLOCK;
    mode->trig_flag.if_block_react_over = 1;
    mode->if_block = 0;
}

/* ======================== 电机执行 ======================== */

void trig_motor_run(trig_t *mode)
{
    if (trig_com != com_nom) {
        trig_motor.motor_tar.set_current = 0;
        return;
    }

    if (mode->fire_state == FIRE_SIN || mode->fire_state == FIRE_VISION) {
        /* 位置控制：编码器目标 */
        trig_motor.motor_tar.set_current = pid_calc(&pid_trig_sin_ecd,
            trig_motor.motor_measure.total_ecd, (float)trig_ecd_set);
    } else if (mode->fire_state == FIRE_CON) {
        /* 速度控制：转速目标 */
        trig_motor.motor_tar.set_current = pid_calc(&pid_trig_con,
            trig_motor.motor_measure.speed_rpm, trig_speed_set);
    } else {
        /* FIRE_NO：保持位置 */
        trig_motor.motor_tar.set_current = pid_calc(&pid_trig_sin_ecd,
            trig_motor.motor_measure.total_ecd, (float)trig_ecd_set);
    }
}

/* ======================== 辅助 ======================== */

bool judge_if_sin_over(void)
{
    return (abs(trig_ecd_set - trig_motor.motor_measure.total_ecd) < 500);
}
