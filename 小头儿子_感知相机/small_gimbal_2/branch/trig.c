/**
 ******************************************************************************
 * @file    trig.c
 * @brief   拨盘触发控制 —— 核心射击执行模块
 *
 * 负责拨弹电机的全部控制逻辑:
 *   1. 射击模式选择 (遥控单发/连发 / PC视觉连发 / 视觉单发)
 *   2. 拨盘状态机调度 (FIRE_NO / FIRE_SIN / FIRE_CON / FIRE_VISION)
 *   3. 卡弹检测与反卡处理 (堵转 -> 卸力 -> 复位)
 *   4. 射频自适应 (根据裁判系统功率限制动态调整射速)
 *
 * ─── 状态机 ───
 *   FIRE_NO     : 空闲, 等待触发信号
 *   FIRE_SIN    : 单发模式, 位置控制转固定角度
 *   FIRE_CON    : 连发模式, 速度控制持续旋转
 *   FIRE_VISION : 视觉单发, 由TJ_Vision_Rx决定击发时机
 *
 * ─── 反卡逻辑 ───
 *   检测: 大电流 + 低转速 + 持续计数 > 阈值
 *   反应: 反向回退 -> 等待卸力 -> 复位堵转标志
 *   目的: 防止拨盘卡死烧毁电机
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

/* ================================================================
 * 全局变量
 * ================================================================ */

/** 拨盘通信状态 (com_nom = 正常通信, com_err = 通信丢失) */
com_mode_t   trig_com;

/** 三个拨盘PID控制器:
 *   sin_ecd  : 单发角度环 (位置控制)
 *   sin_speed: 单发速度环 (备用)
 *   con      : 连发速度环
 */
pid_struct_t pid_trig_sin_ecd;
pid_struct_t pid_trig_sin_speed;
pid_struct_t pid_trig_con;

/** 拨盘目标编码器值 (用于单发位置控制) */
int   trig_ecd_set;

/** 拨盘目标转速 (用于连发速度控制, rpm) */
float trig_speed_set;

/** 全局拨盘状态结构体 */
trig_t trig = {
    .fire_state = FIRE_NO,
    .last_state = FIRE_NO,
    .trig_flag.if_sin_over        = 1,
    .trig_flag.if_con_over        = 1,
    .trig_flag.if_block_react     = 1,
    .trig_flag.if_block_react_over = 1,
};

/* ================================================================
 * 内部状态
 * ================================================================ */

/** 反卡卸力计时器, 单位: 任务周期(1ms) */
static uint16_t react_time;

/* ================================================================
 * 内部函数声明
 * ================================================================ */

/** 从控制数据中同步波轮状态到全局变量 Wheel_State */
static void receive_wheel_state(void);

/* ================================================================
 * 初始化 + 主任务
 * ================================================================ */

/**
 * @brief  初始化三个拨盘PID控制器
 * @note   PID参数说明:
 *         pid_trig_sin_ecd  : Kp=1.0  位置环, 输出限幅4000  (电流值)
 *         pid_trig_sin_speed: Kp=8.0  速度环, 积分Ki=0.0005 (备用)
 *         pid_trig_con      : Kp=25.0 连发速度环, 输出限幅10000
 */
void trig_pid_init(void)
{
    pid_init(&pid_trig_sin_ecd,   1.0f, 0.0f,    0.1f,  10.0f,  4000.0f);
    pid_init(&pid_trig_sin_speed, 8.0f, 0.0005f, 0.0f, 100.0f, 10000.0f);
    pid_init(&pid_trig_con,      25.0f, 0.0f,    0.0f,   0.0f, 10000.0f);
}

/**
 * @brief  拨盘控制主任务 (FreeRTOS线程入口)
 * @note   周期: 1ms
 *         流程: 通信检测 -> 状态机更新 -> 电机执行
 */
void trig_run(void const *argument)
{
    (void)argument;
    vTaskDelay(2);          /* 延迟启动, 等待系统初始化完成 */
    trig_pid_init();
    for (;;)
    {
        get_trig_com();     /* 更新通信状态 */
        trig_task_run(&trig); /* 执行状态机 */
        vTaskDelay(1);
    }
}

/* ================================================================
 * 通信检测
 * ================================================================ */

/**
 * @brief  检测拨盘电机通信是否正常
 * @note   通信正常的条件: 射击模式开启 且 系统通信正常
 *         否则进入 com_err 安全状态
 */
void get_trig_com(void)
{
    trig_com = (control_data.shoot_mode != shoot_no
             && gimbal_system.control_com == com_nom)
             ? com_nom : com_err;
}

/* ================================================================
 * 主状态机
 * ================================================================ */

/**
 * @brief  拨盘状态机主调度 (每个控制周期执行一次)
 * @param  mode 拨盘状态指针
 * @note   执行顺序:
 *         1. 同步波轮状态
 *         2. 选择射击模式 (遥控/视觉)
 *         3. 检测卡弹状态
 *         4. 射击控制调度
 *         5. 电机电流输出
 */
void trig_task_run(trig_t *mode)
{
    receive_wheel_state();     /* 步骤1: 获取遥控器波轮状态 */
    trig_mode_chose(mode);     /* 步骤2: 根据当前模式选择 fire_state */
    get_trig_motor_state(mode);/* 步骤3: 检测是否卡弹 */
    tirg_control(mode);        /* 步骤4: 根据状态机执行射击动作 */
    trig_motor_run(mode);      /* 步骤5: 计算并输出电机电流 */
}

/**
 * @brief  从 control_data 同步波轮状态
 * @note   Wheel_State 是全局变量, 用于 shoot_rc_mode() 判断单发/连发
 */
static void receive_wheel_state(void)
{
    Wheel_State = control_data.other_data.wheel_state;
}

/* ================================================================
 * 模式选择
 * ================================================================ */

/**
 * @brief  根据通信状态和云台模式选择射击子模式
 * @param  mode 拨盘状态指针
 * @note   通信丢失时: 强制进入 FIRE_NO, 电流清零 (安全保护)
 *         遥控模式时: 走 shoot_rc_mode (波轮控制单发/连发)
 *         PC模式时:   走 shoot_pc_mode (视觉控制连发)
 */
void trig_mode_chose(trig_t *mode)
{
    /* 通信异常: 停止一切射击动作, 保护电机 */
    if (trig_com != com_nom)
    {
        mode->fire_state = FIRE_NO;
        trig_ecd_set = trig_motor.motor_measure.total_ecd;  /* 锁定当前位置 */
        trig_motor.motor_tar.set_current = 0;               /* 清零电流 */
        return;
    }

    if (control_data.gimbal_mode == small_gimbal_rc)
    {
        shoot_rc_mode(mode);  /* 遥控器波轮控制 */
    }
    else
    {
        shoot_pc_mode(mode);  /* PC/视觉控制 */
    }
}

/* ================================================================
 * 遥控器射击模式
 * ================================================================ */

/**
 * @brief  遥控器射击模式 —— 根据波轮状态切换单发/连发/空闲
 * @param  mode 拨盘状态指针
 * @note   波轮状态映射:
 *         ZERO_rc       -> 空闲 (不射击)
 *         DOWN_SHORT_rc -> 单发 (转固定的 TRIG_SIN_ECD 角度)
 *         DOWN_LONG_rc  -> 连发 (持续旋转, 首次需先转够单发角度过渡)
 */
void shoot_rc_mode(trig_t *mode)
{
    switch (Wheel_State)
    {
    case ZERO_rc:
    {
        /* 波轮归中: 停止射击, 预置单发请求以便下次快速响应 */
        mode->fire_state = FIRE_NO;
        mode->trig_flag.if_sin_request = 1;
        break;
    }
    case DOWN_SHORT_rc:
    {
        /* 波轮下拨短按: 触发单发 */
        mode->fire_state = FIRE_SIN;
        break;
    }
    case DOWN_LONG_rc:
    {
        /* 波轮下拨长按: 触发连发
         * 过渡逻辑: 如果上一个是单发且还未转够一发角度, 则继续单发
         * 等转够后自动进入连发, 避免中途切换导致弹丸错位 */
        if (mode->last_state == FIRE_SIN
            && trig_motor.motor_measure.total_ecd - (float)trig_ecd_set > -1400)
        {
            mode->fire_state = FIRE_CON;  /* 单发已完成, 切换到连发 */
        }
        else
        {
            mode->fire_state = (mode->last_state == FIRE_SIN)
                             ? FIRE_SIN : FIRE_CON;
        }
        break;
    }
    default:
    {
        /* 未知状态: 安全起见停止射击 */
        mode->fire_state = FIRE_NO;
        break;
    }
    }
}

/* ================================================================
 * PC视觉射击模式
 * ================================================================ */

/**
 * @brief  PC/视觉射击模式 —— 视觉锁定时连发, 否则退化为遥控模式
 * @param  mode 拨盘状态指针
 * @note   视觉模式 (vision_on == 1) : 持续连发
 *         无视觉 (vision_on == 0) : 退化为遥控器波轮控制
 */
void shoot_pc_mode(trig_t *mode)
{
    if (gimbal.vision_on == 1)
    {
        mode->fire_state = FIRE_CON;  /* 视觉锁定: 连发输出 */
    }
    else
    {
        shoot_rc_mode(mode);  /* 无视觉时退化到遥控模式 */
    }
}

/* ================================================================
 * 射击控制调度
 * ================================================================ */

/**
 * @brief  射击控制核心调度 —— 根据当前 fire_state 执行对应动作
 * @param  mode 拨盘状态指针
 * @note   优先级:
 *         1. 射击关闭: 保持位置不动
 *         2. 卡弹处理: 优先反卡, 阻断正常射击
 *         3. 正常射击: 根据 fire_state 走单发/连发/视觉分支
 */
void tirg_control(trig_t *mode)
{
    /* 射击开关关闭: 保持编码器当前位置不动 */
    if (control_data.shoot_mode != shoot_on)
    {
        trig_ecd_set = trig_motor.motor_measure.total_ecd;
        return;
    }

    trig_chose_freq(mode);  /* 先根据功率限制更新射频 */

    /* 卡弹处理最高优先级: 阻断正常射击流程 */
    if (mode->trig_flag.if_block_react_over == 0)
    {
        block_react(mode);
        return;
    }

    /* 正常射击状态分发 */
    switch (mode->fire_state)
    {
    case FIRE_SIN:
    {
        /* 单发模式: 从连发切过来时重置目标位置 */
        if (mode->last_state == FIRE_CON)
        {
            trig_ecd_set = trig_motor.motor_measure.total_ecd;
        }
        trig_sin(mode);
        break;
    }
    case FIRE_CON:
    {
        /* 连发模式: 每次进入时用当前位置作为基准 */
        trig_ecd_set = trig_motor.motor_measure.total_ecd;
        trig_con(mode);
        break;
    }
    case FIRE_VISION:
    {
        /* 视觉单发: 由 TJ_Vision_Rx 控制击发时机 */
        trig_vision(mode);
        break;
    }
    case FIRE_NO:
    default:
    {
        /* 空闲: 待单发完成标志置位后, 准备好下一次单发请求 */
        if (judge_if_sin_over() == YES)
        {
            mode->trig_flag.if_sin_request = 1;
            trig_ecd_set = trig_motor.motor_measure.total_ecd;
        }
        break;
    }
    }

    /* 记录本次状态, 供下一周期判断状态切换 */
    mode->last_state = mode->fire_state;
}

/* ================================================================
 * 单发
 * ================================================================ */

/**
 * @brief  单发控制 —— 每次调用增加固定编码器偏移
 * @param  mode 拨盘状态指针
 * @note   - 功率不足时: 保持当前位置等待 (避免欠功率导致堵转)
 *         - 功率充足时: trig_ecd_set += TRIG_SIN_ECD (转动一发的角度)
 *         - TRIG_SIN_ECD = 32700, 即拨盘转动约一发的编码器增量
 */
void trig_sin(trig_t *mode)
{
    if (mode->trig_flag.if_sin_request != 1)
    {
        return;  /* 未请求单发, 不执行 */
    }
    mode->trig_flag.if_sin_request = 0;  /* 消费请求标志 */

    /* 功率检查: 当前功率 >= 最大功率 - 常规功率限制 时说明功率不足 */
    if (control_data.shoot_power >= TRIG_MAX_POWER - TRIG_NORMOL_POWER_LIMIT)
    {
        trig_ecd_set = trig_motor.motor_measure.total_ecd;  /* 功率不足, 等待 */
    }
    else
    {
        trig_ecd_set += TRIG_SIN_ECD;  /* 增加一发弹的编码器偏移 */
    }
}

/* ================================================================
 * 连发
 * ================================================================ */

/**
 * @brief  连发控制 —— 设置拨盘目标转速
 * @param  mode 拨盘状态指针
 * @note   - 遥控模式: 使用固定射频 CON_FREQ_20
 *         - 视觉模式: 根据目标类型动态选择射频
 *           * 前哨站 (ARMOR_OUTPOST): CON_FREQ_12 (12发/秒)
 *           * 其他装甲板:            CON_FREQ_20 (20发/秒)
 *         - 转速换算: trig_speed_set = TRIG_1FPS_SPEED * freq
 *           TRIG_1FPS_SPEED = 2160/8 = 270 rpm/发每秒
 */
void trig_con(trig_t *mode)
{
    if (gimbal.vision_on == 0)
    {
        /* 遥控连发: 固定射速 */
        trig_speed_set = TRIG_1FPS_SPEED * (float)mode->trig_freq;
    }
    else
    {
        /* 视觉连发: 根据目标装甲板类型动态选择射频 */
        if (Rx_Vision.fire && Rx_Vision.Fire_Mode > 0)
        {
            mode->trig_freq = (Rx_Vision.armor_id == ARMOR_OUTPOST)
                            ? CON_FREQ_12 : CON_FREQ_20;
        }
        else
        {
            mode->trig_freq = 0;  /* 视觉未触发, 不发射 */
        }
        trig_speed_set = TRIG_1FPS_SPEED * (float)mode->trig_freq;
    }
}

/* ================================================================
 * 视觉单发
 * ================================================================ */

/**
 * @brief  视觉单发模式 —— 由上位机 TJ_Vision_Rx 控制击发
 * @param  mode 拨盘状态指针
 * @note   - control_fire 模式下: 每次射击转4发弹的角度
 *         - 非 control_fire 模式: 保持位置不动
 */
void trig_vision(trig_t *mode)
{
    if (TJ_Vision_Rx.Vision_gimbal_mode == control_fire)
    {
        if (judge_if_sin_over() == YES)
        {
            trig_ecd_set += (TRIG_SIN_ECD * 4);  /* 一次转4发 */
        }
    }
    else
    {
        trig_ecd_set = trig_motor.motor_measure.total_ecd;  /* 待命, 不动 */
    }
}

/* ================================================================
 * 射频自适应
 * ================================================================ */

/**
 * @brief  射频自适应 —— 根据裁判系统功率限制动态调整射速
 * @param  mode 拨盘状态指针
 * @note   策略:
 *         - 遥控模式: 固定 CON_FREQ_20 (不限制功率)
 *         - PC模式:
 *           * 前哨站: 功率限制 250W (TRIG_OUTPOST_POWER_LIMIT)
 *           * 常规:   功率限制  80W (TRIG_NORMOL_POWER_LIMIT)
 *           * 功率不足时: trig_freq = 0 (暂停射击, 等待功率恢复)
 */
void trig_chose_freq(trig_t *mode)
{
    /* 遥控模式不限制射频 */
    if (control_data.gimbal_mode != small_gimbal_pc)
    {
        mode->trig_freq = CON_FREQ_20;
        return;
    }

    /* PC模式: 根据目标类型选择功率上限 */
    float limit = (Rx_Vision.armor_id == ARMOR_OUTPOST)
                ? TRIG_OUTPOST_POWER_LIMIT : TRIG_NORMOL_POWER_LIMIT;

    /* 当前功率 >= 最大功率 - 限制值 时, 暂停射击等待恢复 */
    mode->trig_freq = (control_data.shoot_power >= TRIG_MAX_POWER - limit)
                    ? 0 : CON_FREQ_20;
}

/* ================================================================
 * 卡弹检测
 * ================================================================ */

/**
 * @brief  更新拨盘电机状态 (卡弹检测入口)
 * @param  mode 拨盘状态指针 (当前未使用, 保留接口)
 */
void get_trig_motor_state(trig_t *mode)
{
    (void)mode;
    judge_if_block();  /* 每次调用都检查是否卡弹 */
}

/**
 * @brief  卡弹判定逻辑
 * @note   判定条件 (三个同时满足):
 *         1. 反馈电流 > BLOCK_CURRENT_THRESH (7000)  —— 电机出力过大
 *         2. 实际转速   < BLOCK_SPEED_THRESH   (50)    —— 电机转不动
 *         3. 转速 >= 0                                 —— 非反转状态
 *         4. 未标记卡弹 (trig.if_block == 0)
 *
 *         计数超过 BLOCK_CNT_THRESH (70ms) 后触发卡弹,
 *         根据当前射击模式区分 SIN_BLOCK(单发卡弹) / CON_BLOCK(连发卡弹)
 */
void judge_if_block(void)
{
    /* 堵转条件检测 */
    if (trig_motor.motor_measure.feedback_current > BLOCK_CURRENT_THRESH
        && trig_motor.motor_measure.speed_rpm < BLOCK_SPEED_THRESH
        && trig_motor.motor_measure.speed_rpm >= 0
        && trig.if_block == 0)
    {
        trig.block_state.cnt++;  /* 满足堵转条件, 累加计数 */
    }
    else
    {
        trig.block_state.cnt = 0;  /* 不满足, 清零计数 */
    }

    /* 持续堵转超过阈值: 标记卡弹 */
    if (trig.block_state.cnt > BLOCK_CNT_THRESH)
    {
        trig.if_block = 1;
        trig.block_state.block_type = (trig.fire_state == FIRE_CON)
                                    ? CON_BLOCK : SIN_BLOCK;
        trig.trig_flag.if_block_react_over = 0;  /* 触发反卡流程 */
    }

    /* 正常状态下清除反卡标志 */
    if (trig.if_block == 0 && trig.trig_flag.if_block_react == 1)
    {
        trig.trig_flag.if_block_react = 0;
    }
}

/* ================================================================
 * 反卡处理
 * ================================================================ */

/**
 * @brief  反卡总调度 —— 根据卡弹类型分发处理
 * @param  mode 拨盘状态指针
 * @note   单发卡弹: sin_block_react (反转 + 延时等待)
 *         连发卡弹: con_block_react (反转 + 等待卸力)
 */
void block_react(trig_t *mode)
{
    if (mode->block_state.block_type == SIN_BLOCK)
    {
        sin_block_react(mode);
    }
    if (mode->block_state.block_type == CON_BLOCK)
    {
        con_block_react(mode);
    }
}

/**
 * @brief  单发卡弹反卡 —— 反转电机 + 延时卸力
 * @param  mode 拨盘状态指针
 * @note   步骤:
 *         1. 记录当前位置, 设置目标为当前位置 (电机停止出力)
 *         2. 等待电机停止 (目标位置与实际位置差 < 1000)
 *         3. 延时 BLOCK_REACT_TIME (50ms) 让弹丸自然脱落
 *         4. 复位卡弹标志, 恢复正常射击
 */
void sin_block_react(trig_t *mode)
{
    /* 首次进入反卡: 记录当前位置, 停止运动 */
    if (mode->trig_flag.if_block_react == 0)
    {
        trig_ecd_set = trig_motor.motor_measure.total_ecd;
        mode->trig_flag.if_block_react = 1;
    }

    /* 等待电机停止 (位置误差 < 1000 编码器单位) */
    if (mode->trig_flag.if_block_react == 1
        && abs(trig_ecd_set - trig_motor.motor_measure.total_ecd) < 1000)
    {
        react_time++;  /* 电机已停止, 开始计时卸力 */
    }

    /* 卸力时间到: 复位卡弹状态 */
    if (react_time > BLOCK_REACT_TIME)
    {
        reset_block_flag(mode);
        react_time = 0;
    }
}

/**
 * @brief  连发卡弹反卡 —— 反转电机立即卸力
 * @param  mode 拨盘状态指针
 * @note   连发模式下卡弹后立即停止目标运动,
 *         等待电机停下来后直接复位 (不需要额外延时)
 */
void con_block_react(trig_t *mode)
{
    /* 首次进入反卡: 记录当前位置, 停止运动 */
    if (mode->trig_flag.if_block_react == 0)
    {
        trig_ecd_set = trig_motor.motor_measure.total_ecd;
        mode->trig_flag.if_block_react = 1;
    }

    /* 电机停止后立刻复位 */
    if (mode->trig_flag.if_block_react == 1
        && abs(trig_ecd_set - trig_motor.motor_measure.total_ecd) < 1000)
    {
        reset_block_flag(mode);
    }
}

/**
 * @brief  复位卡弹状态 —— 清除所有卡弹标志
 * @param  mode 拨盘状态指针
 * @note   调用后恢复正常射击流程
 */
void reset_block_flag(trig_t *mode)
{
    mode->block_state.block_type = NO_BLOCK;
    mode->trig_flag.if_block_react_over = 1;  /* 反卡流程结束 */
    mode->if_block = 0;                       /* 清除卡弹标记 */
}

/* ================================================================
 * 电机执行
 * ================================================================ */

/**
 * @brief  拨盘电机电流输出 —— 根据当前状态选择控制模式
 * @param  mode 拨盘状态指针
 * @note   控制模式映射:
 *         - FIRE_SIN / FIRE_VISION : 位置环 (pid_trig_sin_ecd)
 *         - FIRE_CON               : 速度环 (pid_trig_con)
 *         - FIRE_NO / 通信异常      : 位置保持 (pid_trig_sin_ecd, 目标=当前位置)
 */
void trig_motor_run(trig_t *mode)
{
    /* 通信丢失: 电机断电保护 */
    if (trig_com != com_nom)
    {
        trig_motor.motor_tar.set_current = 0;
        return;
    }

    if (mode->fire_state == FIRE_SIN || mode->fire_state == FIRE_VISION)
    {
        /* 单发/视觉单发: 位置控制, 目标 = trig_ecd_set */
        trig_motor.motor_tar.set_current = pid_calc(&pid_trig_sin_ecd,
            trig_motor.motor_measure.total_ecd, (float)trig_ecd_set);
    }
    else if (mode->fire_state == FIRE_CON)
    {
        /* 连发: 速度控制, 目标 = trig_speed_set */
        trig_motor.motor_tar.set_current = pid_calc(&pid_trig_con,
            trig_motor.motor_measure.speed_rpm, trig_speed_set);
    }
    else
    {
        /* 空闲: 位置保持, 目标 = 当前位置 (锁定不动) */
        trig_motor.motor_tar.set_current = pid_calc(&pid_trig_sin_ecd,
            trig_motor.motor_measure.total_ecd, (float)trig_ecd_set);
    }
}

/* ================================================================
 * 辅助
 * ================================================================ */

/**
 * @brief  判断单发是否完成 (当前位置是否到达目标位置)
 * @return true=已完成, false=运动中
 * @note   判定条件: |目标编码器 - 当前编码器| < 500
 */
bool judge_if_sin_over(void)
{
    return (abs(trig_ecd_set - trig_motor.motor_measure.total_ecd) < 500);
}