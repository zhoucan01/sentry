# 哨兵机器人底盘控制系统

> RoboMaster 哨兵机器人全向舵轮底盘运动控制固件

## 项目简介

基于 STM32F407 + FreeRTOS 的全向舵轮底盘控制系统，实现四轮独立转向（舵角控制）+ 独立驱动（轮速控制）的完整运动学解算、PID 控制与功率管理。

## 硬件平台

| 组件 | 型号 | 数量 |
|------|------|------|
| 主控 | STM32F407IGHx | 1 |
| 底盘电机 | M3508 直流无刷 | 4 |
| 舵机电机 | 6020 直流无刷 | 4 |
| IMU | BMI088 | 1 |
| 超电 | 超级电容模块 | 1 |

## 软件架构

```
steer_chassis/
├── Core/              # STM32 HAL 层 + FreeRTOS 配置
├── branch/            # 核心业务逻辑
│   ├── system.c/h         # 系统任务 + 模式调度 + 角度解算
│   ├── sentry_chassis.c/h # 底盘运动控制（速度解算/逆运动学/功率限制/PID）
│   ├── Odometer.c/h       # 里程计观测器（线性卡尔曼滤波 + RMS打滑检测）
│   ├── motor.c/h          # 电机CAN通信
│   └── SuperCAP.c/h       # 超级电容管理
├── bsp/boards/        # 板级外设驱动
│   ├── bsp_transmit.c/h   # 串口收发（上位机通信协议）
│   ├── bsp_pid.c/h        # PID控制器
│   └── bsp_can.c/h        # CAN总线驱动
├── application/       # 应用层模块
│   ├── CAN_receive.c/h    # CAN接收（电机反馈解析）
│   └── ins_task.c/h       # 惯导任务
└── MDK-ARM/           # Keil MDK-ARM V5 工程
```

## 控制流水线

每 1ms 执行一次完整的控制循环：

```
[1] 坐标系旋转    → 云台速度 → 底盘坐标系
[2] 逆运动学分解  → 底盘 vx/vy/ω → 四轮 vx/vy
[3] 舵角解算      → atan2 + 劣弧优化 + 编码器转换
[4] 速度合成修正  → 方向修正 + cos? 衰减
[5] PID 计算      → 舵角位置环 → 舵速环 → 轮速环
[6] 功率限制      → 超电自适应功率分配
```

## 核心算法

### 里程计观测器 (`Odometer.c`)

基于线性卡尔曼滤波器的底盘速度估计：

- **状态**: $x = [v_x, v_y, \omega]^T$
- **测量**: 四轮编码器轮速
- **自适应噪声**: RMS 残差反馈 → 打滑轮子自动降权
- **参考论文**: Batch-LIWO, 式 3.84~3.96

### 功率限制 (`sentry_chassis.c`)

电机功率模型反解：

$$P = k_p \cdot I \cdot \omega + k_w \cdot \omega^2 + k_i \cdot I^2 + C$$

超限时等比例缩放，通过二次方程反解目标电流。

## 编译

- **IDE**: Keil MDK-ARM V5
- **编译器**: ARM Compiler V5.06 update 7
- **目标**: STM32F407IGHx

编译状态：**0 Error, 0 Warning** ?

## 分支说明

| 分支 | 说明 |
|------|------|
| `main` | 原始代码 |
| `zc_withai` | AI 辅助优化版本（代码美化 + 重构） |

## 优化记录 (`zc_withai`)

- `sentry_chassis.c`: 提取 4 个公共辅助函数，消除 ~200 行重复代码
- `system.c`: `get_diff_angle` 从 40 行 → 10 行，使用 `NormAngle` 归一化
- `Odometer.c`: 补充 `Odometer_run` FreeRTOS 任务入口
- 消除所有编译警告（49 → 0），代码段减少 ~20%
