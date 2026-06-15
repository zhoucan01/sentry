# 哨兵机器人全栈控制系统

> RoboMaster 哨兵机器人 — 底盘 + 双云台 + 导航决策 完整固件

## 项目概述

本仓库包含一台哨兵机器人的 **全部嵌入式固件**，由三个独立 STM32F407 工程组成，分别负责底盘运动控制、大Yaw云台感知、小云台射击控制，三者通过 CAN 总线协同工作。

```
┌─────────────────────────────────────────────────────┐
│                    哨兵机器人                         │
│                                                      │
│  ┌──────────────┐  ┌──────────────┐  ┌────────────┐ │
│  │ steer_chassis│  │ 大yaw_感知相机│  │小头儿子_    │ │
│  │   底盘控制    │  │ 导航+视觉云台 │  │ 感知相机    │ │
│  │  (STM32F407) │  │ (STM32F407)  │  │ (STM32F407) │ │
│  └──────┬───────┘  └──────┬───────┘  └─────┬──────┘ │
│         │                 │                 │         │
│         └────────┬────────┴────────┬────────┘         │
│                  │    CAN Bus     │                   │
│           ┌──────┴──────┐  ┌──────┴──────┐           │
│           │ 4×M3508 底盘│  │ 4×6020 舵机 │           │
│           └─────────────┘  └─────────────┘           │
└─────────────────────────────────────────────────────┘
```

---

## 工程一：`steer_chassis` — 底盘舵轮运动控制

### 功能
全向舵轮底盘运动学解算、PID 控制、功率限制、里程计观测

### 硬件
| 组件 | 型号 | 数量 |
|------|------|------|
| 主控 | STM32F407IGHx | 1 |
| 底盘电机 | M3508 | 4 |
| 舵机电机 | 6020 | 4 |
| 超电 | 超级电容模块 | 1 |

### 控制流水线 (1ms)
```
坐标系旋转 → 逆运动学 → 舵角(atan2+劣弧) → 速度合成 → PID → 功率限制
```

### 核心文件
| 文件 | 功能 |
|------|------|
| `branch/system.c/h` | 系统任务、模式调度、角度解算、离线检测 |
| `branch/sentry_chassis.c/h` | 运动控制核心：速度解算/逆运动学/功率限制/PID |
| `branch/Odometer.c/h` | 里程计观测器：线性卡尔曼滤波 + RMS打滑检测 |
| `branch/motor.c/h` | 电机 CAN 通信（正常/错误帧切换） |
| `branch/SuperCAP.c/h` | 超级电容管理 |
| `bsp/boards/bsp_transmit.c/h` | 串口协议：上位机指令解析 + 状态回传 |
| `bsp/boards/bsp_pid.c/h` | PID 控制器 |

---

## 工程二：`大yaw_感知相机` — 导航决策 + 视觉云台

### 功能
哨兵导航决策、大 Yaw 轴云台控制、视觉追踪、哨兵行为状态机

### 核心文件
| 文件 | 功能 |
|------|------|
| `branch/decision.c/h` | 哨兵电控决策：巡逻/追击/回防状态机 |
| `branch/navigation.c/h` | 导航信息处理：坐标系转换、路径规划 |
| `branch/gimbal_big_yaw.c/h` | 大 Yaw 云台 PID 控制（角度环 + 速度环） |
| `branch/chassis_control.c/h` | 底盘控制指令生成 |
| `branch/omni.c/h` | 全向移动解算 |
| `branch/small_gimbal.c/h` | 小云台联动控制 |
| `branch/system.c/h` | 大 Yaw 系统任务调度 |

### 关键特性
- **导航坐标系转换**：红/蓝方自动识别，坐标变换
- **决策状态机**：巡逻 → 发现目标 → 追击 → 丢失 → 回防
- **视觉追踪**：支持 Nautilus_Vision 视觉模块
- **USB 虚拟串口**：与上位机/迷你PC通信

---

## 工程三：`小头儿子_感知相机` — 小云台 + 射击控制

### 功能
小云台 Yaw/Pitch 双轴控制、摩擦轮射击、弹道补偿

### 核心文件
| 文件 | 功能 |
|------|------|
| `small_gimbal_2/branch/gimbal.c/h` | 小云台 Yaw/Pitch PID 控制 |
| `small_gimbal_2/branch/shoot.c/h` | 摩擦轮射击控制（单发/连发） |
| `small_gimbal_2/branch/system.c/h` | 小云台系统任务调度 |
| `small_gimbal_2/branch/trig.c/h` | 三角函数快速计算 |
| `small_gimbal_2/branch/motor.c/h` | 电机 CAN 通信 |

### 关键特性
- **双轴云台**：Yaw + Pitch 级联 PID
- **射击控制**：拨弹 + 摩擦轮调速
- **视觉联动**：配合 Nautilus_Vision 自瞄

---

## 编译

| 项目 | IDE | 编译器 | 目标 |
|------|-----|--------|------|
| 全部 | Keil MDK-ARM V5 | ARM Compiler V5.06u7 | STM32F407IGHx |

---

## 优化记录 (`zc_withai` 分支)

| 文件 | 改动 |
|------|------|
| `steer_chassis/branch/sentry_chassis.c` | 全面重构：提取 `RotateSpeedToBase`/`CalcMotorPower`/`ApplyPowerLimit`/`ResolveSteerShortArc` 四个公共函数，消除 ~200 行重复代码 |
| `steer_chassis/branch/system.c` | `get_diff_angle` 从 40 行 → 10 行，`NormAngle` 归一化 |
| `steer_chassis/branch/Odometer.c` | 补充 `Odometer_run` FreeRTOS 任务入口 |
| 全部头文件 | Doxygen 注释，`(void)` 规范化，消除编译警告 |
| **效果** | 49→0 Warning，代码段减少 ~20% |

