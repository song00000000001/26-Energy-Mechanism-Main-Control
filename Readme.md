# 26神符主控 — Energy Mechanism Main Control

> SCUT RobotLab · RoboMaster 2026 赛季 · 能量机关主控板软件

---

## 目录

- [项目简介](#项目简介)
- [硬件平台](#硬件平台)
- [软件架构](#软件架构)
- [目录结构](#目录结构)
- [通信接口](#通信接口)
- [系统状态机](#系统状态机)
- [快速开始](#快速开始)
- [子模块说明](#子模块说明)
- [调试说明](#调试说明)

---

## 项目简介

本仓库为 **26赛季神符（能量机关）主控板** 的嵌入式软件工程。项目源于 `26-missile`，在其通用任务框架基础上进行了大规模裁剪与重构：

- **精简**无关外设模块
- **重构**状态机逻辑（小符 / 大符 / 成功等多状态）
- **替换**电机驱动方案（改用达妙 DM 系列电机）
- **优化**通信链路（上位机、分控、电机三路 CAN/UART 独立管理）

---

## 硬件平台

| 参数 | 规格 |
|---|---|
| 主控芯片 | STM32F405RGT6（LQFP64） |
| 主频 | 168 MHz（HSE 25 MHz，PLL 倍频） |
| Flash | 1 MB |
| RAM | 192 KB（FreeRTOS 堆 80 KB） |
| 编译工具链 | Keil MDK-ARM V5.32 / GCC |
| HAL 固件包 | STM32Cube FW_F4 V1.28.3 |
| IDE 配置文件 | `Template.ioc`（STM32CubeMX 6.15.0） |

---

## 软件架构

```
┌─────────────────────────────────────────────┐
│           用户应用层 (USP/Application)         │
│  状态机任务 │ 电机控制 │ 装甲板控制 │ 小符/大符逻辑  │
├─────────────────────────────────────────────┤
│           服务层 (USP/Services)               │
│  通信服务 (CAN/UART) │ 调试服务 │ 设备初始化服务   │
├─────────────────────────────────────────────┤
│           中间件层 (SRML)                      │
│  drv_uart │ drv_can │ 电机驱动 │ PID │ 滤波器    │
├─────────────────────────────────────────────┤
│           HAL 驱动层 (Drivers)                │
│      STM32F4xx HAL Driver + CMSIS            │
├─────────────────────────────────────────────┤
│             实时操作系统                        │
│                  FreeRTOS (CMSIS V1)          │
└─────────────────────────────────────────────┘
```

操作系统使用 **FreeRTOS**，各功能模块以独立任务运行，通过 FreeRTOS 队列（Queue）进行任务间通信。

---

## 目录结构

```
.
├── Core/                   # STM32CubeMX 自动生成的 HAL 初始化代码
│   ├── Inc/                #   头文件（main.h、外设配置头文件等）
│   └── Src/                #   源文件（main.c、外设初始化、中断处理等）
├── Drivers/                # STM32F4xx HAL 驱动 + CMSIS
├── MDK-ARM/                # Keil MDK 工程文件
├── Middlewares/            # STM32 USB 中间件
├── SRML/                   # 实验室中间件库（Git 子模块）
├── USB_DEVICE/             # USB CDC 虚拟串口
├── USP/                    # 用户代码目录
│   ├── Application/        #   应用层
│   │   └── Robot_Module/   #     机器人核心模块
│   │       ├── state_machine_task.cpp   # 主状态机任务
│   │       ├── motor_ctrl_task.cpp      # 电机控制任务
│   │       ├── armer_ctrl_task.cpp      # 装甲板控制任务
│   │       ├── small_enegy_logic.cpp    # 小能量机关逻辑
│   │       ├── big_enegy_logic.cpp      # 大能量机关逻辑
│   │       ├── global_data.h/.cpp       # 全局数据定义
│   │       ├── robot_config.h           # 机器人配置（ID、引脚等）
│   │       └── config.h                 # 系统初始化接口
│   │   ├── Service_Communication.cpp   # 通信服务（CAN/UART）
│   │   ├── Service_Device.cpp          # 设备初始化服务
│   │   ├── Service_Debug.cpp           # 调试服务
│   │   ├── System_Config.cpp           # 系统设备 & 任务初始化
│   │   └── System_DataPool.cpp         # 全局数据池
│   ├── Services/           #   自定义通信协议
│   │   ├── protocol.h/.cpp             # 通信协议
│   │   └── can_comm_protocal.h/.cpp    # CAN 通信协议
│   └── Middlewares/        #   调试中间件
│       └── Systemview/     #     SEGGER SystemView
├── Template.ioc            # STM32CubeMX 工程配置
└── Readme.md               # 本文档
```

---

## 通信接口

| 接口 | 功能 | 波特率 / 速率 |
|---|---|---|
| **CAN1** | 达妙（DM）电机控制与反馈（ID: 0x202） | 1 Mbps |
| **CAN2** | 分控板通信（分控 ID 1-5，发 0x210+ID，收 0x220+ID） | 1 Mbps |
| **USART1** | 上位机（手机 App）控制 | 115200 bps |
| **USART2** | 遥控器（DR16 DBUS / 富斯 FS-I6X） | 100000 bps |
| **USART4** | 预留 | 默认 |
| **USART5** | 预留 | 默认 |
| **USART6** | VOFA+ 调试数据上传 | 115200 bps |
| **USB CDC** | 虚拟串口（调试 / OpenLog） | — |

### 上位机协议（USART1）

| Header | Content | 说明 |
|---|---|---|
| `0xA5` | 目标模式（0-5） | 切换能量机关运行模式（0:停止, 1:准备, 2:小符, 3:大符, 4:连续小符, 5:连续大符） |
| `0xA0` | `0x00` | 切换灯光颜色（红/蓝） |
| | `0x01` | 切换电机使能/失能状态 |
| | `0x02` | 切换“超时自动重置”功能 |
| `0xFF` | 无意义 | 触发一次模拟击打事件（调试用） |

### 分控 CAN2 协议

**主控 → 分控（控制包，ID = 0x210 + 分控ID，3字节）**
- 依靠分控id区分扇叶。

| Byte | 含义 |
|---|---|
| 0 | 灯光效果（`LightEffectId_t`） |
| 1 | 颜色 |
| 2 | 大符组数阶段 |

**分控 → 主控（反馈包，ID = 0x220 + 分控ID，2字节）**

| Byte | 含义 |
|---|---|
| 0 | 被击打的环索引（1-10） |
| 1 | 预留 |

---

## 系统状态机

主状态机运行在 `task_state_machine` 任务中，频率 **100 Hz**（10 ms/次）。

```
                ┌────────┐
    tar_stop ──►│  idle  │
                └────┬───┘
                     │ tar_start
                ┌────▼────────┐
                │  wait_start │
                └────┬────────┘
         ┌───────────┼───────────┐
         │tar_small  │tar_big    │tar_test
    ┌────▼────┐  ┌───▼────┐  ┌──▼──────┐
    │  small  │  │  big   │  │  test   │
    │ energy  │  │ energy │  │  mode   │
    └────┬────┘  └───┬────┘  └─────────┘
         └─────┬─────┘
               │ tar_success
          ┌────▼───┐
          │success │ (闪烁7次)
          └────┬───┘
               │ 自动转换
          ┌────▼──────┐
          │ successed │ (永久全亮)
          └───────────┘
```

| 状态 | 说明 |
|---|---|
| `idle` | 待机，熄灭所有灯效 |
| `wait_start` | 等待激活，目标速度置为 0.1 |
| `small_energy` | 小能量机关：随机生成单目标序列，依序点亮并等待击打 |
| `big_energy` | 大能量机关：正弦速度曲线，每轮生成两个连击目标 |
| `success` | 成功：全部灯效闪烁 7 次 |
| `successed` | 成功后保持：永久全亮 |
| `test_mode` | 测试模式（保留） |

---

## 快速开始

### 克隆仓库（含子模块）

```bash
git clone <repo-url>
cd 26-Energy-Mechanism-Main-Control
git submodule update --init --recursive
```

### 编译与烧录

1. 使用 **Keil MDK-ARM V5.32** 打开 `MDK-ARM/` 目录下的工程文件（`.uvprojx`）。
2. 确认目标芯片为 **STM32F405RGTx**。
3. 编译（Build）并通过 SWD 接口烧录。

### 配置 SRML

1. 将 `SRML/srml_config_template.h` 复制到用户代码目录，并重命名为 `srml_config.h`（已存在于 `USP/srml_config.h`）。
2. 根据实际使用的模块，在 `srml_config.h` 中通过预编译宏选择性开启/关闭各功能。

---

## 子模块说明

本项目以 Git 子模块方式引用 **SRML**（SCUT Robotlab Middlewares Layer Library）。

| 命令 | 说明 |
|---|---|
| `git submodule update --init --recursive` | 初始化并拉取子模块 |
| `cd SRML && git pull origin master` | 更新 SRML 库到最新版本 |

SRML 提供的核心模块包括：

- 底层驱动：`drv_uart`、`drv_can`、`drv_timer`
- 电机驱动：大疆电机（DJI）、达妙电机（DM）
- 算法：PID、滤波器、卡尔曼滤波
- 调试：Easylog 日志库

详细文档请参考 `SRML/README.md`。

---

## 调试说明

### SEGGER SystemView

本工程集成了 **SEGGER SystemView**，可实时可视化 FreeRTOS 任务调度。在 `main.c` 中通过 `SEGGER_SYSVIEW_Conf()` 初始化，需在 RTOS 内核启动前调用。

### VOFA+ 数据可视化

通过 **USART6**（115200 bps）输出调试数据至 VOFA+ 上位机软件，便于实时监控电机速度、PID 输出等关键变量。

### 调试模式（Watch 窗口）

在 Keil 调试器的 Watch 窗口中，可修改以下全局变量触发调试功能：

| 变量 | 类型 | 说明 |
|---|---|---|
| `Debugger.enable_debug_mode` | `Debug_Mode_e` | 进入调试模式 |
| `Debugger.Debug_simulate_hit` | `bool` | 置 `true` 模拟一次击打事件 |
| `Debugger.debug_arm_light_effect` | `uint8_t` | 切换灯效（调试模式下） |
| `g_TargetCtrl.target_mode` | `EnergyTargetMode_t` | 手动切换目标运行模式 |
| `g_TargetCtrl.TargetColor` | `light_color_enum` | 手动切换目标颜色 |
