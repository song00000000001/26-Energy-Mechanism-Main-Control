# Changelog

最新的修改写在最上面，方便快速查找

## [3.0] - 2023-09-10

### 迁移为AC6编译环境，支持C++14

### 目录树：

```
D:.
│  .gitignore
│  CHANGELOG.md
│  README.md
│  SRML.h
│  srml_config_template.h
│  srml_std_lib.cpp
│  srml_std_lib.h
│
├─Drivers
│  ├─Components
│  │      drv_can.c
│  │      drv_can.h
│  │      drv_flash.c
│  │      drv_flash.h
│  │      drv_i2c.c
│  │      drv_i2c.h
│  │      drv_memory.cpp
│  │      drv_spi.c
│  │      drv_spi.h
│  │      drv_timer.c
│  │      drv_timer.h
│  │      drv_uart.c
│  │      drv_uart.h
│  │      drv_VirtualCom.c
│  │      drv_VirtualCom.h
│  │      SRML_Timer.cpp
│  │      SRML_Timer.h
│  │
│  └─Devices
│      ├─BMX055
│      │      BMX055.c
│      │      BMX055.h
│      │      BMX055_Config.cpp
│      │      BMX055_Config.h
│      │      BMX055_Processing.c
│      │      BMX055_Processing.h
│      │      Mahony_AHRS.c
│      │      Mahony_AHRS.h
│      │
│      ├─DR16
│      │      dr16.cpp
│      │      dr16.h
│      │
│      ├─Flash
│      │  │  W25Qx.cpp
│      │  │  W25Qx.h
│      │  │
│      │  └─FATFS
│      │          diskio.c
│      │          diskio.h
│      │          ff.c
│      │          ff.h
│      │          ffconf.h
│      │
│      ├─HT_04
│      │      HT04.cpp
│      │      HT04.h
│      │
│      ├─Lpms_Be2
│      │      LPMS_BE2.cpp
│      │      LPMS_BE2.h
│      │
│      ├─MF9025_V2
│      │      MF9025_v2.cpp
│      │      MF9025_v2.h
│      │
│      ├─Motor_AK80
│      │      motor_AK80.cpp
│      │      motor_AK80.h
│      │
│      ├─Motor_Dji
│      │      motor_dji.h
│      │      motor_README.md
│      │
│      ├─Motor_DM
│      │      motor_dm.h
│      │
│      ├─MPU6050
│      │      dmpKey.h
│      │      dmpmap.h
│      │      inv_mpu.c
│      │      inv_mpu.h
│      │      inv_mpu_dmp_motion_driver.c
│      │      inv_mpu_dmp_motion_driver.h
│      │      mpu6050.c
│      │      mpu6050.h
│      │      mpu_component.c
│      │      mpu_component.h
│      │
│      ├─Referee
│      │      referee.cpp
│      │      referee.h
│      │      referee_README.md
│      │
│      └─VSEC
│              VSEC.cpp
│              VSEC.h
│
└─Middlewares
    ├─Abstract_class_library
    │      abstractIMU.cpp
    │      abstractIMU.h
    │      abstractMotor.cpp
    │      abstractMotor.h
    │      README.md
    │
    ├─Algorithm
    │  ├─DiffCalculator
    │  │      DiffCalculator.h
    │  │
    │  ├─filters
    │  │      filters.cpp
    │  │      filters.h
    │  │      SecondButterworthLPF.h
    │  │
    │  └─PID
    │          PID.cpp
    │          PID.h
    │
    ├─Matrix
    │      AxisAngle.h
    │      Dcm.h
    │      Dual.h
    │      Euler.h
    │      filter.h
    │      helper_functions.h
    │      integration.h
    │      LeastSquaresSolver.h
    │      Matrix.h
    │      PseudoInverse.h
    │      PX4_math.h
    │      Quaternion.h
    │      Readme.md
    │      Scalar.h
    │      Slice.h
    │      SparseVector.h
    │      SquareMatrix.h
    │      stdlib_imports.h
    │      Vector.h
    │      Vector2.h
    │      Vector3.h
    │
    ├─Module
    │  │  chassis.cpp
    │  │  chassis.h
    │  │  motor_ctrl.h
    │  │  power_ctrl.cpp
    │  │  power_ctrl.h
    │  │  power_ctrl_README.md
    │  │
    │  └─digital_power
    │          digital_Power.cpp
    │          digital_Power.h
    │          DP_power_ctrl.cpp
    │          DP_power_ctrl.h
    │          README.md
    │          数字电源功控库适配更新.md
    │
    ├─Protocol
    │      serial_line_ip.cpp
    │      serial_line_ip.h
    │
    └─Utility
            asuwave.c
            asuwave.h
            linux_list.h
            my_assert.h
            openlog.h
            sys_analysis.cpp
            sys_analysis.h
            sys_log.cpp
            sys_log.h
```



### 增加：

- `drv_memory.cpp`：基于freertos实现动态内存管理，
- `Middlewares/Algorithm/filters/SecondButterworthLPF.h`: 二阶巴特沃斯滤波器
- `Drivers/Devices/HT_04`：HT04电机库
- `Drivers/Devices/MF9025_V2`：9025电机库
- `Middlewares/Algorithm/DiffCalculator`：微分计算器
- `Middlewares/Algorithm/gimbal_motor_controller`：云台前馈全补偿控制器
- `Drivers/Devices/Lpms_Be2`：阿路比陀螺仪驱动库
- `Middlewares/abstract_class_library`：抽象类库，用于对各种模块的抽象封装
- `Middlewares/Module/digital_power`：数字电源库
- `Drivers/Components/drv_memory.h`：添加drv_memory模块，基于FreeRTOS重载new和delete，实现动态内存分配
- `Middlewares/Matrix`：添加PX4矩阵运算库
- `Drivers/Devices/Motor_DM`：添加达妙电机库
- 增加对于STM32H723的适配

### 修改：

- `PID.cpp`: pid类增加清除积分、微分反馈函数，解决ki为负值时的bug; 将PID_Timer改为SRML_Timer，在pid类内使用微分计算器;
- `PID.h`：pid类增加清除积分、微分反馈函数，解决ki为负值时的bug
- `referee.cpp`: 补全画ui函数
- `referee.h`: 补全画ui函数





## [2.3] - 2022-04-09

## 增加：

- `drv_VirtualCom.c`：增加虚拟串口模块，需要配合CUBE MX生成的库文件使用
- `drv_VirtualCom.h`：相应模块的头文件


## [2.3] - 2021-10-13

### 增加：
- `openlog.h`：增加openlog模块

### 更改：
- 更新mpu6050陀螺仪库
- `scut_usl.cpp/h`：重命名为`srml_std_lib.cpp/h`
- `srml_std_lib.cpp/h`：命名空间改为std_lib，并修改各文件中的包含和使用
- 部分计算绝对值函数由`fabs`改为`std::abs`
### 修复：
- `shitmountain.cpp/h`：增加include `hal.h`,解决uint16_t Undefine



## [2.2] - 2021-04-25
### 更改：
- `dr16.h`：`LinkageStatus_Typedef`缩短名称长度
### 修复：
- 所有计算绝对值函数由`fabs`改为`std::abs`
- `PID.cpp`：`myPID::Adjust`函数中使用`epsilon`比较浮点数
- `motor_dji.h`：`MotorMsgPack`函数使用引用传参
- `motor_ctrl.h`：`MotorCascadeCtrl::Adjust`函数添加`virtual`和`override`

## [2.1] - 2021-04-22
### 新增：
- `drv_can.h`：添加`CAN_COB`结构体定义
- `drv_uart.h`：添加`USART_COB`结构体定义
- `motor_dji.h`：添加`MotorMsgPack`函数
- `motor_ctrl.h`：电机控制库
### 更改：
- `motor.h`：重命名为`motor_dji.h`

## [2.0] - 2021-04-12
### 新增：
- 所有c/cpp文件添加包含`SRML.h`，并启用条件编译
- `srml_config_template.h`：配置模板
- `shit_mountain.cpp/h`：常用函数统一存放点
- `asuwave.c/h`：网页版上位机
### 更改：
- 所有自定义计算绝对值函数改为`std::abs`
- 所有`_constrain`改为`shit::constrain`
- `drv_i2c.h`：`IIC_PIN_Typedef`结构体内引脚类型由`uint32_t`改成`uint16_t`
- `BMX055_Config.c`：重命名为`BMX055_Config.cpp`
- `BMX055_Config.cpp`：`BMX055_Init`函数中分支山用`shit::get_gpio_pin_num`函数代替
- `mpu6050_config.cpp`：`MPU6050_Init`函数中分支山用`shit::get_gpio_pin_num`函数代替
- `dr16.h`：键位定义由宏定义改为枚举；`SW_Status_Typedef`、`LinkageStatus_Typedef`添加`DR16_`前缀
- `SerialLineIP.cpp`：重命名为`serial_line_ip.cpp`
- `SerialLineIP.h`：重命名为`serial_line_ip.h`
- `myAssert.h`：重命名为`my_assert.h`
### 修复：
- `drv_flash.h`：`get_sector`函数不被导出
- `drv_i2c.c`：`IIC_Delay`函数添加`volatile`，避免被优化
- `drv_timer.c`：`Timer_Init`函数中删去`HAL_TIM_Base_Start`，避免重复无用启动
- `VSEC.cpp`：`VSEC_UnPack`函数中修复变量类型不匹配问题
- `chassis.cpp`：`CChassis::position_control`函数中`memset`改为赋空值；`_Chassis_BubbleSort`改为`std::sort`
### 删除：
- `Drivers.h`
- `Kalman.cpp/h/md`
- `myMat.h`
- `old_PID.cpp/h`
- `Middlewares.h`
- `unified_frame.h`
- `control_analysis.cpp/h`
- `dev_manager.h`
- `linux_kfifo.c/h`

## [1.0] - 2021-03-23
正式发布
