# SRML底盘库

---

这是一个旨在优化麦轮底盘运动，简化底盘控制的库。

## 底盘库内容

### 定义了几种不同的地盘类型：麦轮、月球车、舵轮

以下讨论范围基于麦轮

### 定义枚举量

### 定义四种底盘模式_chassis_Mode: 速度模式 Normal_Speed 常规位置模式 Normal_Position  跟随位置模式 Follow_Position  停止模式 Halt

### 三种速度挡位_chassis_SpeedGears： SLOW_GEAR

NORMAL_GEAR
FAST_GEAR

### 声明结构体类型

### 全局位姿 _chassis_GlobalPos

包含x、y、roll、pitch、yaw以及重载运算符定义同类型结构体运算关系

### 速度矢量 _chassis_Velocity

包含x、y、z方向的速度以及重载运算符定义同类型结构体运算关系

### 属性 _chasis_Parameter

力矩优化标志位                  torqueOptimize_flag

姿态优化标志位                  attitudeOptimize_flag

启动模式、正常模式、刹车模式的最大加速度  max launch / normal / brake  acceleratio

舵相关速度系数                  coefficient---

## 声明底盘类

### 私有成员 底盘模式、属性、当前速度挡位 麦轮运动解算 model_resolve()

### 位置控制 position_control()

***外部加载的用户自定义位置控制函数***

跟随位置模式下，相对位姿置零，相当于增量式控制位置。

### 速度控制 speed_control()

运动解算和限制轮速，如果转速最大的轮子的转速超过限定值，就将所有轮子转速进行等比例缩小，使底盘在整体降速的同时又不引起运动歧变。处理后的转速***（当前wheel_rpm[], 目标wheel_rpmOut[] )*** 再通过外部加载的用户自定义速度控制函数算出电机输出 ***( 存储在wheel_Out[] 中 )***

### 力矩优化 torque_optimization(） 用于启动和刹车时优化力矩，防止因加速度过大而轮子打滑而延长加速或制动距离 通过比较指令速度矢量和上一目标速度矢量，限制目标速度的变化量不大于预设的值，实现力矩优化。这里目标速度的变化量等于 ***步长***（task_run_interval，即离散系统的dt)与力矩优化后的加速度的乘积。

### 姿态控制 attitude_control()

用于避免底盘翻车

通过限制底盘yaw轴不变（不应该是限制roll和pitch吗）来防翻车

### 公有成员

### 构造函数

初始化底盘对象，前三个参数与麦轮无关设为零，第四设置电机最大输出，第五轮子最大转速，第六是否开启力矩优化（默认开启），默认不用姿态控制，第七设置计算步长（力矩优化算加速度用）默认0.001f，第八与麦轮无关（设置舵运动范围）

```
CChassis( float chassis_wheel_track, float chassis_wheel_base, float wheel_radius,\
              float max_motor_output, int16_t wheel_max_speed,\
              bool optimize_flag = 1, float run_interval = 0.001f, float str_range = 2*PI)_range = 2*PI

```

### ~~一些绕来绕去的变量~~

```
_chassis_GlobalPos   Zero_Pos;                 //!<零位姿点
_chassis_GlobalPos   Command_Pos;              //!<指令位姿
_chassis_Velocity    Command_Velocity;         //!<指令速度矢量
_chassis_Velocity    Target_Velocity;          //!<优化后输出目标速度矢量
int16_t              wheel_rpmOut[WHEEL_NUM];  //!<轮子解算后转速
_chassis_GlobalPos   Current_Pos;              //!<当前位姿
int16_t              wheel_rpm[WHEEL_NUM];     //!<轮子转速
int16_t              wheel_torque[WHEEL_NUM];  //!<轮子转矩（电流）
int32_t              wheel_Out[WHEEL_NUM];     //!<控制器作用后输出

```

### 回调函数装载接口

这一步没啥好说的，回调函数基本原理。

用户只需外部定义相应的控制函数，然后调用相应的load把函数名（指针）传进去，注意，*这里只能传c语言的函数，不能传类的成员函数*。

```
 void Load_AttitudeController(_chassis_Velocity*(*pFunc)(const _chassis_GlobalPos*, const _chassis_GlobalPos*));
 void Load_PositionController(_chassis_Velocity*(*pFunc)(const _chassis_GlobalPos*, const _chassis_GlobalPos*));
 void Load_SpeedController(int32_t*(*pFunc)(const int16_t*, const int16_t*));

```

### 设置目标值

Set_Target(float target_X, float target_Y ,float target_Z)

速度模式下，不同挡位乘以不同系数，位置模式下，指令位姿的yaw = target_z

### 其他

其他设置各种参数的成员函数、更新当前参数的成员函数就不一一解析了

## 底盘库的使用

1. 定义底盘对象，初始化；
2. 定义速度控制器、位置控制器、姿态控制器，调用底盘对象的装载接口；
3. 在can接收任务中，自定义函数解包电机数据，并传给底盘对象，以更新当前值
4. 调用底盘对象的set_target()，将目标值传给底盘对象；
