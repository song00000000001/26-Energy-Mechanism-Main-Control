<!-- 
  - Copyright (c) 2019 - ~, SCUT-RobotLab Development Team

  - @author  ZengXilang chenmoshaoalen@126.com

  - @brief   SRML底盘库麦轮部分文档

  - @date    2022-11-17
 -->

 @brief SRML底盘库

@author 曾熙朗

# 0. 前言

## 0.1 在学习底盘库之前建议先学习麦轮底盘解算
参考学习链接：

1. [麦克纳姆轮及其速度分解计算](https://blog.csdn.net/banzhuan133/article/details/69229922)

2. [麦克纳姆轮运动学解算](https://blog.csdn.net/oXiaoLingTong/article/details/120198677)

## 0.2 为什么写这篇文档
我在学习底盘库的过程中遇到了较大的阻力，库中代码的编写个人感觉有点混乱，代码书写格式给阅读带来一定困难，同时在一些关键变量和计算上缺乏注释，让人不知所云，故写这样一篇文档，一是注释记录，二是为了便于他人理解底盘库。

# 1. 简单的麦轮底盘PID速度环
我们从最简单的没有任何优化的速度环讲起。

在没有优化，单纯速度环控制的情况下，我们只需要设置 3个方向的速度Target值，经过麦轮解算得到 4个电机的目标速度，再经过电机本身的PID计算，即可得到输出。
<br>

在库中是这样体现的：
```
wheel_rpmOut[LF] =   Target_Velocity.x_speed + Target_Velocity.y_speed - Target_Velocity.z_speed;
wheel_rpmOut[RF] = - Target_Velocity.x_speed + Target_Velocity.y_speed + Target_Velocity.z_speed;
wheel_rpmOut[RB] =   Target_Velocity.x_speed + Target_Velocity.y_speed + Target_Velocity.z_speed;
wheel_rpmOut[LB] = - Target_Velocity.x_speed + Target_Velocity.y_speed - Target_Velocity.z_speed;
```
其中，数组`wheel_rpmOut`是经过解算后得到的4个电机的目标转速（注意不是电流输出），`LF RF RB LB`是四个电机的名字，`Target_Velocity`是底盘3个方向的移动速度目标值，库中把3个方向封装成一个向量。`x_speed`是底盘x正方向的速度，`y_speed`是底盘y正方向的速度，`z_speed`则是底盘转速。

在目标转速计算好后，再把这个目标值传给PID计算，计算出输出传给电调驱动电机。

# 2. 加入力矩优化的麦轮底盘速度环
所谓的力矩优化，就是对麦轮的加速度做一个限制。如果麦轮的转速变化过快，容易导致打滑，从而导致启动或刹车的过程延长（类比汽车的防抱死系统）。

为了实现这样一个优化，我们需要把目标值继续拆分，拆分为`Command_Velocity`指令目标速度向量，和`Target_Velocity`目标速度向量，两者是同类型的变量。前者即遥控器发来的目标速度，后者即做完力矩优化后，轮子的目标速度。

优化的过程就是通过`Command_Velocity`计算出`Target_Velocity`的过程。

优化过程分为3种情况：
- 第一种情况是从速度0启动或者从很低速到高速的过程，在这个过程中，加速度不能够太大。
- 第二种情况是一般过程，已经有一定的速度继续加速的过程，这个过程加速度可以大一些。
- 第三种过程是减速。

以上三种过程会分别对应（定义）不同的加速度系数。

这些加速度系数在库中分别为：
- max_launch_acceleration：启动加速度
- max_normal_acceleration：一般加速度
- max_brake_acceleration：刹车加速度

通过加速度系数乘以时间间隔`task_run_interval`，就可以得到一个速度变化量`dspeed`，再与上次的目标速度相加，就得到下一个时间点的`Target_Velocity`。

通过多次这样的累加，最后`Target_Velocity`就会达到`Command_Velocity`。

库中代码实现是这样的（`torque_optimization()`函数）：
>用了一个中间变量`Last_Target`，当计算完成后再把`Last_Target`赋值给`Target_Velocity`，第一、第二种情况的区分使用`launch_speed`“启动速度” 进行判断（这里我改了下缩进）

```
//x方向
if(std::abs(Command_Velocity.x_speed) > std::abs(Last_Target.x_speed))
{ 
  //accelerate加速
  if(std::abs(Command_Velocity.x_speed) <= Param.launch_speed)
    max_accceleration = Param.max_launch_acceleration;
  else
    max_accceleration = Param.max_normal_acceleration;
}
else
  //decelerate减速
  max_accceleration = Param.max_brake_acceleration;

//速度变化量
dspeed = max_accceleration*((float)Param.task_run_interval);
      
if(std::abs(Command_Velocity.x_speed - Last_Target.x_speed) >= dspeed)
{
  if(Command_Velocity.x_speed >= Last_Target.x_speed)
    Last_Target.x_speed += dspeed;
  else Last_Target.x_speed -= dspeed;
}
else
  Last_Target.x_speed = Command_Velocity.x_speed;

//y方向代码类似
...

//z方向（旋转）不做优化
Last_Target.z_speed = Command_Velocity.z_speed;

Target_Velocity = Last_Target;
```

当然，库中初始化时直接在构造函数中把`launch_speed`启动速度设置成了轮子的最大转速`wheel_max_speed`，这是我不太理解的地方。这样写相当于加速度只有一个，即`max_launch_acceleration`，（此处假设遥控器作了保护，其发送的指令速度不会超过`wheel_max_speed`的话）。

# 3. 速度环的基础上再加速度控制
此处的速度控制，则是对整个底盘4个电机速度的等比例变换。由于带负载后，电机的转速存在一个上限值，那么当计算出来的目标转速超过某个电机的最高转速，那么电机就没办法达到这个转速，从而使底盘运动产生变形。

速度控制的原理非常简单，只需要先找到目标转速最高的电机，然后计算出一个比例系数`scale`，即 `轮子的最大转速/目标转速最高的电机的目标转速`，然后每个轮子的目标转速都乘上这个系数即可。这样会产生的效果就是，当某个电机的转速超过最大值时，该电机的目标速度会设置为最大转速，并且会导致其他电机的转速减慢，从而整个底盘的运动也减慢，在控制的时候需要万分注意。

库中的代码实现（`speed_control()`函数）：
```
//把底盘目标转速的绝对值放入一个数组中
for (uint8_t k = 0; k < WHEEL_NUM; ++k )
  TempBuff[k] = std::abs(wheel_rpmOut[k]);

//对底盘的目标速度进行一个排序       
std::sort(TempBuff, TempBuff + WHEEL_NUM);
  if(TempBuff[WHEEL_NUM - 1] >= Param.wheel_max_speed)
    //计算这个比例系数
    scale = ((float)Param.wheel_max_speed)/((float)TempBuff[WHEEL_NUM - 1]);
  else
    scale = 1.0f;

for (uint8_t k = 0; k < WHEEL_NUM; ++k )
  wheel_rpmOut[k] = (int16_t)((float)wheel_rpmOut[k] * scale);
  
  /*
    Calculate target output for motors by external controller.
    速度环PID计算
  */
memcpy(wheel_Out, speed_controller(wheel_rpm, wheel_rpmOut), sizeof(int32_t)*WHEEL_NUM);
  
  /*
    Constrain for wheel output to protect the actuator.
    输出限幅
  */
for (uint8_t k = 0; k < WHEEL_NUM; ++k )
  wheel_Out[k] = std_lib::constrain(wheel_Out[k], (int32_t)-Param.MAX_MOTOR_OUTPUT, (int32_t)Param.MAX_MOTOR_OUTPUT);
```

# 4. 位置控制
位置控制在库中写得非常容易理解，不过控制器要自己写，无非是各种状况的PID（或其他算法）的计算，此处就不展开了。

# 5. 姿态保护
姿态保护（`attitude_control()`函数）是为了防止旋转（原地小陀螺）的过程中，轮子不当的速度变化（比如突然降为0）容易造成翻车。在旋转的过程中，通过将当前IMU Yaw轴角度值 设为目标值，将下次的 IMU Yaw轴角度值 设为当前值，进行（PID）计算，从而给`Target_Velocity`一个反向的补偿，对速度的变化进行一个约束。当然，姿态保护的控制器也是需要自己写的，还有待研究。

# 6. 库的使用
详见 *chassis.cpp* 开头的注释 *How to use this Lib*。

# 7. 一些变量/函数的注释
在`_chassis_WheelType`轮子的枚举中，麦轮用到的是如下这个：
```
#define WHEEL_NUM  4
enum _chassis_WheelType
{
  LF = 0U,
  RF,
  RB,
  LB
};
```

轮子不同的速度模式，选择不同的模式会调用不同的速度系数：
```
enum _chassis_SpeedGears
{
  SLOW_GEAR = 0U,
  NORMAL_GEAR,
  FAST_GEAR
};
```

底盘控制的参数（注释在代码里面）：
```
typedef struct 
{
  //力矩优化使能标志位
  bool  torqueOptimize_flag;
  //姿态保护使能标志位
  bool  attitudeOptimize_flag;
  //轮子的最大转速
  int16_t   wheel_max_speed;
  
  /* Time interval for speed resolve. */
  //也相当于是freertos的任务运行周期，库里设置成了1ms
  float  task_run_interval;
  
  /* Acceleration for different stages(Unit: rpm/s) */
  int16_t   max_launch_acceleration;
  int16_t   max_normal_acceleration;
  uint16_t   max_brake_acceleration;
  //启动速度（上限？库里面直接把最大转速赋给了它让我很不理解）
  int16_t   launch_speed;
  
  /* Coefficients for different gears(Max_gear_speed = Coefficients*wheel_max_speed).(Between 0 and 1) */
  //速度系数，在调用Set_Target(float target_X, float target_Y ,float target_Z)函数时会把遥控器传来的目标速度乘上这些系数再赋值给Command_Velocity
  float  coefficient_slow;
  float  coefficient_normal;
  float  coefficient_fast;
  float  coefficient_z;
	
	/* User Specific Parameters */
#if defined(SWERVE_CHASSIS)
  float STEER_RANGE;
#endif
  //下面这3个我是没看懂
	float CHASSIS_WHEEL_TRACK;
	float CHASSIS_WHEEL_BASE;
	float WHEEL_RADIUS;
  //电机最大输出
	int16_t MAX_MOTOR_OUTPUT;
	float RUN_INTERVAL;	
}_chassis_Parameter;
```

剩余的函数、变量通过函数名、变量名都很容易理解，此处不再赘述。

# 参考资料
1. 《SRML底盘库理解》 吴宇栋 2021

2. [麦克纳姆轮及其速度分解计算](https://blog.csdn.net/banzhuan133/article/details/69229922) 一把木剑 2017

3. [麦克纳姆轮运动学解算](https://blog.csdn.net/oXiaoLingTong/article/details/120198677) 范子琦 2021

4. SRML《chassis.cpp》《chassis.h》 Mentos Seetoo, Kainan.Su 2019



<!-- /************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/ -->