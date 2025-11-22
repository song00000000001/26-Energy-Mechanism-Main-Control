# Mecenum底盘控制库

### 简介
麦克纳姆轮底盘运动控制库，具备以下功能：
1. 麦克纳姆底盘正逆解算
2. 防运动崎变
3. 起步刹车优化（力矩优化）
4. 防翻车策略（姿态控制模式）
5. 位置控制模式
其中前三项是基础功能，后两项是可选功能，也是有待进一步验证的功能

> 起步刹车优化的默认参数不是实验数据测出的（旧库直接给出），理论上应该实测出来才贴合实车，不过有效果也可用了
> 防翻车功能即姿态控制模式还只是个框架代码，实际功能有待验证和改写


### 食用方法

1. 定义底盘对象并初始化

   ```C++
     Chassis_ClassDef(float Max_Motor_Output = 16384,                  // 电机最大输出，默认c620最大输出
                      int16_t Wheel_Max_Speed = 4000,                  // 麦轮最大转速
                      bool torque_optimize_flag = 1,                   // 是否开启力矩优化，默认开
                      bool attitude_control_flag = 0,                  // 是否开启姿态控制，默认关
                      float Dt = 0.001f)                               // 步长，默认1ms
   ```

2. 进行相关参数设置（非必须操作）

   * 设置底盘模式（初始化对象时默认速度模式）

     ```
     enum class E_Chassis_Mode // 底盘模式
     {
       Speed = 0U, // 速度模式
       Position,   // 位置模式
       Halt        // 暂停模式
     };
     void Chassis_ClassDef::Switch_Mode(E_Chassis_Mode target_mode);
     ```

   * 设置速度挡位（初始化对象时默认高速挡位，）

     ```
     enum class E_Chassis_SpeedGears // 速度挡位
     {
       SLOW_GEAR = 0U,
       NORMAL_GEAR,
       FAST_GEAR
     };
     void Chassis_ClassDef::Set_SpeedGear(E_Chassis_SpeedGears targetGear);
     ```
     使用该语句会将底盘模式自动设为速度模式，常与失控保护语句配合使用

   * 其它参数设置（初始化对象时会设置一遍pid以及各种参数）

     ```
     void setAllPidParam();
     void setSpeedParam(float slow, float normal, float fast);
     void setAccelerationParam(int16_t launch, int16_t normal, uint16_t brake);
     void setTorqueOptimizeFlag(bool flag);
     void setAttitudeControlFlag(bool flag);
     ```
   * 对于力矩优化设置，默认开，用来防止过大的速度突变（启动和刹车时）
   * 对于姿态控制设置，默认关，若开启则用于防翻车（有待测试和验证） 

 3. 输入底盘目标

    ```
    /**
          设置目标值：
          1.速度模式 --> 设置三轴运动速度比例系数(X,Y,Rotate)：范围-1.0~1.0
          2.位置模式 --> 设置目标位姿（X,Y,Rotate）
        */
    void Chassis_ClassDef::setTarget(float target_X, float target_Y, float target_Z);
    ```

    目标值一般为dt7遥控下发的数据，数值范围为-1.0f ~ 1.0f

 4. 调用底盘总控制接口

    ```
    E_Chassis_Mode Chassis_ClassDef::Control();
    ```

 5. 发送电机数据（默认can1发送）

    ```
    void Chassis_ClassDef::sendMotorData();
    ```

    特别地，记得在电机can接收任务里更新电机数据，调用接口

    ```
    void Chassis_ClassDef::updateMotorData(CAN_COB CAN_RxMsg);
    ```

 6. 当想进行失控保护时，调用

    ```
    void Chassis_ClassDef::lostControlSave();
    ```

    该方法会将底盘状态设为停止，同时电调输出清零，重置所有pid参数。
    恢复底盘控制需重设一下底盘模式（如果是速度模式，则设置速度挡位即可）


### 使用示例
    ```
    if(board_com.Check_Link())	//在线状态
    {
      chassis_mode = board_com.getChassisMode();

      switch(chassis_mode){
        case E_Chassis_Status::Fast : {
          chassis.switchMode(E_Chassis_Mode::Speed); 
          chassis.setSpeedGear(E_Chassis_SpeedGears::FAST_GEAR);
          break;
        }
        case E_Chassis_Status::Normal : {
          chassis.switchMode(E_Chassis_Mode::Speed); 
          chassis.setSpeedGear(E_Chassis_SpeedGears::NORMAL_GEAR);
          break;
        }
        case E_Chassis_Status::Slow : {
          chassis.switchMode(E_Chassis_Mode::Speed); 
          chassis.setSpeedGear(E_Chassis_SpeedGears::SLOW_GEAR);
          break;
        }
        case E_Chassis_Status::Halt : {
          chassis.switchMode(E_Chassis_Mode::Halt); 
          break;
        }
        case E_Chassis_Status::Position : {
          chassis.switchMode(E_Chassis_Mode::Position); 
          break;
        }
        default:
          break;
      }

      board_com.Get_XYZ_Target(&speed_x,&speed_y,&speed_z);
      
      chassis.setTarget(speed_x,speed_y,speed_z);
      chassis.Control();
      chassis.sendMotorData();
    }
    else						//离线状态
    {
      chassis.lostControlSave();
    }
    ```

