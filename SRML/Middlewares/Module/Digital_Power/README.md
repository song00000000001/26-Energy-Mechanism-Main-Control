# 数字电源类用法
## 注意事项！！！
1. **数字电源执行任务优先级需要设置为PriorityRealtime**  
2. **数字电源任务延时请用vTaskDelay(1)，不要改任务周期，也不要用vTaskDelayUntil**  
3. **数字电源库文件内部已经实例化了类变量，并且在头文件extern，请勿在别的地方实例化数字电源变量，仅使用库自带的！！！**  

## 代码示例
```
//初始化代码，请勿在类的构造函数内调用，在主程序开始运行后再调用
digital_Power.digital_Power_Init();

  for (;;)
  {
    /* wait for next circle */
    vTaskDelay(1);//不要改任务周期，也不要用vTaskDelayUntil
    // 传入1、底盘功率限制，2、当前缓冲能量
    digital_Power.Update(Referee.GameRobotState.classis_power_limit, Referee.PowerHeatData.chassis_power_buffer);
    // 传入当前血量，防止死亡后电容供电
    digital_Power.digital_Power_Control(Referee.GameRobotState.remain_HP);
  }
```
在主程序开始运行后再调用一次``digital_Power_Init()``，**请勿在全局变量的构造函数内使用！！！**  
循环调用``Update()``与``digital_Power_Control()``  
在``Update()``函数传入1、底盘功率限制，2、当前缓冲能量  
然后调用``digital_Power_Control(Referee.GameRobotState.remain_HP)``函数，传入当前血量即可
## 数据读取
```
digital_Power.unit_DPW_data.Vcap;//电容电压，单位为V
digital_Power.unit_DPW_data.Vbat;//电池电压，单位为V
digital_Power.unit_DPW_data.Vout;//输出电压，单位为V
digital_Power.unit_DPW_data.I_in;//电池电流，单位为A，正为充电，负为放电
digital_Power.unit_DPW_data.I_chg;//电容充电电流，单位为A，正为充电，负为放电
digital_Power.unit_DPW_data.I_out;//输出电流，单位为A

digital_Power.power.pow_In;//电池输入功率，单位为w
digital_Power.power.pow_Charge;//电容充电功率，正为充电，负为放电，单位为w
digital_Power.power.pow_motor;//电机设备使用功率功率，单位为w
```