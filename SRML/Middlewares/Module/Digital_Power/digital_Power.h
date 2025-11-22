/**
******************************************************************************
* Copyright (c) 2022 - ~, SCUT-RobotLab Development Team
* @file    digital_Power.h
* @author  林茂辉 15766420234
* @brief   Header file of SourceManage.
******************************************************************************
* @attention
*
* if you had modified this file, please make sure your code does not have many
* bugs, update the version Number, write dowm your name and the date, the most
* important is make sure the users will have clear and definite understanding
* through your new brief.
*
* <h2><center>&copy; Copyright (c) 2022 - ~, SCUT-RobotLab Development Team.
* All rights reserved.</center></h2>
******************************************************************************
*/

#ifndef _DIGITAL_POWER_H
#define _DIGITAL_POWER_H

/*Include------------------------------------------------------------------------*/
#ifdef STM32F405xx
#include "math.h"
#include "stm32f4xx_hal.h"
#include "Middlewares/Algorithm/filters/filters.h"

#ifdef __cplusplus

#define Shutdown1_Pin GPIO_PIN_8
#define Shutdown1_Port GPIOC
#define Shutdown2_Pin GPIO_PIN_9
#define Shutdown2_Port GPIOC

#define Buck_Pin GPIO_PIN_14
#define Boost_Pin GPIO_PIN_15

#define LED_Port GPIOC
#define YELLOW_LED_PIN GPIO_PIN_13
#define GREEN_LED_PIN GPIO_PIN_14
#define BLUE_LED_PIN GPIO_PIN_15

#define CHG_VOLTAGE_MAX 29000 // 最大电压，单位为mV
#define CHG_VOLTAGE_MIN 5000 // 最小电压，单位为mV
#define CHG_CURRENT_MAX 8000  // 最大充电电流，单位为mA
#define OUT_CURRENT_MAX 12000 // 最大放电电流，单位为mA
#define LOWCURRENT_VOL 22000  // 开启小电流充电的电压，单位为mV
#define LOWCURRENT 4500       // 小电流的值，单位为mAs

#define CHG_PWM_RLOAD 2000                      // 168000000/2 / 2000 = 42k
#define CHG_PWM_MAX (CHG_PWM_RLOAD * (1 + 0.6)) // boost最大占空比36/24=1.5

#define PWM_Timer htim1
#define Buck_Channel TIM_CHANNEL_2
#define Boost_Channel TIM_CHANNEL_3

// ADC通道排次(Regular)
#define ADC_Channel_Numbers_Regular 8
// 以下顺序为ADC规则通道配置顺序(rank)，对应原始数据数组
#define ADC_Io_Bias 0   // Rank1     Channel 1   高侧电流输出偏置PA1   Iout_Bias
#define ADC_Ichg_Bias 1 // Rank2     Channel 5   PA5                   Icap_IN_Bias
#define ADC_Icap_Bias 2 // Rank3     Channel 14  PC4                   Icap_OUT_Bias
#define ADC_Icap_L 3    // Rank4     Channel 15  PC5                   Icap_OUT_LOW
#define ADC_Iin_Bias 4  // Rank5     Channel 12  PC2                   Iin_Bias
#define ADC_Vcap 5      // Rank6    Channel 8   PB0                   ADC_CAP
#define ADC_Vin 6       // Rank7    Channel 10  PC0                   ADC_VIN
#define ADC_Vo 7        // Rank8    Channel 13  PC3                   ADC_VOUT

// ADC通道排次(Injected)
#define ADC_Channel_Numbers_Injected 4
// 以下顺序为ADC注入通道配置顺序(rank)，对应原始数据数组
#define ADC_Io_H_Injected 0   // Rank1     Channel 0   高侧电流输出PA0   Iout_HIGH
#define ADC_Ichg_H_Injected 1 // Rank2     Channel 4   PA4              Icap_IN_HIGH
#define ADC_Iin_H_Injected 2  // Rank3     Channel 11  PC1              Iin_HIGH
#define ADC_Icap_H_Injected 3 // Rank4     Channel 7   PA7              Icap_OUT_HIGH

// 分压倍率
#define Vin_Ratio 11
#define Vcap_Ratio 11
#define Vout_Ratio 11
#define Io_Ratio 22.72f
#define Ichg_H_Ratio 22.72f
#define Icap_H_Ratio 22.72f
#define Iin_Ratio 15.15f
// 霍尔电流传感器为100mV/A，输出偏置2.5，因此需要先测量偏置值
#define Icap_L_Ratio 20   // 采样电阻2mR，放大倍数25
#define Ichg_L_Ratio 7.5f // 采样电阻2mR，放大倍数40

namespace digital_Power_namespace
{
  const uint16_t mos_switch_internal = 200;

  enum Cap_State_TypeDef
  {
    capCharge = 0,
    capDischarge = 1
  };

  enum Switch_TypeDef
  {
    buck = 0,
    boost = 1
  };

  enum Mos_State_TypeDef
  {
    mos_off,
    mos_on
  };

  struct DPW_TypeDef
  {
    Switch_TypeDef switching_status; // 开关状态 0-关闭 1-buck 2-boost 3-buck-boost
    uint16_t Vbat;                   // 输入电压
    uint16_t Vcap;                   // 电容电压
    uint16_t Vout;                   // 输出电压
    int32_t I_in_H;                  // 输入电流
    int32_t I_chg_H;                 // 充电电流
    int32_t I_cap_H;
    int32_t I_cap_L;
    int32_t I_out_H;
    int16_t out_duty; // 占空比
  } ;

  struct unit_DPW_TypeDef
  {
    const float Vcap_MAX = CHG_VOLTAGE_MAX / 1000.f;
    const float Vcap_MIN = CHG_VOLTAGE_MIN / 1000.f;
    float Vbat;  // 输入电压
    float Vcap;  // 电容电压
    float Vout;  // 输出电压
    float I_in;  // 输入电流
    float I_chg; // 充电电流
    float I_cap;
    float I_out;
  } ;

  struct power_t
  {
    float pow_In;
    float pow_motor;
    float pow_Charge;
  } ;

  template <typename T, typename T1>
  T constrain(T input, T1 a, T1 b)
  {
    T min, max;
    if (a < b)
    {
      min = a;
      max = b;
    }
    else
    {
      min = b;
      max = a;
    }

    if (input <= min)
      return min;
    else if (input >= max)
      return max;
    return input;
  }

  class pi_cal
  {
  public:
    float target = 0;
    float current = 0;
    float kp = 0, ki = 0;
    float Out = 0;

    float integral_max = 0;
    float integral_min = 0;
    float OutMax = 0;
    float OutMin = 0;
    void adjust()
    {
      err = target - current;
      integral += err;
      integral = constrain(integral, integral_min, integral_max);

      Out = kp * err + ki * integral;
      Out = constrain(Out, OutMin, OutMax);
    }
    void cleanIntegral() { integral = 0; }
    void setParam(float _kp, float _ki, float _I_term_max, float _out_max)
    {
      kp = _kp;
      ki = _ki;

      integral_max = abs(_I_term_max / _ki);
      integral_min = -integral_max;

      OutMax = abs(_out_max);
      OutMin = -OutMax;
    }
    void setOutMax(float _out_max)
    {
      OutMax = abs(_out_max);
      OutMin = -OutMax;
    }
    void setI_term(float _I_term)
    {
      integral = _I_term / ki;
    }

  private:
    float err = 0;
    float integral = 0;
  };
};

class digital_Power_Classdef
{
private:
  void adc_Calculate(void);
  void adc_injected_Calculate(ADC_HandleTypeDef *hadc);
  void switch_OFF(void);
  void switch_ON(void);

  void calc_Power(void);
  void set_TargetCurrent(void);
  void calc_Current(void);
  void set_Output();

  bool no_bat_check();
  bool overCurrent_check();
  void VcapOver_protect();
  void low_Vbat_check();

  bool Low_Bias_Ready;                                          // 采集偏置电压结束标志位
  int32_t ADC1_RegularConv_Buffer[ADC_Channel_Numbers_Regular]; // ADC1规则组原始数据
  int32_t ADC_RegularConv_mV[ADC_Channel_Numbers_Regular];

  int32_t ADC_InjectedConv_mV[ADC_Channel_Numbers_Injected];

  digital_Power_namespace::pi_cal power_buffer_PI;

  uint8_t Vcap_compare_state = 1, voltage_Loop_state = 0, low_Vbat_state = 0;
  uint16_t no_bat_cnt = 500;
  int16_t ADC_Icap_L_Bias = 0;

  friend void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc);

  MeanFilter<10> VcapMeanFilter;

  uint16_t RF_power_limit = 50;
  uint16_t mos_switch_cnt = digital_Power_namespace::mos_switch_internal;
public:
  digital_Power_Classdef();
  digital_Power_namespace::DPW_TypeDef DPW;
  digital_Power_namespace::unit_DPW_TypeDef unit_DPW_data; // 使用国际标准单位表示数据，电压为V，电流为A
  digital_Power_namespace::power_t power;                  // 单位为w
  digital_Power_namespace::pi_cal current_Loop, voltage_Loop;

  void digital_Power_Init(void); // 请勿在全局变量的构造函数内使用，请在main()函数开始运行后再使用
  void Update(uint16_t _RF_power_limit, uint8_t _power_buffer);
  void digital_Power_Control(uint16_t HP);
	float cal_adaptive_outDuty();

  int32_t target_charge_power = 0, target_charge_current = 0;
  digital_Power_namespace::Mos_State_TypeDef mos_state = digital_Power_namespace::mos_off;
};
extern digital_Power_Classdef digital_Power;
#endif //_cplusplus

#endif //STM32F405xx

#endif //_DIGITAL_POWER_H
