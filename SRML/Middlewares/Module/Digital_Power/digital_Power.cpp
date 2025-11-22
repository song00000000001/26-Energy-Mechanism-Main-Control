/**
******************************************************************************
* Copyright (c) 2022 - ~, SCUT-RobotLab Development Team
* @file    digital_Power.cpp
* @author  林茂辉 15766420234
* @author  余俊晖 13826150939 2460857175@qq.com
* @brief   数字电源
* @date    2023-3-299
* @version 2.0
* @par Change Log：
* <table>
* <tr><th>Date        	<th>Version  <th>Author    		<th>Description
* <tr><td>2022-10-25   	<td> 1.0     <td>林茂辉        <td>
* <tr><td>2023-3-29   	<td> 2.0     <td>余俊晖        <td>超功率机制改为自动超功率，将之前的pid计算改为用类，加入命名空间防止库内自定义类型污染外部，电容放电时不再使用电压环，增加防止电池关闭时电容供电的机制
* </table>
*
==============================================================================
					##### How to use this driver #####
==============================================================================
	@note
digital_Power.digital_Power_Init();//请勿在全局变量的构造函数内使用，请在main()函数开始运行后再使用
while(1)
{
	digital_Power.Update(int16_t _target_change_power, uint16_t _RF_power_limit, uint8_t _power_buffer);
	digital_Power.digital_Power_Control(uint16_t HP);
}

数据获取：
digital_Power_Classdef::unit_DPW_data 电压电流数据，均为国际标准单位
digital_Power_Classdef::power 功率数据，均为国际标准单位
	@warning

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

/* Includes ------------------------------------------------------------------*/
#include "srml_config.h"

#if defined(STM32F405xx) && USE_SRML_DIGITAL_POWER

#include "digital_Power.h"
#include "srml_std_lib.h"

#define fmax(a, b) (a > b) ? a : b
#define fmin(a, b) (a < b) ? a : b

#define CONTROL_1khz 0
#define CONTROL_42khz 1 // 经过测试42khz占用芯片算力22.8%
#define CONTROL_21khz 0 // 经过测试21khz占用芯片算力15%

#define DEBUG_MODE 0 // 1为调试模式，0为正常模式

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim1;

using namespace digital_Power_namespace;
digital_Power_Classdef digital_Power;

static void TIM_CCxNChannelCmd(TIM_TypeDef *TIMx, uint32_t Channel, uint32_t ChannelNState);

uint8_t overCurrent_cnt = 0;
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	digital_Power.adc_injected_Calculate(hadc);
	if (digital_Power.overCurrent_check() == true) // 过流检测保护
	{
		digital_Power.switch_OFF();
		overCurrent_cnt++;
	}
	else if (digital_Power.low_Vbat_state == true) // 低压保护
	{
		digital_Power.switch_OFF();
	}
#if CONTROL_42khz
	else if (digital_Power.mos_state == mos_on)
	{
		digital_Power.calc_Current();
		digital_Power.set_Output();
	}
#elif CONTROL_21khz
static bool en_current_loop = true;
	if (digital_Power.mos_state == mos_on)
	{
		if (en_current_loop == true)
		{
			digital_Power.calc_Current();
			digital_Power.set_Output();
		}
		en_current_loop = !en_current_loop; // 只有在开mos的时候才运行失能电流环，避免开mos时电流环处于失能状态
	}
	else
	{
		en_current_loop = true;
	}
#endif
}

digital_Power_Classdef::digital_Power_Classdef()
{
	voltage_Loop.setParam(2, 0, 1000, LOWCURRENT);
	voltage_Loop.target = CHG_VOLTAGE_MAX;
// 电容充电环PID，kp增加要慎重，增太大会引发充电电流大幅震荡，使mos管过热，引发事故
#if CONTROL_1khz
	current_Loop.setParam(0.01f, 0.005f, CHG_PWM_RLOAD, CHG_PWM_RLOAD); // 1khz参数
#elif CONTROL_42khz
	current_Loop.setParam(0.01f, 0.008f / 42.f, CHG_PWM_RLOAD, CHG_PWM_RLOAD); // 42khz参数
#elif CONTROL_21khz
	current_Loop.setParam(0.01f, 0.008f / 21.f, CHG_PWM_RLOAD, CHG_PWM_RLOAD);
#endif
	power_buffer_PI.setParam(2.f, 0, 20, 50);
}
/**
 * @brief 数字电源初始化
 * 禁止在类的构造函数内调用，必须在主程序开始运行后调用
 */
void digital_Power_Classdef::digital_Power_Init(void)
{
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC1_RegularConv_Buffer, ADC_Channel_Numbers_Regular);
	HAL_ADCEx_InjectedStart_IT(&hadc2);

	HAL_TIMEx_PWMN_Start(&PWM_Timer, Buck_Channel);
	HAL_TIMEx_PWMN_Start(&PWM_Timer, Boost_Channel);

	mos_state = mos_on;
	switch_OFF();

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
}

/**
 * @brief 传入数据
 * @note
 * @param _RF_power_limit    裁判系统底盘限制功率
 * @param _power_buffer   当前裁判系统缓冲能量
 * @retval None
 * @author lmh
 */
void digital_Power_Classdef::Update(uint16_t _RF_power_limit, uint8_t _power_buffer)
{
	RF_power_limit = _RF_power_limit;
	if (mos_switch_cnt > 0)
		this->mos_switch_cnt--;

	adc_Calculate();
	calc_Power();

	power_buffer_PI.target = 52;
	power_buffer_PI.current = std_lib::constrain(_power_buffer, (uint8_t)0, (uint8_t)60);
	power_buffer_PI.adjust();

	// 电压环触发滞回比较器
	if (DPW.Vcap > CHG_VOLTAGE_MAX - 1500)
	{
		voltage_Loop_state = 1;
	}
	else if (DPW.Vcap < CHG_VOLTAGE_MAX - 2000)
	{
		voltage_Loop_state = 0;
	}

	set_TargetCurrent();
}
/**
 * @brief 数字电源控制函数
 *
 * @param HP
 */
void digital_Power_Classdef::digital_Power_Control(uint16_t HP)
{
	low_Vbat_check();

	if (Low_Bias_Ready && HP != 0 && low_Vbat_state == false && no_bat_check() == false)
	{
		VcapOver_protect(); // 电容过压保护策略
		switch_ON();
#if CONTROL_1khz
	if (mos_state == mos_on)
	{
		calc_Current();
		set_Output();
	}
#endif
	}
	else
	{
		switch_OFF();
	}
}
/**
 * @brief 设定电容充电电流
 * @note
 * @param None
 * @retval None
 * @author lmh
 */
#if DEBUG_MODE
int16_t debug_target_charge_power = 0;
#endif

void digital_Power_Classdef::set_TargetCurrent(void)
{
	static uint8_t low_current_state = 0;
	target_charge_power = 0.95f * RF_power_limit - power.pow_motor - power_buffer_PI.Out;
#if DEBUG_MODE
	target_charge_power = debug_target_charge_power;
#endif

	if (DPW.Vcap > LOWCURRENT_VOL)
	{
		low_current_state = 1;
	}
	else if (DPW.Vcap < LOWCURRENT_VOL - 1000)
	{
		low_current_state = 0;
	}

	int32_t charge_current_min, charge_current_max = fmin(CHG_CURRENT_MAX, 25000 * (unit_DPW_data.Vcap / unit_DPW_data.Vbat));
	if (charge_current_max < 100)
		charge_current_max = 100;

	if (DPW.Vcap > 16000)
	{
		charge_current_min = -OUT_CURRENT_MAX;
	}
	else if (DPW.Vcap > 5000) // 16v ~ 5v之间放电电流逐渐减小
	{
		charge_current_min = -OUT_CURRENT_MAX * (unit_DPW_data.Vcap - unit_DPW_data.Vcap_MIN) * 0.090909f; // 最后的小数为1/(16-5)，因为乘法消耗算力更小
	}
	else
	{
		charge_current_min = 0;
	}

	target_charge_current = target_charge_power * 1000 * 1000 / DPW.Vbat;
	if (low_current_state == 1)
	{
		target_charge_current = std_lib::constrain(target_charge_current, charge_current_min, (int32_t)LOWCURRENT);
	}
	else
	{
		target_charge_current = std_lib::constrain(target_charge_current, charge_current_min, (int32_t)charge_current_max);
	}

	voltage_Loop.target = CHG_VOLTAGE_MAX;
	voltage_Loop.OutMax = fmax(target_charge_current, 0);
	voltage_Loop.OutMin = -1000;
}

/**
 * @brief PID电流环计算
 * @note
 * @param None
 * @retval None
 * @author lmh
 */
void digital_Power_Classdef::calc_Current(void)
{
	if (voltage_Loop_state == 1 && target_charge_current >= 0) // 电压接近29v且还在充电是使用电压环控制目标电流、
	{
		voltage_Loop.current = DPW.Vcap;
		voltage_Loop.adjust();
		current_Loop.target = voltage_Loop.Out;
	}
	else
	{
		current_Loop.target = target_charge_current;
	}

	current_Loop.current = DPW.I_chg_H;
	current_Loop.adjust();
	DPW.out_duty = current_Loop.Out + 2000;
}

/**
 * @brief 占空比下发函数
 *
 * @param duty
 */
void digital_Power_Classdef::set_Output()
{
	int16_t temp = cal_adaptive_outDuty();
	if(DPW.Vbat > 20000)
	{	// 根据稳态值进行一个限幅
		if (temp < 2300 && temp > 1600) // 因为out duty在1800-2200之间会存在跳变，因此这段的限幅要给得大一些
		{
			DPW.out_duty = std_lib::constrain(DPW.out_duty, (int16_t)1400, (int16_t)2500);
		}
		else
		{
			DPW.out_duty = std_lib::constrain(DPW.out_duty, (int16_t)(temp - 300), (int16_t)(temp + 400));
		}
	}

	uint16_t duty = DPW.out_duty;
	if ((duty > CHG_PWM_RLOAD - 200) && (duty < CHG_PWM_RLOAD + 200))
	{
		duty = CHG_PWM_RLOAD + 200;
	}
	/////////////////////////传统方法Buck-Boost//////////////////////////////////////

	if (duty < CHG_PWM_RLOAD) // Buck Mode
	{
		DPW.switching_status = buck;
		PWM_Timer.Instance->CCR2 = duty;				// buck占空比
		PWM_Timer.Instance->CCR3 = CHG_PWM_RLOAD - 200; // boost占空比，保持90%的占空比，相当于一直通
	}
	else // Boost Mode
	{
		DPW.switching_status = boost;
		duty = 2 * CHG_PWM_RLOAD - duty;
		PWM_Timer.Instance->CCR2 = CHG_PWM_RLOAD - 200; // buck占空比，保持90%的占空比，相当于一直通
		PWM_Timer.Instance->CCR3 = duty;				// boost占空比
	}
}

/**
 * @brief 电池关闭检测函数
 *
 * @return true 没有电池供电
 * @return false 有电池供电
 */
int16_t debug_I_in_H = 0;
bool digital_Power_Classdef::no_bat_check()
{
	static MeanFilter<100> I_in_H_filter;
	int16_t I_in_H = I_in_H_filter.f(DPW.I_in_H);
#if DEBUG_MODE
	debug_I_in_H = I_in_H;
#endif
	// 检测电池是否关闭
	if (no_bat_cnt == 0)
	{
		if (power.pow_In < 5)
		{
			return 1;
		}
		else
		{
			no_bat_cnt = 250;
		}
	}
	else
	{
		if (I_in_H < -25 && I_in_H > -180)
		{
			no_bat_cnt--;
		}
		else
		{
			no_bat_cnt = 250;
		}
	}
	return 0;
}
/**
 * @brief 电源低电压检测
 *
 */
void digital_Power_Classdef::low_Vbat_check()
{
	static uint8_t cnt = 10;
	if (DPW.Vbat < 15000)
	{
		if (cnt != 0)
		{
			cnt--;
		}
		else
		{
			low_Vbat_state = true;
		}
	}
	else
	{
		cnt = 10;
	}

	low_Vbat_state = false;
}
/**
 * @brief 过流判断
 *
 */
bool digital_Power_Classdef::overCurrent_check()
{
	if (DPW.I_in_H > 30000 || DPW.I_in_H < -12000 || DPW.I_cap_H > 28000 || DPW.I_cap_H < -28000 || DPW.I_chg_H < -18500 || DPW.I_chg_H > CHG_CURRENT_MAX + 2000)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/**
 * @brief 电容过压保护策略
 *
 */
void digital_Power_Classdef::VcapOver_protect()
{
	if (DPW.Vcap > 30000) // 滞回比较器
	{
		Vcap_compare_state = 0;
	}
	else if (DPW.Vcap < 28500)
	{
		Vcap_compare_state = 1;
	}

	if (Vcap_compare_state == 0 && target_charge_current > 0) // 电容电源过高，且还在充电，则放一部分电
	{
		target_charge_current = -500;
	}
}

/**
 * @brief 计算电池输入功率、输出功率、电容功率
 * @note
 * @param None
 * @retval None
 * @author lmh
 */
void digital_Power_Classdef::calc_Power(void)
{
	power.pow_In = unit_DPW_data.Vbat * unit_DPW_data.I_in;
	power.pow_motor = unit_DPW_data.Vout * unit_DPW_data.I_out;
	power.pow_Charge = unit_DPW_data.Vbat * unit_DPW_data.I_chg;
}

/**
 * @brief Open DC-DC
 * @note
 * @param
 * @retval None
 * @author lmh
 */
void digital_Power_Classdef::switch_ON(void)
{
	if (mos_state == mos_on											  // 为on模式则不用执行，为了不与set_Output内部的保护措施冲突
		|| (this->mos_state == mos_off && this->mos_switch_cnt != 0)) // 未满足延时，不允许切换
	{
		return;
	}
	TIM_CCxNChannelCmd(PWM_Timer.Instance, Buck_Channel, TIM_CCxN_ENABLE);
	TIM_CCxNChannelCmd(PWM_Timer.Instance, Boost_Channel, TIM_CCxN_ENABLE);

	Shutdown1_Port->BSRR = Shutdown1_Pin; // 开启buck
	Shutdown2_Port->BSRR = Shutdown2_Pin; // 开启boost

	LED_Port->BSRR = (uint32_t)YELLOW_LED_PIN << 16U; // 关闭黄灯
	LED_Port->BSRR = (uint32_t)GREEN_LED_PIN << 16U;  // 关闭绿灯
	LED_Port->BSRR = BLUE_LED_PIN;					  // 开启蓝灯

	this->mos_state = mos_on;
}

/**
 * @brief Stop DC-DC
 * @note
 * @param
 * @retval None
 * @author lmh
 */
void digital_Power_Classdef::switch_OFF(void)
{
	if (low_Vbat_state == false)
	{
		current_Loop.setI_term(cal_adaptive_outDuty() - 2000); // 把电流环积分设置为稳态时的值，防止重新开始计算时跑飞
	}

	this->mos_switch_cnt = mos_switch_internal; // 重新计数
	if (this->mos_state == mos_off)				// 若已为off模式，则不用执行
	{
		return;
	}

	Shutdown1_Port->BSRR = (uint32_t)Shutdown1_Pin << 16U; // 关闭buck
	Shutdown2_Port->BSRR = (uint32_t)Shutdown2_Pin << 16U; // 关闭boost

	LED_Port->BSRR = YELLOW_LED_PIN;				// 开启黄灯
	LED_Port->BSRR = GREEN_LED_PIN;					// 开启绿灯
	LED_Port->BSRR = (uint32_t)BLUE_LED_PIN << 16U; // 关闭蓝灯

	TIM_CCxNChannelCmd(PWM_Timer.Instance, Buck_Channel, TIM_CCxN_DISABLE);
	TIM_CCxNChannelCmd(PWM_Timer.Instance, Boost_Channel, TIM_CCxN_DISABLE);

	this->mos_state = mos_off;
}

/**
 * @brief 计算当前电容电压下的稳态占空比，精度为±50
 *
 * @return float
 */
float digital_Power_Classdef::cal_adaptive_outDuty()
{
	float ans;
	if (DPW.Vcap > DPW.Vbat)
	{
		ans = DPW.Vcap / (float)DPW.Vbat * 2000 + 200;
	}
	else
	{
		ans = DPW.Vcap / (float)DPW.Vbat * 2000 - 200;
	}
	ans -= 52 * (26 - unit_DPW_data.Vbat);//观察发现
	ans -= 0.14542089f * unit_DPW_data.Vcap * unit_DPW_data.Vcap 
			+ 4.2353949f * unit_DPW_data.Vcap 
			- 290.33440248f;
	ans = std_lib::constrain(ans, 50.f, 2800.f);
	return ans;
}

/**
 * @brief 把ADC值转换成电压电流真实值并滤波
 * @note
 * @param
 * @retval None
 * @author lmh
 */

void digital_Power_Classdef::adc_Calculate(void)
{
	static int8_t cnt = 0;
	static uint32_t bias_temp1;
	for (int i = 0; i < ADC_Channel_Numbers_Regular; i++)
	{
		ADC_RegularConv_mV[i] = ((int16_t)ADC1_RegularConv_Buffer[i] * 0.805664f); // 0.805664f = 3300 / 4096
	}

	if ((!Low_Bias_Ready) && (ADC_RegularConv_mV[ADC_Vin] > 1500) && cnt < 10) // 判断为上电而不是逻辑上电
	{
		bias_temp1 += ADC_RegularConv_mV[ADC_Icap_L];
		cnt++;
		if (cnt >= 10)
		{
			ADC_Icap_L_Bias = bias_temp1 * 0.1f;
			Low_Bias_Ready = 1;
		}
	}

	DPW.Vbat = ADC_RegularConv_mV[ADC_Vin] * Vin_Ratio;
	DPW.Vcap = VcapMeanFilter.f(ADC_RegularConv_mV[ADC_Vcap]) * Vcap_Ratio;
	DPW.Vout = ADC_RegularConv_mV[ADC_Vo] * Vout_Ratio;
	DPW.I_cap_L = (ADC_RegularConv_mV[ADC_Icap_L] - ADC_Icap_L_Bias) * Icap_L_Ratio;

	unit_DPW_data.Vbat = DPW.Vbat * 0.001f;
	unit_DPW_data.Vcap = DPW.Vcap * 0.001f;
	unit_DPW_data.Vout = DPW.Vout * 0.001f;

	unit_DPW_data.I_in = DPW.I_in_H * 0.001f;
	unit_DPW_data.I_out = DPW.I_out_H * 0.001f;
	unit_DPW_data.I_cap = DPW.I_cap_H * 0.001f;
	unit_DPW_data.I_chg = DPW.I_chg_H * 0.001f;
}
/**
 * @brief adc注入中断的数据处理
 *
 * @param hadc
 */
void digital_Power_Classdef::adc_injected_Calculate(ADC_HandleTypeDef *hadc)
{
	ADC_InjectedConv_mV[0] = hadc->Instance->JDR1 * 0.805664f; // 0.805664f = 3300 / 4096
	ADC_InjectedConv_mV[1] = hadc->Instance->JDR2 * 0.805664f;
	ADC_InjectedConv_mV[2] = hadc->Instance->JDR3 * 0.805664f;
	ADC_InjectedConv_mV[3] = hadc->Instance->JDR4 * 0.805664f;

	DPW.I_out_H = (ADC_InjectedConv_mV[ADC_Io_H_Injected] - ADC_RegularConv_mV[ADC_Io_Bias]) * Io_Ratio;
	DPW.I_chg_H = (ADC_InjectedConv_mV[ADC_Ichg_H_Injected] - ADC_RegularConv_mV[ADC_Ichg_Bias]) * Ichg_H_Ratio;
	DPW.I_cap_H = (ADC_InjectedConv_mV[ADC_Icap_H_Injected] - ADC_RegularConv_mV[ADC_Icap_Bias]) * Icap_H_Ratio;
	DPW.I_in_H = (ADC_InjectedConv_mV[ADC_Iin_H_Injected] - ADC_RegularConv_mV[ADC_Iin_Bias]) * Iin_Ratio;
}

/**
 * @brief 定时器互补通道使能函数（stm32库内的static函数，因此只能复制出来）
 *
 * @param TIMx
 * @param Channel
 * @param ChannelNState
 */
static void TIM_CCxNChannelCmd(TIM_TypeDef *TIMx, uint32_t Channel, uint32_t ChannelNState)
{
	uint32_t tmp;

	tmp = TIM_CCER_CC1NE << (Channel & 0x1FU); /* 0x1FU = 31 bits max shift */

	/* Reset the CCxNE Bit */
	TIMx->CCER &= ~tmp;

	/* Set or reset the CCxNE Bit */
	TIMx->CCER |= (uint32_t)(ChannelNState << (Channel & 0x1FU)); /* 0x1FU = 31 bits max shift */
}

#endif /* defined(STM32F405xx) && USE_SRML_DIGITAL_POWER */
