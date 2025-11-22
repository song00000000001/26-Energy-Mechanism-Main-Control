/**
  ******************************************************************************
  * Copyright (c) 2023 - ~, SCUT-RobotLab Development Team
  * @file    DiffCalculator.h
  * @author  余俊晖 (2460857175@qq.com)
  * @brief   基于SRML_Timer的微分计算器，模板参数输入滤波器类名，会给微分结果自动滤波后返回
  * @date    2023-06-01
  * @version 1.0
  *
  ==============================================================================
					        How to use this library 
  ==============================================================================
    @note
			- DiffCalculator::calc(float input)，通过SRML_Timer计算时间差dt进行微分

            - DiffCalculator::calc(float input, float _dt)，使用参数dt进行微分
  	@warning 
			- 构造函数输入参数必须符合滤波器的构造函数参数，不然就会编译报错
			- Standard C++11 required! 
  
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have many 
  * bugs, update the version Number, write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  *
  * <h2><center>&copy; Copyright (c) 2023 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */
#ifndef _DIFF_CALCULATOR_H_
#define _DIFF_CALCULATOR_H_

#include "Drivers/Components/SRML_Timer.h"

#ifdef __cplusplus
/**
 * @brief 微分计算器
 * 
 * @tparam Diff_Filter_Type 滤波器类型
 */
template <class Diff_Filter_Type>
class DiffCalculator: public SRML_Timer
{
public:
    /* 用模板构造函数，解决不同filter的构造函数参数不同的问题, 如果构造函数的参数不符合filter类型的构造函数传参，则会编译失败 */
    template <typename... Args>
    DiffCalculator(Args... args) : filter(args...) {}

    float get_diff() const { return diff; }

    float calc(float input)
    {
        if (UpdataTimeStamp()) // 计算dt
            return 0;

        return calc(input, this->dt);
    }

    float calc(float input, float _dt)
    {
        float origin_diff;

        origin_diff = (input - last_data) / _dt; // 差分原始数据
        last_data = input;                       // 更新上次数据

        diff = get_filter_result(origin_diff);
        return diff;
    }

protected:
    Diff_Filter_Type filter;
    float get_filter_result(float data)
    {
        return filter.f(data);
    }
    float last_data = 0;
    float diff;
};
#endif  /* __cplusplus */

#endif  /* _DIFF_CALCULATOR_H_ */
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/

