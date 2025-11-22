/**
  ******************************************************************************
  * Copyright (c) 2023 - ~, SCUT-RobotLab Development Team
  * @file    DiffCalculator.h
  * @author  张至睿、余俊晖
  * @brief   各种二阶滤波器
  * @date    2024-04-25
  * @version 2.0
  *
  ==============================================================================
                            How to use this library
  ==============================================================================
    @note
            - 构造函数输入：截止频率，采样频率，阻尼比
            - 使用LPF_SecondOrder_Classdef::f(float input)进行滤波
    @warning

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
#ifndef _SECOND_ORDER_FILTER_H
#define _SECOND_ORDER_FILTER_H
#include "filters.h"

#ifdef __cplusplus

#define SecondOrderButterworthLPF LPF_SecondOrder_Classdef // 兼容老的类名，将于24赛季结束后删除

/**
 * @brief 二阶低通滤波器(Low Pass Filter)
 *
 */
class LPF_SecondOrder_Classdef;
/**
 * @brief 三阶低通滤波器(Low Pass Filter)
 *
 */
class LPF_ThirdOrder_Classdef;

/**
 * @brief 二阶高通滤波器(High Pass Filter)
 *
 */
class HPF_SecondOrder_Classdef;

/**
 * @brief 二阶带阻滤波器(Band Stop Filter)
 *
 */
class BSF_SecondOrder_Classdef;


class LPF_SecondOrder_Classdef : public SRML_Filter::filter_base<2>
{
public:
    LPF_SecondOrder_Classdef(float cutoff_freq, float sampling_freq, float _zeta = sqrtf(2) / 2)
    {
        calculateCoefficients(cutoff_freq, sampling_freq, _zeta);
    }

    /**
     * @brief 计算滤波器输入、输出系数
     *
     * @param cutoff_freq 截止频率/Hz
     * @param sampling_freq 采样频率/Hz
     * @param _zeta 阻尼比
     */
    void calculateCoefficients(float cutoff_freq, float sampling_freq, float _zeta = sqrtf(2) / 2)
    {
        SRML_Filter::cal2ndOrderLPFCoeffs(cutoff_freq, sampling_freq, _zeta, a, b);
    }
};

class HPF_SecondOrder_Classdef : public SRML_Filter::filter_base<2>
{
public:
    HPF_SecondOrder_Classdef(float cutoff_freq, float sampling_freq, float _zeta = sqrtf(2) / 2)
    {
        calculateCoefficients(cutoff_freq, sampling_freq, _zeta);
    }

    /**
     * @brief 计算滤波器输入、输出系数
     *
     * @param cutoff_freq 截止频率/Hz
     * @param sampling_freq 采样频率/Hz
     * @param _zeta 阻尼比
     */
    void calculateCoefficients(float cutoff_freq, float sampling_freq, float _zeta = sqrtf(2) / 2)
    {
        SRML_Filter::cal2ndOrderHPFCoeffs(cutoff_freq, sampling_freq, _zeta, a, b);
    }
};

class BSF_SecondOrder_Classdef : public SRML_Filter::filter_base<2>
{
public:
    BSF_SecondOrder_Classdef(float stop_freq, float sampling_freq, float depth, float width)
    {
        calculateCoefficients(stop_freq, sampling_freq, depth, width);
    }

    /**
     * @brief 计算滤波器输入、输出系数
     *
     * @param stop_freq 陷波频率
     * @param sampling_freq 采样频率
     * @param depth 陷波深度，不应超过0.7
     * @param width 陷波宽度
     */
    void calculateCoefficients(float stop_freq, float sampling_freq, float depth, float width)
    {
        SRML_Filter::cal2ndOrderBSFCoeffs(stop_freq, sampling_freq, depth, width, a, b);
    }
};

class LPF_ThirdOrder_Classdef : public SRML_Filter::filter_base<3>
{
public:
    LPF_ThirdOrder_Classdef(float cutoff_freq, float sampling_freq, float _zeta = sqrtf(2) / 2)
    {
        calculateCoefficients(cutoff_freq, sampling_freq, _zeta);
    }

    /**
     * @brief 计算滤波器输入、输出系数
     * 
     * @param cutoff_freq 截止频率/Hz
     * @param sampling_freq 采样频率/Hz
     * @param _zeta 阻尼比
     */
    void calculateCoefficients(float cutoff_freq, float sampling_freq, float _zeta = sqrtf(2) / 2)
    {
        SRML_Filter::cal3rdOrderLPFCoeffs(cutoff_freq, sampling_freq, _zeta, a, b);
    }
};

#endif /* __cplusplus */

#endif /* _SECOND_ORDER_FILTER_H */
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
