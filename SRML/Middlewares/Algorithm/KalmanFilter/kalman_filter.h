/**
 ******************************************************************************
 * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
 * @file    kalman_filter.h
 * @author  许志成
 * @brief   Header file
 ******************************************************************************
 * @attention
 *
 * if you had modified this file, please make sure your code does not have many
 * bugs, update the version Number, write dowm your name and the date, the most
 * important is make sure the users will have clear and definite understanding
 * through your new brief.
 *
 * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
 * All rights reserved.</center></h2>
 ******************************************************************************
 */
#pragma once

#ifndef _KALMAN_FILTER_H_
#define _KALMAN_FILTER_H_
/* Includes ------------------------------------------------------------------*/
#include "srml_config.h"

#ifdef __cplusplus

#if USE_SRML_KALMAN

#if USE_SRML_MATH
#include "Middlewares/Algorithm/Math/PX4_math.h"
#endif

using namespace matrix;

template <uint16_t StateDim, uint16_t MeasureDim, uint16_t ControlDim>
class KalmanFilter_Classdef
{
public:
    void parameter_config(const Matrix<float, StateDim, 1>& X_0, const Matrix<float, StateDim, StateDim>& P_0)
    {
        X_posterior = X_0;
        P_posterior = P_0;
    }

    void InputZ(const Matrix<float, MeasureDim, 1>& curZ)
    {
        Z = curZ;
    }

    void InputU(const Matrix<float, 1, ControlDim>& curU)
    {
        U = curU;
    }

    void SetQ(const Matrix<float, StateDim, StateDim>& newQ)
    {
        Q = newQ;
    }

    void SetR(const Matrix<float, MeasureDim, MeasureDim>& newR)
    {
        R = newR;
    }

    void SetH(const Matrix<float, MeasureDim, StateDim>& newH)
    {
        H = newH;
    }

    void SetA(const Matrix<float, StateDim, StateDim>& newA)
    {
        A = newA;
    }

    void SetB(const Matrix<float, StateDim, ControlDim>& newB)
    {
        B = newB;
    }

    Matrix<float, StateDim, 1> getX() const
    {
        return X_posterior;
    }

    void kalmanUpdate(void)
    {
        User_Func1_f();

        if (SkipEq1 == false)
        {
            X_prior = A * X_posterior + B * U;
        }

        User_Func2_f();

        if (SkipEq2 == false)
        {
            P_prior = A * P_posterior * A.T() + Q;
        }

        User_Func3_f();

        if (SkipEq3 == false)
        {
			SquareMatrix<float, MeasureDim> temp = (H * P_prior * H.T() + R);
            K = P_prior * H.T() * temp.I();
        }

        User_Func4_f();

        if (SkipEq4 == false)
        {
            X_posterior = X_prior + K * (Z - H * X_prior);
        }

        User_Func5_f();

        if (SkipEq5 == false)
        {
            P_posterior = (eye<float, StateDim>() - K * H) * P_prior;
        }
    }

    // 配合用户定义函数使用,作为标志位用于判断是否要跳过标准KF中五个环节中的任意一个
    bool SkipEq1 = false;
    bool SkipEq2 = false;
    bool SkipEq3 = false;
    bool SkipEq4 = false;
    bool SkipEq5 = false;

protected:
    Matrix<float, StateDim, 1> X_prior;     // 先验卡尔曼状态量
    Matrix<float, StateDim, 1> X_posterior; // 卡后验尔曼状态量

    Matrix<float, MeasureDim, 1> Z; // 观测向量
    Matrix<float, ControlDim, 1> U; // 输入向量

    Matrix<float, StateDim, StateDim> A;   // 状态空间矩阵
    Matrix<float, StateDim, ControlDim> B; // 控制矩阵

    Matrix<float, MeasureDim, StateDim> H; // 观测矩阵

    Matrix<float, StateDim, MeasureDim> K; // 卡尔曼增益

    Matrix<float, StateDim, StateDim> P_prior;     // 先验估计协方差矩阵
    Matrix<float, StateDim, StateDim> P_posterior; // 后验估计协方差矩阵
    Matrix<float, MeasureDim, MeasureDim> R;       // 观测噪声协方差矩阵
    Matrix<float, StateDim, StateDim> Q;       // 过程噪声协方差矩阵

    // 用户定义函数,可以替换或扩展基准KF的功能
    virtual void init()
    {}
    virtual void User_Func1_f()
    {}
    virtual void User_Func2_f()
    {}
    virtual void User_Func3_f()
    {}
    virtual void User_Func4_f()
    {}
    virtual void User_Func5_f()
    {}
};

#endif // USE_SRML_KALMAN

#endif // c++

#endif // header
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
