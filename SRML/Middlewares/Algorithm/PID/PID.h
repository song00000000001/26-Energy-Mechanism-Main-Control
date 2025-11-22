/**
 ******************************************************************************
 * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
 * @file    PID.h
 * @author  BigeYoung 851756890@qq.com & M3chD09
 * @brief   PID controller sets. This file provides traditional PID controller
 *			     and modern Fuzzy PID controller.
 ******************************************************************************
 * @attention
 *
 * if you had modified this file, please make sure your code does not have any
 * bugs, update the version Number, write dowm your name and the date. The most
 * important thing is make sure the users will have clear and definite under-
 * standing through your new brief.
 *
 * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
 * All rights reserved.</center></h2>
 ******************************************************************************
 */
#ifndef _PID_H_
#define _PID_H_
#pragma once

#ifdef __cplusplus
/* Includes ------------------------------------------------------------------*/
#include <stddef.h>

#include "srml_std_lib.h"
#include "Drivers/Components/SRML_Timer.h"
#include "Middlewares/Algorithm/Filters/Filters.h"

#if defined(USE_AC6)
#include <limits.h>
#elif defined(USE_AC5)
#include <limits>
#include <limits.h>

#ifndef __FLT_MAX__
#define __FLT_MAX__ FLT_MAX
#endif

#ifndef __FLT_MIN__
#define __FLT_MIN__ FLT_MIN
#endif

#endif

#include <math.h>
/* Private macros ------------------------------------------------------------*/

/* Private type --------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/**
 * @brief Class for open loop control.
 */
class OpenLoop
{
public:
    OpenLoop(float gain = 1.0f) :
      Gain(gain)
    {}

    float Adjust()
    {
        Out = Gain * Target;
        return Out;
    }
    float Target = 0, Current = 0;
    float Out = 0;

    float Gain = 1.0f;
};

/**
 * @brief Class for traditional PID control.
 */
class myPID : public SRML_Timer
{
public:
    myPID()
    {}
    myPID(float _Kp, float _Ki, float _Kd) :
      Kp(_Kp), Ki(_Ki), Kd(_Kd)
    {}
    void SetPIDParam(float _Kp, float _Ki, float _Kd, float _I_Term_Max, float _Out_Max)
    {
        Kp = _Kp;
        Ki = _Ki;
        Kd = _Kd;
        I_Term_Max = fabsf(_I_Term_Max);
        Out_Max = fabsf(_Out_Max);
    };
    float Adjust();
    float Adjust_importDiff(float diff);
    void clean_intergral(void)
    {
        integral_e = 0;
    }
    float Target = 0, Current = 0, Error = 0;
    float Out = 0;

    float Kp = 0, Ki = 0, Kd = 0;
    float I_Term_Max = 0; /*<! I项限幅 */
    float Out_Max = 0;    /*<! 输出限幅 */

    float I_Term = 0; /* 积分器输出 */
    float P_Term = 0; /* 比例器输出 */
    float D_Term = 0; /* 微分器输出 */

    float I_SeparThresh = __FLT_MAX__; /*!< 积分分离阈值，需为正数。std::abs(error)大于该阈值取消积分作用。*/

    float VarSpeed_I_A = __FLT_MAX__; /*!< 变速积分 A，需为正数。*/
    float VarSpeed_I_B = __FLT_MAX__; /*!< 变速积分 B，需为正数， \n
                                     在 error<=B 的区间内，为普通积分效果， \n
                                     在 B<error<=A+B 的区间内，为变速积分效果， \n
                                     在 A+B<error 的区间内，不继续积分。*/

    float DeadZone = 0; /*!< 死区，需为整数，std::abs(error)小于DeadZone时，输出为0。 */

    float (*ErrorFilter)(float data) = nullptr;
    float (*DiffFilter)(float data) = nullptr;

    bool D_of_Current = false; /*!< 启用微分先行，文献中Current多译作Process Variable(PV)。 */
private:
    float integral_e = 0;
    float pre_error = 0;
    float pre_current = 0;
};

/**
 * @brief Class for Fuzzy PID control.
 */
class FuzzyPID : public SRML_Timer
{
public:
    float Adjust();
    float Target = 0, Current = 0, Error;
    float Out = 0;

    float Table[3][3] = {
      /*				d_err_N, d_err_Z, d_err_P */
      /* error_N */ {-200, -100, 0},
      /* error_Z */ {-100, 0, 100},
      /* error_P */ {0, 100, 200}};

    // N(负数, Negative)
    // Z(零, Zero)
    // P(正数, Positive)
    float error_N = -__FLT_MAX__, error_P = __FLT_MAX__;
    float d_err_N = -__FLT_MAX__, d_err_P = __FLT_MAX__;

    LowPassFilter LowPass_d_err = LowPassFilter(0);

private:
    float pre_error = 0;
    float Membership(float x, float Negative, float Positive, int POS);
    float Linear(float X, float X0, float X1)
    {
        return (X - X0) / (X1 - X0);
    }
};

/* Exported variables ---------------------------------------------------------*/
/* Exported function declarations ---------------------------------------------*/
#endif /* __cplusplus */

#endif /* _PID_H_ */
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
