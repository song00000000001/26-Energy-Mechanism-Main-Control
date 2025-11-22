/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    PID.cpp
  * @author  BigeYoung 851756890@qq.com & M3chD09
  * @brief   PID controller sets. This file provides traditional PID controller
  *			     and modern Fuzzy PID controller.
  * @date    2019-06-26
  * @version 1.2
  *
  ==============================================================================
                            How to use this library
  ==============================================================================
    @note
      The following Controllers is provied by this lib: \n
      - Open loop controller.
      - Traditional PID controller.
      - Fuzzy PID controller.
    @see
      - To view more details about how to use this library, please visit: \n
        https://www.scut-robotlab.cn/git/Embedded/motor.git
    @warning
      - Function `getMicroTick_regist()` must be regist once before using this library!
      - 1 System tick in this library is standing for 1 us.
      - Header file `"filter.h"` required!
      - Standard C++11 required!
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

/* Includes ------------------------------------------------------------------*/
#include "srml_config.h"

#if USE_SRML_PID

#include "PID.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* function prototypes -------------------------------------------------------*/
float myPID::Adjust()
{
    /*Error time*/
    if (UpdataTimeStamp())
        return 0;

    Error = Target - Current;

    if (std::abs(Error) < DeadZone)
    {
        Out = 0;
        return Out;
    }

    /* Using Low-Pass Filter to preprocess*/
    if (ErrorFilter != nullptr)
        Error = ErrorFilter(Error);

    P_Term = Kp * Error;

    /* PID with Changing integration rate */
    float I_VarSpeedf = 0;
    if (std::abs(Error) <= VarSpeed_I_B)
        I_VarSpeedf = 1;
    else if (std::abs(Error) <= double(VarSpeed_I_A) + VarSpeed_I_B)
        I_VarSpeedf = (VarSpeed_I_A - (std::abs(Error)) + VarSpeed_I_B) / VarSpeed_I_A;

    float integral_max = fabsf(I_Term_Max / Ki);
    if (std::abs(Ki) > std::numeric_limits<float>::epsilon())
    {
        integral_e += I_VarSpeedf * Error * dt;
        /*Constrain*/
        integral_e = std_lib::constrain(integral_e, -integral_max, integral_max);
    }
    else
    {
        integral_e = 0;
    }

    /* Using Integral separation */
    if (std::abs(Error) < I_SeparThresh)
    {
        I_Term = Ki * integral_e;
        /*Constarin*/
        I_Term = std_lib::constrain(I_Term, -I_Term_Max, I_Term_Max);
    }
    else
    {
        /*Clear*/
        I_Term = 0;
    }

    float d_err = 0;
    if (D_of_Current)
        d_err = (Current - pre_current) / dt;
    else
        d_err = (Error - pre_error) / dt;

    if (DiffFilter != nullptr)
        d_err = DiffFilter(d_err);
    D_Term = Kd * d_err;

    pre_error = Error;
    pre_current = Current;

    Out = P_Term + I_Term + D_Term;

    /* Constarin */
    Out = std_lib::constrain(Out, -Out_Max, Out_Max);

    return Out;
}

float myPID::Adjust_importDiff(float diff)
{
    /*Error time*/
    if (UpdataTimeStamp())
        return 0;

    Error = Target - Current;

    if (std::abs(Error) < DeadZone)
    {
        Out = 0;
        return Out;
    }

    /* Using Low-Pass Filter to preprocess*/
    if (ErrorFilter != nullptr)
        Error = ErrorFilter(Error);

    P_Term = Kp * Error;

    /* PID with Changing integration rate */
    float I_VarSpeedf = 0;
    if (std::abs(Error) <= VarSpeed_I_B)
        I_VarSpeedf = 1;
    else if (std::abs(Error) <= double(VarSpeed_I_A) + VarSpeed_I_B)
        I_VarSpeedf = (VarSpeed_I_A - (std::abs(Error)) + VarSpeed_I_B) / VarSpeed_I_A;

    float integral_max = fabsf(I_Term_Max / Ki);
    if (std::abs(Ki) > std::numeric_limits<float>::epsilon())
    {
        integral_e += I_VarSpeedf * Error * dt;
        /*Constrain*/
        integral_e = std_lib::constrain(integral_e, -integral_max, integral_max);
    }
    else
    {
        integral_e = 0;
    }

    /* Using Integral separation */
    if (std::abs(Error) < I_SeparThresh)
    {
        I_Term = Ki * integral_e;
        /*Constarin*/
        I_Term = std_lib::constrain(I_Term, -I_Term_Max, I_Term_Max);
    }
    else
    {
        /*Clear*/
        I_Term = 0;
    }

    if (DiffFilter != nullptr)
        diff = DiffFilter(diff);
    D_Term = Kd * diff;

    pre_error = Error;
    pre_current = Current;

    Out = P_Term + I_Term + D_Term;

    /* Constarin */
    Out = std_lib::constrain(Out, -Out_Max, Out_Max);

    return Out;
}

float FuzzyPID::Adjust()
{
    if (UpdataTimeStamp())
        return 0;

    Error = Target - Current;
    float d_err = (Error - pre_error) / dt;
    pre_error = Error;
    d_err = LowPass_d_err.f(d_err);

    Out = 0;
    float ms_sum = 0;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            float ms_error = Membership(Error, error_N, error_P, i);
            float ms_d_err = Membership(d_err, d_err_N, d_err_P, j);
            float ms = ms_error * ms_d_err;
            ms_sum += ms;
            Out += ms * Table[i][j];
        }
    }
    Out /= ms_sum;
    return Out;
}

float FuzzyPID::Membership(float x, float Negative, float Positive, int POS)
{
    if (x < Negative)
    {
        if (POS == 0)
            return 1;
        else
            return 0;
    }
    else if (x < 0)
    {
        if (POS == 0)
            return (1 - Linear(x, Negative, 0));
        else if (POS == 1)
            return Linear(x, Negative, 0);
        else
            return 0;
    }
    else if (x < Positive)
    {
        if (POS == 1)
            return (1 - Linear(x, 0, Positive));
        else if (POS == 2)
            return Linear(x, 0, Positive);
        else
            return 0;
    }
    else
    {
        if (POS == 2)
            return 1;
        else
            return 0;
    }
}

#endif /* USE_SRML_PID */
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
