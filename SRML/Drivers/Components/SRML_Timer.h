/**
 ******************************************************************************
 * Copyright (c) 2023 - ~, SCUT-RobotLab Development Team
 * @file SRML_Timer.h
 * @author 余俊晖 (2460857175@qq.com)
 * @brief SRML计时器功能，us级精度，主要用于PID的积分、微分计算器的计算等等
 * @version 1.0
 * @date 2023-09-02
 *
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
#ifndef SRML_Timer_H
#define SRML_Timer_H
#include <stdint.h>

#ifdef __cplusplus
typedef uint64_t (*SystemTick_Fun)(void);

class SRML_Timer
{
public:
    static uint8_t getMicroTick_regist(SystemTick_Fun getTick_fun);
                                            /*<! Regist get time function */
protected:
    static SystemTick_Fun Get_SystemTick;   /*<! Pointer of function to get system tick */
    float dt;				                        /*!< Differentiation of real time*/
    uint64_t last_time; 	                  /*!< Last recorded real time from systick*/
    uint8_t UpdataTimeStamp(void);                                 
};
#endif  /* __cplusplus */

#endif  /* SRML_Timer_H */
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
