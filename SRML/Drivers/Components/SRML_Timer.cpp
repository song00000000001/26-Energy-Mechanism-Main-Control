/**
 ******************************************************************************
 * Copyright (c) 2023 - ~, SCUT-RobotLab Development Team
 * @file SRML_Timer.cpp
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
#include "SRML_Timer.h"
#include "drv_timer.h"
#include <stddef.h>
SystemTick_Fun SRML_Timer::Get_SystemTick = Get_SystemTimer;

uint8_t SRML_Timer::UpdataTimeStamp(void)
{
  uint32_t now_time;
  
  /*Check `Get_SystemTick` */
  if(SRML_Timer::Get_SystemTick != NULL)
  {
    /*Convert to system time*/
    if (last_time == 0)
    {
      last_time = SRML_Timer::Get_SystemTick();
      return 1;
    }
    now_time = SRML_Timer::Get_SystemTick();

    /*Overflow*/
    if (now_time < last_time)
    {
      if(last_time - now_time > INT32_MAX) // 真溢出
        dt = (float)(now_time + (0xFFFFFFFF - last_time));
      else
      {
        return dt; // 假溢出返回上一次dt
      }
    }
    else
      dt = (float)(now_time - last_time);

    last_time = now_time;

    dt *= 0.000001f;
    
    return 0;
  }
  else{
    dt = 0;
    return 1;
  }
}

/**
 * @brief  Regist get time function(1Tick = 1us)
 * @param  realTime_fun: Pointer of function to get system real time
 * @retval 1: success
           0: error input param
 * @author
 */
uint8_t SRML_Timer::getMicroTick_regist(SystemTick_Fun getTick_fun)
{
  if(getTick_fun != NULL)
  {
    SRML_Timer::Get_SystemTick = getTick_fun;
    return 1;
  }
  else 
    return 0;
}
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
