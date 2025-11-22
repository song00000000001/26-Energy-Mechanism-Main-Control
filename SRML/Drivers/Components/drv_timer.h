/**
 ******************************************************************************
 * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
 * @file    drv_timer.h
 * @author  Mentos_Seetoo 1356046979@qq.com
 * @brief   Code for Timer Management in STM32 series MCU, supported packaged:
 *          - STM32Cube_FW_F4_V1.24.0.
 *          - STM32Cube_FW_F1_V1.8.0.
 *          - STM32Cube_FW_H7_V1.5.0.
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
#ifndef _DRV_TIMER_H
#define _DRV_TIMER_H

/* Includes ------------------------------------------------------------------*/
#include "srml_std_lib.h"

#ifdef __cplusplus
extern "C"
{
#endif
  /* Private macros ------------------------------------------------------------*/
  /* Private type --------------------------------------------------------------*/
  /* Exported macros -----------------------------------------------------------*/
  /* Exported types ------------------------------------------------------------*/
  typedef enum
  {
    USE_MODULE_DELAY = 1, /*!< 根据模块的系统计时实现 delay_ms_nos()*/
    USE_HAL_DELAY         /*!< 使用 HAL_Delay() 实现 delay_ms_nos()*/
  } EDelay_src;
  /* Exported variables ---------------------------------------------------------*/
  /* Exported function declarations --------------------------------------------*/
  void Timer_Init(TIM_HandleTypeDef *htim, EDelay_src src); // 模块使能函数
  uint64_t Get_SystemTimer(void);     // 获取系统启动时间（启动了多久），单位为us
  uint64_t Get_SystemTimer_ms(void);  // 获取系统启动时间（启动了多久），单位为ms
  void delay_ms_nos(uint32_t cnt);    // 以ms为单位进行延时
  void delay_us_nos(uint32_t cnt);    // 以us为单位进行延时

// 判断是否开启了回调函数重定向功能
#if (USE_HAL_TIM_REGISTER_CALLBACKS == 0)
  // 更新计数，在没有开启回调函数重定向功能时，需要在中断函数内手动添加
  void Update_SystemTick(void); 
#else
  // 启用回调函数重定向功能后，将函数变为空define，既不会因为符号缺失报错，也不会让更新计数被外部二次执行
  #define Update_SystemTick(void) 
#endif

#ifdef __cplusplus
}
#endif

#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
