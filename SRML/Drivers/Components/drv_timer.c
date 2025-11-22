/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    drv_timer.c
  * @author  Mentos_Seetoo 1356046979@qq.com M3chD09 rinngo17@foxmail.com
  * @brief   Code for Timer Management in STM32 series MCU, supported packaged:
  *          - STM32Cube_FW_F4_V1.24.0.
  *          - STM32Cube_FW_F1_V1.8.0.
  *          - STM32Cube_FW_H7_V1.5.0.
  * @date    2019-06-12
  * @version 3.0
  * @par Change Log：
  * <table>
  * <tr><th>Date        <th>Version  <th>Author         <th>Description
  * <tr><td>2019-06-12  <td> 1.0     <td>Mentos Seetoo  <td>Creator
  * <tr><td>2019-10-28  <td> 2.0     <td>Mentos Seetoo  <td>Add Timer manage object.
  * <tr><td>2021-04-06  <td> 3.0     <td>M3chD09        <td>Remove HAL_TIM_Base_Start in Timer_Init.
  * </table>
  *
  ==============================================================================
                            How to use this driver  
  ==============================================================================
    @note
      -# 使用`Timer_Init()`设置延时定时器`TIM_X`,设置`delay_ms()`函数使用HAL库的实现
          `HAL_Delay()`或使用模块内方法实现。		
      -# 配置`TIM_X`自增时间为1us，在对应中断函数中加入:`Update_SystemTick();`。
      -# 在需要延时的地方使用`delay_us_nos()`或`delay_ms_noe()`。
		
    @warning
      -# 本模块的所有延时函数均为堵塞式延时。
      -# 使用前必须进行初始化！
      -# 添加预编译宏`USE_FULL_ASSERT`可以启用断言检查。
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

#include "srml_config.h"

#if USE_SRML_TIMER

/* Includes ------------------------------------------------------------------*/
#include "drv_timer.h"

/* Private define ------------------------------------------------------------*/
#define microsecond() Get_SystemTimer() // 微秒
/* Private variables ---------------------------------------------------------*/
__CCM volatile uint32_t SystemTimerCnt;

struct 
{
	TIM_HandleTypeDef*	htim_x;
	EDelay_src	delay_ms_src;
  uint32_t 	timer_period;
} Timer_Manager;
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
static void Error_Handler(void);

#if (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
  void PeriodElapsedCallback(TIM_HandleTypeDef *htim);             /*!< TIM Period Elapsed Callback                             */
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */

/* function prototypes -------------------------------------------------------*/
/**
* @brief  Initialize Timer
* @param  htim_x : HAL Handler of timer x.
* @param  src : Choose the src for delay_ms().
* @retval None
*/
void Timer_Init(TIM_HandleTypeDef* htim, EDelay_src src)
{
	/* Check the parameters */
	assert_param(htim != NULL);

#if (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
  htim->PeriodElapsedCallback = PeriodElapsedCallback;
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */

  Timer_Manager.htim_x = htim;
	Timer_Manager.delay_ms_src = src;

  // 自动获取初始化时的定时器周期
	Timer_Manager.timer_period = Timer_Manager.htim_x->Init.Period + 1;
  
  // 清除定时器中断
	__HAL_TIM_CLEAR_IT(Timer_Manager.htim_x, TIM_IT_UPDATE);
  if(HAL_TIM_Base_Start_IT(Timer_Manager.htim_x)!=HAL_OK)
      Error_Handler();
}


/**
* @brief  Get the system tick from timer.
* @param  None
* @retval current tick.
*/
uint64_t Get_SystemTimer(void)
{
  // 先读一遍update_cnt,再读一遍定时器计数值，再读一遍update_cnt，如果两次cnt不一样，
	// 则表示中途进过一次定时器中断，timer_cnt归零过一次，不能确保此时计数值timer_cnt
	// 对应的是哪一个update_cnt，需要重新读取一次timer_cnt，并按照这个新的来计算当前
	// 系统时间。
	__HAL_TIM_DISABLE_IT(Timer_Manager.htim_x, TIM_IT_UPDATE);
	uint32_t update_cnt = SystemTimerCnt;
	__HAL_TIM_ENABLE_IT(Timer_Manager.htim_x, TIM_IT_UPDATE);

	uint32_t timer_cnt	= Timer_Manager.htim_x->Instance->CNT;

	__HAL_TIM_DISABLE_IT(Timer_Manager.htim_x, TIM_IT_UPDATE);
	uint32_t update_cnt_confirm = SystemTimerCnt;
	__HAL_TIM_ENABLE_IT(Timer_Manager.htim_x, TIM_IT_UPDATE);

	if(update_cnt == update_cnt_confirm)
	{
		return update_cnt * Timer_Manager.timer_period + timer_cnt;
	}
	else
	{
		// 如果两次获取到的溢出次数不一样，证明中间经历过一次溢出，
		// 不能确定定时器计数值对应的是溢出前获取的还是溢出
		// 后获取的，因此重新取一次，这次一定是对应溢出后的定时器计数值。
		uint32_t timer_cnt_confirm = Timer_Manager.htim_x->Instance->CNT;
		return update_cnt_confirm * Timer_Manager.timer_period + timer_cnt_confirm;
  }
	// return (uint64_t)Timer_Manager.htim_x->Instance->CNT + SystemTimerCnt * 0xffffUL;
}

uint64_t Get_SystemTimer_ms(void)
{
  return Get_SystemTimer() / 1000;
}

#if (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
  /**
   * @brief 用于替换SRML定时器的中断回调函数
   * 
   * @param htim 回调函数格式，实际并没有用到
   */
  void PeriodElapsedCallback(TIM_HandleTypeDef *htim)
  {
    SystemTimerCnt++;
  }
#else
  /**
  * @brief  Update system tick that had run in Timer Interupt.
  * @not    Add this function into Timer interupt function.
  * @param  None
  * @retval None
  */
  void Update_SystemTick(void)
  {
    SystemTimerCnt++;
  }
#endif

/**
* @brief  Delay microsecond.
* @param  cnt : microsecond to delay 
* @retval None
*/
void delay_us_nos(uint32_t cnt)
{
	uint64_t temp = cnt  + microsecond();
	while(temp >= microsecond());
}

/**
* @brief  Delay millisecond.
* @param  cnt : millisecond to delay
* @retval None
*/
void delay_ms_nos(uint32_t cnt)
{
	if(Timer_Manager.htim_x != NULL && Timer_Manager.delay_ms_src == USE_MODULE_DELAY)
	{
		uint64_t temp = cnt * 1000 + microsecond();
		while(temp >= microsecond());
	}
	else
		HAL_Delay(cnt);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
static void Error_Handler(void)
{
  /* Nromally the program would never run here. */
  while(1){}
}

#endif /* USE_SRML_TIMER */

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
