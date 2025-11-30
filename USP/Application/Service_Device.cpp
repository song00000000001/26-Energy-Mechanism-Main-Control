/**
 ******************************************************************************
 * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
 * @file missle_ctrl.cpp
 * @author lpk
 * @brief 镖架主控代码
 * @date 2025-3-9
 * @version 1.0
 * @par Change Log：
 * <table>
 * <tr><th>Date <th>Version <th>Author <th>Description
 * <tr><td>2019-06-12 <td> 1.0 <td>S.B. <td>Creator
 * </table>
 *
 ==============================================================================
 ##### How to use this driver #####
 ==============================================================================
 @note
 -#
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
/* Includes ------------------------------------------------------------------*/
#include "internal.h"
#include "global_data.h"
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
TaskHandle_t LoaderMotor_Handle;
TaskHandle_t DR16_Handle;
TaskHandle_t Rx_Referee_Handle;
TaskHandle_t LaunchCtrl_Handle;
TaskHandle_t motor_HAndle;
TaskHandle_t Loader_Ctrl_Handle;
TaskHandle_t Vision_Task_Handle;
TaskHandle_t Yaw_Task_Handle;
/* Private function declarations ---------------------------------------------*/
void tskDR16(void *arg);
void Rx_Referee(void *arg);
void Vision_Task(void *arg);


/* Function prototypes -------------------------------------------------------*/
/**
 * @brief  Initialization of device management service
 * @param  None.
 * @return None.
 */
void Service_Devices_Init(void)
{
	xTaskCreate(tskLoaderMotor, "App.Motor", Small_Stack_Size, NULL, PriorityAboveNormal, &LoaderMotor_Handle);
	xTaskCreate(LaunchCtrl, "App.LaunchCtrl", Normal_Stack_Size, NULL, PriorityAboveNormal, &LaunchCtrl_Handle);
	xTaskCreate(Loader_Ctrl, "App.Loader_Ctrl", Small_Stack_Size, NULL, PriorityAboveNormal, &Loader_Ctrl_Handle);
	xTaskCreate(Vision_Task, "App.Vision_Task", Small_Stack_Size, NULL, PriorityAboveNormal, &Vision_Task_Handle);
	xTaskCreate(Yaw_Task, "App.Yaw_Task", Normal_Stack_Size, NULL, PriorityAboveNormal, &Yaw_Task_Handle);
#if USE_SRML_DR16
	xTaskCreate(tskDR16, "App.DR16", Small_Stack_Size, NULL, PrioritySuperHigh, &DR16_Handle);
#endif
#if USE_SRML_REFEREE
	xTaskCreate(Rx_Referee, "Rx_Referee", Normal_Stack_Size, NULL, PriorityNormal, &Rx_Referee_Handle);
#endif
}

/**
 * @brief 电视通信任务
 * @parma None
 * @return None
 */
void Vision_Task(void *arg)
{
	USART_COB UART_pack;
	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();
	UART_pack.port_num = 1;
	UART_pack.address = (uint8_t *)&vision_send_pack;
	UART_pack.len = sizeof(vision_send_pack);
	for (;;)
	{
		vTaskDelayUntil(&xLastWakeTime_t, 1);
		vision_send_pack.mode = 3;
		if (DR16.GetStatus() == DR16_ESTABLISHED && DR16.GetS1() == SW_DOWN) // 左拨杆拨到下，进入视觉模式
		{
			vision_send_pack.tracker_bit = 1;
			vision_send_pack.calibration_state = (DR16.GetS2() == SW_DOWN); // 右拨杆拨到下，开始标定
		}
		else
		{
			vision_send_pack.calibration_state = 0;
		}
		if (xTaskGetTickCount() - vision_last_recv_time > 150)
		{
			vision_recv_pack.target_mode = 0;
		}
		//	 SRML_UART_Transmit_DMA(&UART_pack);
		vTaskDelay(1);
	}
}
#if USE_SRML_DR16
/**
 *	@brief	Dr16 data receive task
 */
void tskDR16(void *arg)
{
	/* Cache for Task */
	USART_COB Rx_Package;
	/* Pre-Load for task */
	DR16.Check_Link(xTaskGetTickCount());
	/* Infinite loop */
	for (;;)
	{
		/* Enter critical */
		xSemaphoreTake(DR16_mutex, portMAX_DELAY);
		/*	等待数据	*/
		if (xQueueReceive(DR16_QueueHandle, &Rx_Package, 100) == pdPASS)
		{
			// Read Message
			DR16.DataCapture((DR16_DataPack_Typedef *)Rx_Package.address);
		}
		/*	检测遥控器连接 */
		DR16.Check_Link(xTaskGetTickCount());
		/*	判断是否连接 	 */
		if (DR16.GetStatus() != DR16_ESTABLISHED)
		{
			/**
			 * lost the remote control
			 */

			/* Leave critical */
			xSemaphoreGive(DR16_mutex);
			continue;
		}
		/*	更新遥控器控制	*/

		/* Leave critical */
		xSemaphoreGive(DR16_mutex);
	}
}
#endif

#if USE_SRML_REFEREE
/**
 * @brief  接受裁判系统数据
 * @param  None.
 * @return None.
 */
void Rx_Referee(void *arg)
{
	/* Preoad for task */
	USART_COB *referee_pack;
	TickType_t xLastWakeTime_t = xTaskGetTickCount();

	/* Infinite loop */
	for (;;)
	{
		if (xTaskNotifyWait(0x00000000, 0xFFFFFFFF, (uint32_t *)&referee_pack, portMAX_DELAY) == pdTRUE)
		{
			Referee.unPackDataFromRF((uint8_t *)referee_pack->address, referee_pack->len);
		}
	}
}
#endif
