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
/* Private define ------------------------------------------------------------*/
int status = 1;																			// 装填状态（1~4分别对应4发飞镖）
int cnt = 0;																				// 控制status在每次装填后只加一
int goal;																						// 装填电机的目标值
int open = 0;																				// 此值为0时装填镖体，为1时准备发射
float visionangle;																	// 视觉目标角
myPID anglepid;																			// 装填电机的编码器环
myPID speedpid;																			// 装填电机的速度环
static bool vision_aim_state = 0;										// 视觉瞄准状态
static float vision_base_angle, storage_base_angle; // 视觉基准角、原基准角暂存
/* Private variables ---------------------------------------------------------*/
TaskHandle_t LoaderMotor_Handle;
TaskHandle_t DR16_Handle;
TaskHandle_t Rx_Referee_Handle;
TaskHandle_t LaunchCtrl_Handle;
TaskHandle_t HX711_Handle;
TaskHandle_t motor_HAndle;
TaskHandle_t Loader_Ctrl_Handle;
TaskHandle_t Vision_Task_Handle;
TaskHandle_t Yaw_Task_Handle;
/* Private function declarations ---------------------------------------------*/
void tskLoaderMotor(void *arg);
void LaunchCtrl(void *arg);
void Loader_Ctrl(void *arg);
void tskDR16(void *arg);
void Rx_Referee(void *arg);
void Vision_Task(void *arg);
void Yaw_Task(void *arg);
void turn1(float angle);
void HX711_Read(void *arg);
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
	xTaskCreate(HX711_Read, "App.Hx711", Small_Stack_Size, NULL, PriorityAboveNormal, &HX711_Handle);
#if USE_SRML_DR16
	xTaskCreate(tskDR16, "App.DR16", Small_Stack_Size, NULL, PrioritySuperHigh, &DR16_Handle);
#endif
#if USE_SRML_REFEREE
	xTaskCreate(Rx_Referee, "Rx_Referee", Normal_Stack_Size, NULL, PriorityNormal, &Rx_Referee_Handle);
#endif
}
/**
 * @brief 发射主控任务
 * @parma None
 * @return None
 */
enum Missle_State_t
{
	DEINIT,
	WAIT_ACT,
	PULL,
	BACK,
	WAIT_SHOOT
};

Missle_State_t state = DEINIT;
float yaw_target = 0, yaw_goal = 0, igniter_target_pos = 0, igniter_goal_pos = 0;
/**
 * @brief 飞镖调参版数据查询接口
 *
 * @return uint8_t
 */
uint8_t getLoadStatus()
{
	return status;
}
bool Is_Launching()
{
	return (state == WAIT_SHOOT || state == PULL);
}
void LaunchCtrl(void *arg)
{
	Motor_CAN_COB Tx_Buff;
	Motor_CAN_COB Tx_Buff1;
	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	for (;;)
	{
		vTaskDelay(1);
		if (DR16.GetStatus() == DR16_ESTABLISHED)
		{
			if (DR16.GetS1() != SW_DOWN || vision_aim_state == 1) // 视觉调整yaw时不允许发射
			{
				switch (state)
				{
				case DEINIT: // 初始化
					Launch.Igniter_On();
					Launch.Deliver_Init(); // 左右滑块
					Launch.Igniter_Init(); // 扳机丝杆
					if (Launch.Igniter_Init_flag == 1 && Launch.is_Deliver_Init() == 1)
					{
						state = WAIT_ACT;
					}
					break;
				case WAIT_ACT: // 等待蓄力指令,并且丝杆移动到指定位置
					Launch.Deliver_Init();
					Launch.Igniter_On();
					if (cnt == 0)
					{
						status++;
						cnt = 1; // 使status只加一次
					}
					if (DR16.GetS1() == SW_MID || DR16.GetS1() == SW_DOWN) // 丝杆数据用调参板数据
					{
						igniter_goal_pos = DartsData[status - 1].Ignitergoal[HitTarget];
						igniter_goal_pos = std_lib::constrain(igniter_goal_pos, 3.f, 260.f);
						Launch.Igniter_Pos_Set(igniter_target_pos);
					}
					if (DR16.GetS2() == SW_DOWN && Launch.PID_Igniter_Angle.Error < 0.1) // 右边拨杆下拨且丝杆移动到位后，滑块拉下，扳机扣下
					{
						state = PULL;
					}
					break;

				case PULL:
					if (Launch.Pull_Ready_flag != 1)
					{
						Launch.Igniter_Off();
						Launch.Deliver_Pull();
					}
					else
					{
						Launch.Igniter_Off(); // 扳机扣下
						vTaskDelay(300);
						state = BACK;
					}
					break;
				case BACK:
					Launch.Igniter_Off();
					Launch.Deliver_Init();
					open = 1; // 小风车强制切换到发射状态
					goal = 2750;
					if (Launch.is_Deliver_Init() == 1)
					{
						state = WAIT_SHOOT;
					}
					break;
				case WAIT_SHOOT: // 进入待发射状态
					Launch.Deliver_Init();
					if (DR16.GetS2() == SW_UP) // 扳机释放，发射！
					{
						Launch.Igniter_On();
						state = WAIT_ACT; // 回到等待指令状态，准备下一次发射
						vTaskDelay(2000);
						open = 0;
						cnt = 0; // 更新装填状态，准备装填下一发
					}
					else
					{
						Launch.Igniter_Off();
					}
					break;

				default:
					break;
				}
			}
		}
		/*丝杆电机进入调试模式，遥控可控制丝杆的前后*/
		if (state != PULL && DR16.GetS1() == SW_UP && Launch.Igniter_Init_flag == 1) // 左拨杆朝上，进入调试模式
		{
			igniter_target_pos += DR16.Get_RY_Norm() * 0.03f;
			igniter_target_pos = std_lib::constrain(igniter_target_pos, 0.f, 260.f);
			Launch.Igniter_Pos_Set(igniter_target_pos);
		}
		Launch.adjust();
		/*关控保护*/
		if (DR16.GetStatus() != DR16_ESTABLISHED)
		{
			Launch.disable();
			Yaw.disable();
			loadermotor[0].Out = 0;
		}
		/*切换目标*/
		if (DR16.Get_RY_Norm() < -0.9f)
		{
			if (DR16.Get_RX_Norm() < -0.9f)
			{
				vTaskDelay(1000);
				HitTarget = DR16.Get_RX_Norm() < -0.9f ? Outpost : HitTarget;
			}
			else if (DR16.Get_RX_Norm() > 0.9f)
			{
				vTaskDelay(1000);
				HitTarget = DR16.Get_RX_Norm() > 0.9f ? Base : HitTarget;
			}
		}
		/*打包数据发送*/
		MotorMsgPack(Tx_Buff, Launch.DeliverMotor[L], Launch.DeliverMotor[R], Launch.IgniterMotor);
		xQueueSend(CAN2_TxPort, &Tx_Buff.Id200, 0);
	}
}
/**
 * @brief Yaw轴控制任务
 * @parma None
 * @return None
 */
void Yaw_Task(void *arg)
{
	Motor_CAN_COB Tx_Buff1;
	TickType_t xLastWakeTime_t;
	static uint32_t tick = xTaskGetTickCount();
	uint32_t currentTick = xTaskGetTickCount();
	yaw_target = 0;
	for (;;)
	{
		vTaskDelayUntil(&xLastWakeTime_t, 1);
		if (DR16.GetStatus() == DR16_ESTABLISHED)
		{
			if (Yaw.is_Yaw_Init() == 1 && DR16.GetS1() == SW_UP) // 左拨杆朝上，进入调试模式
			{
				yaw_target -= DR16.Get_LX_Norm() * 0.002f;
				yaw_target = std_lib::constrain(yaw_target, -10.2f, 10.2f);
				Yaw.update(yaw_target);
			}
			if (Yaw.is_Yaw_Init() == 1 && DR16.GetS1() == SW_DOWN) // 左拨杆朝下拨，进入视觉接管模式
			{
				storage_base_angle = Yaw_Angle[HitTarget];
				if (Yaw.is_Yaw_Init() == 1 && DR16.GetS1() == SW_DOWN)
				{
					//			if(vision_recv_pack.ros==3)//若视觉未识别到引导灯，则先自行扫描
					//			{
					//			    if(scan==0&&yaw_target<=7.5)
					//             {yaw_target+=0.01;
					//						  if(yaw_target==7.5){scan=1;}
					//						 }
					//					if(scan==1&&yaw_target>=-7.5)
					//					   {yaw_target-=0.01;
					//						  if(yaw_target==-7.5){scan=2;}
					//						 }
					//					if(scan==2){yaw_target=7.5;}
					//			}
					if (vision_recv_pack.ros == 1)
					{
						yaw_target += 0.0003;
					}
					if (vision_recv_pack.ros == 2)
					{
						yaw_target -= 0.0003;
					}
					if (vision_recv_pack.ros == 0)
					{
						yaw_target += 0;
					}
					yaw_target = std_lib::constrain(yaw_target, -10.2f, 10.2f);
					Yaw.update(yaw_target);
					Yaw_Angle[HitTarget] = yaw_target;
				}
			}
			else if (Yaw.is_Yaw_Init() == 1 && (vision_aim_state == 1 || DR16.GetS1() == SW_MID)) // 非视觉模式  or 瞄准完成
			{
				Yaw.update(_YawCorrectionAngle + Yaw_Angle[HitTarget]); // 更改Yaw轴角度
			}
			if (Yaw.is_Yaw_Init() != 1)
			{
				Yaw.init();
			}
			/*关控保护*/
			if (DR16.GetStatus() != DR16_ESTABLISHED)
			{
				Yaw.disable();
			}
			Yaw.adjust();
			/*打包发送*/
			MotorMsgPack(Tx_Buff1, Yaw.YawMotor);
			xQueueSend(CAN2_TxPort, &Tx_Buff1.Id1ff, 0);
		}
	}
}
/**
 * @brief 装填电机的编码器双环控制函数
 * @parma None
 * @return None
 */
void turn1(float angle)
{
	Motor_CAN_COB Tx_Buff = {};
	anglepid.Target = angle;
	anglepid.Current = loadermotor[0].getEncoder();
	anglepid.Adjust();
	speedpid.Target = anglepid.Out;
	speedpid.Current = loadermotor[0].getSpeed();
	speedpid.Adjust();
	loadermotor[0].Out = speedpid.Out;
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
/**
 * @brief 装填状态控制任务
 * @parma None
 * @return None
 */
void Loader_Ctrl(void *arg)
{
	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();
	for (;;)
	{
		vTaskDelayUntil(&xLastWakeTime_t, 1);
		if (status != 0 && status % 4 == 1) // 第一发
		{
			if (open == 0)
			{
				goal = 2750;
			}
			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 53);	// 一号夹爪夹紧
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 288); // 二号夹爪夹紧
			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 126); // 三号夹爪夹紧
		}
		if (status > 0 && status % 4 == 2) // 第二发
		{
			if (open == 0)
			{
				goal = 1340;
			}
			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 100); // 一号夹爪松开
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 288); // 二号夹爪夹紧
			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 126); // 三号夹爪夹紧
		}
		if (status > 0 && status % 4 == 3) // 第三发
		{
			if (open == 0)
			{
				goal = 4090;
			}
			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 53);	// 一号夹爪夹紧
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 360); // 二号夹爪松开
			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 126); // 三号夹爪夹紧
		}
		if (status > 0 && status % 4 == 0) // 第四发
		{
			if (open == 0)
			{
				goal = 6820;
			}
			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 53);	// 一号夹爪夹紧
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 288); // 二号夹爪夹紧
			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 170); // 三号夹爪松开
		}
	}
}
/**
 * @brief 装填主控任务
 * @parma None
 * @return None
 */
void tskLoaderMotor(void *arg)
{
	/*	pre load for task	*/
	Motor_CAN_COB Tx_Buff = {};
	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();
	anglepid.SetPIDParam(4, 0, 0.1, 100, 1200);
	speedpid.SetPIDParam(8, 1, 0, 100, 16000);
	for (;;)
	{
		vTaskDelayUntil(&xLastWakeTime_t, 1);
		turn1(goal);
		motor_dji::MotorMsgPack(Tx_Buff, loadermotor[0]);
		xQueueSend(CAN2_TxPort, &Tx_Buff.Id2ff, 0);
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
/**
 * @brief  力传感器读取
 * @param  None.
 * @return None.
 */
#define CLK_Pin GPIO_PIN_0
#define CLK_GPIO_Port GPIOB
#define DOUT_Pin GPIO_PIN_1
#define DOUT_GPIO_Port GPIOB
#define SET_CLK(x) HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, (GPIO_PinState)x)
#define READ_DOUT HAL_GPIO_ReadPin(DOUT_GPIO_Port, DOUT_Pin)
int32_t debug_data;
int32_t raw_data;
float force_t;
float force;
float Force_k = (9.8 / 10.6) * 0.2286974737747255f;
float Force_b = (-143.6) + 1910.212986861924f;
float lastforce;
void HX711_Read(void *arg)
{
	uint32_t count;
	SET_CLK(0); // 拉低时钟引脚
	delay_us_nos(2);
	for (;;)
	{
		count = 0;
		while (READ_DOUT != 0)
		{
			vTaskDelay(1);
		}

		taskDISABLE_INTERRUPTS(); // 关闭中断，若使用中断关闭，请确保SRML定时器的中断不受FreeRTOS管辖
		for (int i = 0; i < 24; i++)
		{
			SET_CLK(1);
			count = count << 1;
			delay_us_nos(3);

			if (READ_DOUT == GPIO_PIN_SET)
				count++;

			SET_CLK(0);
			delay_us_nos(3);
		}
		taskENABLE_INTERRUPTS();

		SET_CLK(1);
		debug_data = count;
		count = count ^ 0x800000; // 第25个脉冲信号到来，进行数据转换
															// 获得24位的数据，对0x800000异或相当于把最高位取反。
															// 把符号位当做有效位，防止突然出现负值波动
		delay_us_nos(2);
		SET_CLK(0);
		raw_data = count;
		force = (Force_k * 0.001f * count) - (Force_b);
		vTaskDelay(1);
	}
}
