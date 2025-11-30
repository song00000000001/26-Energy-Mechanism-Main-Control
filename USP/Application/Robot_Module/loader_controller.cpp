#include "internal.h"
#include "global_data.h"

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
