#include "internal.h"
#include "global_data.h"
#include "robot_config.h"
#include "tim.h"

/**
 * @brief 装填主控任务
 * @parma None
 * @return None
 */

void Loader_Ctrl(void *arg)
{
	/*	pre load for task	*/
	Motor_CAN_COB Tx_Buff = {};
	TickType_t xLastWakeTime_t;
    myPID anglepid;									// 装填电机的编码器环
    myPID speedpid;									// 装填电机的速度环
	xLastWakeTime_t = xTaskGetTickCount();
	anglepid.SetPIDParam(10, 0, 0, 100, 1200);
	speedpid.SetPIDParam(50, 1, 0, 1000, 16000);
    static uint16_t goal=0;										// 装填电机的目标值
    static uint16_t dart_count = 1;						      // 装填状态（1~4分别对应4发飞镖）
	for (;;)
	{
		vTaskDelayUntil(&xLastWakeTime_t, 1);
        //if(robot.status)
        //如果状态为失联或者自检,则disable
        //如果状态为自动发射,则依次控制,并且根据误差,推进发射流程

        if (dart_count % 4 == 0) // 第一发
        {
            goal = 120;//安全位置,后续可以细化流程,优化成顺序转到指定位置,而不是每次都跑到同一个地方,可能花时间。
            Loader_Clamps_ClampAll();
        }
        else if (dart_count % 4 == 1) // 第二发
        {
            goal = 60;//第二发镖位置
            Loader_Clamps_Release1();
        }
        else if (dart_count % 4 == 2) // 第三发
        {
            goal = 180;
            Loader_Clamps_Release2();
        }
        else if (dart_count % 4 == 3) // 第四发
        {
            goal = 300;
            Loader_Clamps_Release3();
        }
        else{//意外情况,复位
            goal = 120;//安全位置,后续可以细化流程,优化成顺序转到指定位置,而不是每次都跑到同一个地方,可能花时间。
            Loader_Clamps_ClampAll();
        }
        
        //计算装填电机pid
        /*todo
        song
        注意，这里注释了target，到时候要先调一遍角度环看效果，然后在让发射状态机接管
        */
        //anglepid.Target = goal;
        anglepid.Current = loadermotor.getMotorTotalAngle();//换成getTotalAngle试试,测试后发现,total才是绝对,直接get的会有跳变
        anglepid.Adjust();

        speedpid.Target = anglepid.Out;
        speedpid.Current = loadermotor.getMotorSpeed();
        speedpid.Adjust();

        //设置电机输出
        loadermotor.setMotorCurrentOut(speedpid.Out);
        
        //发送CAN报文
        MotorMsgPack(Tx_Buff, loadermotor);
        //xQueueSend(CAN2_TxPort, &Tx_Buff.Id1ff, 0);   测试电机id1，用此命令
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
	
}
