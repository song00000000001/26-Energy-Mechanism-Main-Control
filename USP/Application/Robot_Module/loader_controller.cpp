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
	
    static uint8_t yaw_safe_posizion=120+5;                      //yaw轴修正角度,根据实际情况调整

    for (;;)
	{
		vTaskDelayUntil(&xLastWakeTime_t, 1);
        
        //校准时，需要保持为安全位置，和第一发相同
        if(Robot.Status.current_state == SYS_CALIBRATING)
        {
            goal = yaw_safe_posizion;//安全位置,后续可以细化流程,优化成顺序转到指定位置,而不是每次都跑到同一个地方,可能花时间。
            Loader_Clamps_ClampAll();
        }

        if (dart_count % 4 == 0) // 第一发
        {
            goal = yaw_safe_posizion;
            Loader_Clamps_ClampAll();
        }
        else if (dart_count % 4 == 1) // 第二发
        {
            goal = 60;//第二发镖位置
            Loader_Clamps_Release1();
        }
        else if (dart_count % 4 == 2) // 第三发
        {
            goal = 180;//第三发镖位置
            Loader_Clamps_Release2();
        }
        else if (dart_count % 4 == 3) // 第四发
        {
            goal = 300;//第四发镖位置
            Loader_Clamps_Release3();
        }
        else{//意外情况,复位
            goal = yaw_safe_posizion;//安全位置,后续可以细化流程,优化成顺序转到指定位置,而不是每次都跑到同一个地方,可能花时间。
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
        
        //xQueueSend(CAN2_TxPort, &Tx_Buff.Id1ff, 0);   测试电机id1，用此命令
        /*todo
        song
        由于发射状态机复杂性,先利用已验证的任务,测试下各电机是否正常
        MotorMsgPack(Tx_Buff, Launcher.DeliverMotor[L], Launcher.DeliverMotor[R], Launcher.IgniterMotor);
		xQueueSend(CAN2_TxPort, &Tx_Buff.Id200, 0);
        */
        #if 0
            MotorMsgPack(Tx_Buff, loadermotor);
        #else
            MotorMsgPack(Tx_Buff, Launcher.DeliverMotor[L], Launcher.DeliverMotor[R], Launcher.IgniterMotor);
        #endif

        #if 0
        //测试电机gm6020-id1，用此命令
        MotorMsgPack(Tx_Buff, loadermotor);
        xQueueSend(CAN2_TxPort, &Tx_Buff.Id1ff, 0);   
        #elif 1
		
        //装填电机gm6020-id2，用此命令 
        MotorMsgPack(Tx_Buff, loadermotor);
        xQueueSend(CAN2_TxPort, &Tx_Buff.Id2ff, 0); 
        #elif 0
        //发射电机,滑台3508-id4-id3,行程2006-id1，用此命令
        MotorMsgPack(Tx_Buff, Launcher.DeliverMotor[L], Launcher.DeliverMotor[R], Launcher.IgniterMotor);
		xQueueSend(CAN2_TxPort, &Tx_Buff.Id200, 0);
        #elif 0
        //yaw电机gm6020-id2，用此命令
        //MotorMsgPack(Tx_Buff, Yawer.YawMotor);
		MotorMsgPack(Tx_Buff, loadermotor);
        xQueueSend(CAN2_TxPort, &Tx_Buff.Id2ff, 0); 
        #endif
		           
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
