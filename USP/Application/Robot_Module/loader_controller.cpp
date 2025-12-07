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
    myPID load_pid_ang;									// 装填电机的编码器环
    myPID load_pid_spd;									// 装填电机的速度环
	xLastWakeTime_t = xTaskGetTickCount();
	load_pid_ang.SetPIDParam(10, 0, 0, 100, 1200);
	load_pid_spd.SetPIDParam(50, 1, 0, 1000, 16000);

    load_pid_ang.Target = 120+5;    //安全位置
    for (;;)
	{
		vTaskDelayUntil(&xLastWakeTime_t, 1);
        if (DR16.GetStatus() == DR16_ESTABLISHED){
             //计算装填电机pid
            load_pid_ang.Current = loadermotor.getMotorTotalAngle();//换成getTotalAngle试试,测试后发现,total才是绝对,直接get的会有跳变
            load_pid_ang.Adjust();

            load_pid_spd.Target = load_pid_ang.Out;
            load_pid_spd.Current = loadermotor.getMotorSpeed();
            load_pid_spd.Adjust();

            //设置电机输出
            loadermotor.setMotorCurrentOut(load_pid_spd.Out);
        }
        else{
            loadermotor.setMotorCurrentOut(0);
            load_pid_ang.clean_intergral();
            load_pid_spd.clean_intergral();
        }

        //发送CAN报文
        #if 0
        //测试电机gm6020-id1，用此命令
        MotorMsgPack(Tx_Buff, loadermotor);
        xQueueSend(CAN2_TxPort, &Tx_Buff.Id1ff, 0);   //注意，输出指令有误
        #elif 0
        //装填电机gm6020-id2，用此命令 
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
