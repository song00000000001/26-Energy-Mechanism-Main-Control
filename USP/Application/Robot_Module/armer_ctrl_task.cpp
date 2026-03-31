/* Robot_Module/armer_ctrl_task.cpp */

#include "internal.h"
#include "global_data.h"
#include "robot_config.h"
#include "can_comm_protocal.h"  

void armer_ctrl_task(void *arg)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(2);

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
       
        #ifdef INCLUDE_uxTaskGetStackHighWaterMark
        //Stack_Remain.Armer_Ctrl_stack_remain = uxTaskGetStackHighWaterMark(NULL);
        #endif
    }
}


void Ctrl_All_Armors(uint8_t cmd, light_color_enum color, uint8_t stage) {
    for (uint8_t i = 0; i < 5; i++) {
        SendFanPacket(i,cmd,color, stage);
    }
}

// 成功后闪烁重置,num>0正闪亮灭,num<0反闪灭亮
void lightSuccessFlash(int8_t num, light_color_enum color) {
    if(num < 0) {
        num = -num;
        for (uint8_t i = 0; i < num; i++) {
            // 全部熄灭
            //Ctrl_All_Armors(FAN_CMD_RESET, color, 0);
            vTaskDelay(500);
            // 全部点亮
            //Ctrl_All_Armors(FAN_CMD_SUCCESS, color, 5);
            vTaskDelay(500);
        }
    }
    else if(num > 0) {
        for (uint8_t i = 0; i < num; i++) {
            // 全部点亮
            //Ctrl_All_Armors(FAN_CMD_SUCCESS, color, 5);
            vTaskDelay(500);
            // 全部熄灭
            //Ctrl_All_Armors(FAN_CMD_RESET, color, 0);
            vTaskDelay(500);
        }
    }
    else 
        return;
}   