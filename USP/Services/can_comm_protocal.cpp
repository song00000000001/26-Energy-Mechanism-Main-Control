#include "can_comm_protocal.h"
#include "global_data.h"

#include <stdarg.h>
#include <stdio.h>

/*todo
song
分控协议有变动,处理接收控制的函数如下:
void can_receive_process(uint8_t *light_effect_id, uint8_t *color, uint8_t *active_groups){
    if(comm_buffers.can_rx_complete)
    {
        comm_buffers.can_rx_complete=false;
        if (comm_buffers.CAN_RxMsg.ID == (CAN_RECEIVE_ID_BASE+sub_ctrl_id) && comm_buffers.CAN_RxMsg.DLC == can_rx_dlc) {
            *light_effect_id = comm_buffers.CAN_RxMsg.Data[0];
            *color = comm_buffers.CAN_RxMsg.Data[1];
            *active_groups = comm_buffers.CAN_RxMsg.Data[2];
        }
    }
}
分控控制简化为发射一个灯效id,一个颜色,一个大符组数的包即可.
经过规则调查,灯效id简化为5种:
//灯效枚举
typedef enum {
    LIGHT_EFFECT_OFF = 0,          // 全灭
    LIGHT_EFFECT_AIMING,           // 待击打瞄准态
    LIGHT_EFFECT_SMALL_HIT,        // 小符击中后
    LIGHT_EFFECT_BIG_STAGE,        // 大符阶段/非待击打灯臂阶段态
    LIGHT_EFFECT_SUCCESS,          // 激活成功
} LightEffectId_t;
颜色简化为:
//颜色枚举
typedef enum {
    COLOR_OFF = 0,                // 无色/全灭
    COLOR_RED,                    // 红色
    COLOR_BLUE,                   // 蓝色
} Color_t;
*/
//灯效枚举
typedef enum {
    LIGHT_EFFECT_OFF = 0,          // 全灭
    LIGHT_EFFECT_AIMING,           // 待击打瞄准态
    LIGHT_EFFECT_SMALL_HIT,        // 小符击中后
    LIGHT_EFFECT_BIG_STAGE,        // 大符阶段/非待击打灯臂阶段态
    LIGHT_EFFECT_SUCCESS,          // 激活成功
} LightEffectId_t;

// 发送装甲板控制包
void SendFanPacket(uint8_t id,uint8_t cmd,light_color_enum color, uint8_t stage) {
    CAN_COB CAN_TxMsg = {};
    CAN_TxMsg.IdType = Can_STDID;
    CAN_TxMsg.ID = CAN_SEND_ID_BASE + id; // 分控 ID 作为低字节
    CAN_TxMsg.DLC = 3;
    if(cmd==FAN_CMD_RESET){
        CAN_TxMsg.Data[0] = LIGHT_EFFECT_AIMING; // 灯效
        CAN_TxMsg.Data[1] = static_cast<uint8_t>(color_red); // 熄灭
        CAN_TxMsg.Data[2] = 5;     // 阶段无效
       
    }
    else if(cmd==FAN_CMD_SELECT){
        CAN_TxMsg.Data[0] = LIGHT_EFFECT_AIMING; // 灯效
        CAN_TxMsg.Data[1] = static_cast<uint8_t>(color_red); // 颜色
        CAN_TxMsg.Data[2] = stage;     // 阶段
    }
    else if(cmd==FAN_CMD_HIT){
        if(g_SystemState.SysMode == small_energy) // 小符击打反馈特殊处理，直接发小符击中灯效，颜色和阶段同选定状态
        {
            CAN_TxMsg.Data[0] = LIGHT_EFFECT_SMALL_HIT; // 灯效
            CAN_TxMsg.Data[1] = static_cast<uint8_t>(color); // 颜色
            CAN_TxMsg.Data[2] = stage;     // 阶段
        }
        else // 大符连击阶段反馈同阶段灯效，颜色同选定状态
        {
            CAN_TxMsg.Data[0] = LIGHT_EFFECT_BIG_STAGE; // 灯效
            CAN_TxMsg.Data[1] = static_cast<uint8_t>(color); // 颜色
            CAN_TxMsg.Data[2] = stage;     // 阶段
        }
        CAN_TxMsg.Data[1] = static_cast<uint8_t>(color); // 击打红色
        CAN_TxMsg.Data[2] = stage;     // 阶段
    }
    //xQueueSend(CAN1_TxPort, &CAN_TxMsg, 0);
    xQueueSend(CAN2_TxPort, &CAN_TxMsg, 0);
}

//分控反馈数据处理函数
void FanFeedbackProcess(CAN_COB &CAN_RxMsg)
{
    // 1. 校验数据包
    uint8_t sub_ctrl_id = CAN_RxMsg.ID - CAN_RECEIVE_ID_BASE;
    if (sub_ctrl_id >= 1 && sub_ctrl_id <= 5 && CAN_RxMsg.DLC == 2) {
        //更新全局状态
        g_SystemState.CurrentHitID = sub_ctrl_id;
        g_SystemState.CurrentHitScores = CAN_RxMsg.Data[0];//分控直接回传击打检测到的环数:0~9
    }
}

void my_printf(uint8_t port_num, const char* format, ...)
{  
    
    USART_COB TxMsg;

    va_list args;
    va_start(args, format);
    // 直接格式化到结构体的数组里
    int len = vsnprintf((char*)TxMsg.data, UART1_TX_BUFFER_SIZE, format, args);
    va_end(args);
    if (len > 0 && len < UART1_TX_BUFFER_SIZE)
    {        
        TxMsg.port_num = port_num; // 调试串口是 USART1
        TxMsg.len = len;
		BaseType_t result = xQueueSend(USART_TxPort, &TxMsg, 0); // 0 表示不等待
        if (result != pdPASS) {
			result=0;
        }
    }
    va_end(args);
}

void hit_feedback_to_uart(uint8_t hitID,uint8_t scores){
    my_printf(upper_uart_id, "Hit ID: %d, Scores: %d\r\n", hitID, scores);
}

/*
一方机器人成功激活小能量机关后，该方所有机器人、前 哨站、基地均获得25%的防御增益，持续45秒。
小能量机关增益持续期间内，所有英雄、步兵、空 中机器人在获得经验时，额外获得原经验100%的经验，一方在一次小能量机关增益期间内通过此方 式最多共获得 1200 点额外经验。
*/
void small_enegy_settlement(uint8_t average_round){
    vTaskDelay(80);//蓝牙app默认把80ms消息打包，这里也延时80ms方便看。
    my_printf(upper_uart_id, "SE settlement,total Round: %d\r\n", average_round);
    vTaskDelay(80);
}

/*
## 大能量机关
- 大能量机关的每块装甲模块被划分为1~10环。一方大能量机关被激活后，系统将根据其击中的平均环数和激活灯臂数、为该方所有机器人提供时间不等、效果不同的攻击、防御和热量冷却增益，为该方前哨站、基地提供相应的防御增益，详见“表 5-17 平均环数与对应增益”。同时，大能量机关被激活时，将有750点经验平均分给激活方所有存活的英 雄、步兵、空中机器人。
表 5-17 平均环数与对应增益

| 平均环数区间 | 攻击增益 | 防御增益 | 热量冷却增益 |
| :----: | :--: | :--: | :----: |
| [1,3]  | 150% | 25%  |   无    |
| (3,7]  | 150% | 25%  |   2倍   |
| (7,8]  | 200% | 25%  |   2倍   |
| (8,9]  | 200% | 25%  |   3倍   |
| (9,10] | 300% | 50%  |   5倍   |
表 5-18 激活灯臂数与对应增益持续时间

| 激活灯臂数 | 增益持续时间（秒） |
| :---: | :-------: |
|   5   |    30     |
|   6   |    35     |
|   7   |    40     |
|   8   |    45     |
|   9   |    50     |
|  10   |    60     |
*/
void big_enegy_settlement(uint8_t average_round, uint8_t actived_arms){
    vTaskDelay(80);
    my_printf(upper_uart_id, "BE Settlement,Average Round: %d, Actived Arms: %d\r\n", average_round, actived_arms);
    vTaskDelay(20);
	if(average_round >= 0 && average_round <= 3){
        my_printf(upper_uart_id, "Attack Gain: 150%%, Defense Gain: 25%%, Heat Cooling Gain: None\r\n");
    }
    else if(average_round > 3 && average_round <= 7){
        my_printf(upper_uart_id, "Attack Gain: 150%%, Defense Gain: 25%%, Heat Cooling Gain: 2x\r\n");
    }
    else if(average_round > 7 && average_round <= 8){
        my_printf(upper_uart_id, "Attack Gain: 200%%, Defense Gain: 25%%, Heat Cooling Gain: 2x\r\n");
    }
    else if(average_round > 8 && average_round <= 9){
        my_printf(upper_uart_id, "Attack Gain: 200%%, Defense Gain: 25%%, Heat Cooling Gain: 3x\r\n");
    }
    else if(average_round > 9 && average_round <= 10){
        my_printf(upper_uart_id, "Attack Gain: 300%%, Defense Gain: 50%%, Heat Cooling Gain: 5x\r\n");
    }
    vTaskDelay(20);
    if(actived_arms >= 5 && actived_arms <= 10){
        uint8_t duration = 30 + (actived_arms-5) * 5; // 根据激活灯臂数计算增益持续时间
        if(actived_arms >= 10) duration = 60; //十组特殊处理
        my_printf(upper_uart_id, "Gain Duration: %d seconds\r\n", duration);
    }
    vTaskDelay(80);
}