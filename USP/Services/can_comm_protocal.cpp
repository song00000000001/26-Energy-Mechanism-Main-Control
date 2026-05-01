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
*/
CAN_COB CAN_TxMsg[5] = {};
// 发送装甲板控制包
void SendFanPacket(uint8_t id,uint8_t cmd,light_color_enum color, uint8_t stage) {
    if(id < 1 || id > 5) {
        //while(1); // id范围检查，for debug,用于排查不应该出现的id
        return; // id范围检查，确保在1~5
    }
   
    CAN_TxMsg[id - 1].IdType = Can_STDID;
    CAN_TxMsg[id - 1].ID = CAN_SEND_ID_BASE + id; // 分控 ID 作为低字节
    CAN_TxMsg[id - 1].DLC = 3;
    CAN_TxMsg[id - 1].Data[0] = cmd; // 命令类型
    CAN_TxMsg[id - 1].Data[1] = static_cast<uint8_t>(color); // 颜色
    CAN_TxMsg[id - 1].Data[2] = stage;     // 阶段
    //xQueueSend(CAN1_TxPort, &CAN_TxMsg, 0);
    xQueueSend(CAN2_TxPort, &CAN_TxMsg[id - 1], 0);
}

//分控反馈数据处理函数
void FanFeedbackProcess(CAN_COB &CAN_RxMsg)
{
    // 1. 校验数据包
    uint8_t sub_ctrl_id = CAN_RxMsg.ID - CAN_RECEIVE_ID_BASE;
    if(sub_ctrl_id < 1 || sub_ctrl_id > 5|| CAN_RxMsg.DLC != 2) 
    {
        return;
    }   

    //更新全局状态
    g_SystemState.CurrentHitID = sub_ctrl_id;
    g_SystemState.CurrentHitScores = CAN_RxMsg.Data[0];//分控直接回传击打检测到的环数:0~9
    

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

void hit_feedback_to_uart(uint8_t targetID,uint8_t hitID,uint8_t scores){
    vTaskDelay(20);//蓝牙app默认把80ms消息打包，这里也延时80ms方便看。
    my_printf(upper_uart_id, "Target/Hit: %d/ %d ;sco: %d\r\n", targetID, hitID, scores);
    vTaskDelay(20);//蓝牙app默认把80ms消息打包，这里也延时80ms方便看。
}

void hit2_feedback_to_uart(uint8_t targetID,uint8_t targetID2,uint8_t hitID,uint8_t scores){
    vTaskDelay(20);//蓝牙app默认把80ms消息打包，这里也延时80ms方便看。
    my_printf(upper_uart_id, "Target/Hit: %d, %d/ %d;sco: %d\r\n", targetID, targetID2, hitID, scores);
    vTaskDelay(20);//蓝牙app默认把80ms消息打包，这里也延时80ms方便看。
}

/*
一方机器人成功激活小能量机关后，该方所有机器人、前 哨站、基地均获得25%的防御增益，持续45秒。
小能量机关增益持续期间内，所有英雄、步兵、空 中机器人在获得经验时，额外获得原经验100%的经验，一方在一次小能量机关增益期间内通过此方 式最多共获得 1200 点额外经验。
*/
void small_enegy_settlement(uint8_t total_round, uint8_t actived_arms){
    vTaskDelay(20);//蓝牙app默认把80ms消息打包，这里也延时80ms方便看。
    uint8_t average_round= total_round / actived_arms; // 计算平均环数
    my_printf(upper_uart_id, "\r\n>>>Hits: %d, Scores: %d, Average: %d\r\n", actived_arms, total_round, average_round);
    vTaskDelay(20);
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
void big_enegy_settlement(uint8_t total_round, uint8_t actived_arms,bool is_lock){
    vTaskDelay(20);
    uint8_t average_round = total_round / actived_arms;
    my_printf(upper_uart_id, "\r\n>>>Arms: %d,Total: %d,Average: %d\r\n", actived_arms, total_round, average_round);
    vTaskDelay(20);
    if(is_lock){
        return;
    }
	if(average_round >= 0 && average_round <= 3){
        my_printf(upper_uart_id, "===AttackGain: 150%%\r\n===DefenseGain: 25%%\r\n===HeatCoolingGain:None\r\n");
    }
    else if(average_round > 3 && average_round <= 7){
        my_printf(upper_uart_id, "===AttackGain: 150%%\r\n===DefenseGain: 25%%\r\n===HeatCoolingGain: 2x\r\n");
    }
    else if(average_round > 7 && average_round <= 8){
        my_printf(upper_uart_id, "===AttackGain: 200%%\r\n===DefenseGain: 25%%\r\n===HeatCoolingGain: 2x\r\n");
    }
    else if(average_round > 8 && average_round <= 9){
        my_printf(upper_uart_id, "===AttackGain: 200%%\r\n===DefenseGain: 25%%\r\n===HeatCoolingGain: 3x\r\n");
    }
    else if(average_round > 9 && average_round <= 10){
        my_printf(upper_uart_id, "===AttackGain: 300%%\r\n===DefenseGain: 50%%\r\n===HeatCoolingGain: 5x\r\n");
    }
    vTaskDelay(20);
    if(actived_arms >= 5 && actived_arms <= 10){
        uint8_t duration = 30 + (actived_arms-5) * 5; // 根据激活灯臂数计算增益持续时间
        if(actived_arms >= 10) duration = 60; //十组特殊处理
        my_printf(upper_uart_id, "===Gain Duration: %d seconds\r\n", duration);
    }
    vTaskDelay(20);
}

void my_upper_ctrl_process(uint8_t *data, uint8_t len){
    if(len != sizeof(UpperCtrlPacket_t)||data == NULL)
    {
        // 长度不匹配或地址为空，可能是错误的包，直接丢弃
        return;
    }
    memcpy(&upper_ctrl_packet, data, sizeof(UpperCtrlPacket_t));
    
    /**
     * 新增上位机控制包，由于当前上位机只有按钮控件，打算用bool型的变量用于切换状态。
     * 目前的按键保留：
     * 停止(a5 00)，准备(a5 01)，小符(a5 02)，大符(a5 03)，连续小符(a5 04)，连续大符(a5 05)
     * 颜色切换:a0 00
     * 电机使能切换:a0 01
     * 大小符取消超时重置切换:a0 02
     * 锁定状态切换:a0 03（开启后击打事件不改变状态和灯效，仅反馈信息，便于视觉测试命中率，打错保留重置惩罚。）
     * 模拟击打：ff xx（xx可以是任意值，表示模拟一次击打事件，供测试用）
        */
    if(upper_ctrl_packet.ctrl_header == 0xA5)
    {
        g_TargetCtrl.target_mode = static_cast<EnergyTargetMode_t>(upper_ctrl_packet.ctrl_content);
    }
    if(upper_ctrl_packet.ctrl_header == 0xA0)
    {
        if(upper_ctrl_packet.ctrl_content == 0x00)
        {
            // 颜色切换
            g_TargetCtrl.UpperCtrlBool.upperctrl_color_toggle = !g_TargetCtrl.UpperCtrlBool.upperctrl_color_toggle; // 切换颜色切换状态
            g_TargetCtrl.TargetColor = (g_TargetCtrl.UpperCtrlBool.upperctrl_color_toggle) ? color_blue : color_red;
        }
        else if(upper_ctrl_packet.ctrl_content == 0x01)
        {
            // 电机使能切换
            g_TargetCtrl.UpperCtrlBool.upperctrl_motor_enable = !g_TargetCtrl.UpperCtrlBool.upperctrl_motor_enable;
        }
        else if(upper_ctrl_packet.ctrl_content == 0x02)
        {
            // 超时重置切换
            g_TargetCtrl.UpperCtrlBool.upperctrl_timeout_reset_enable = !g_TargetCtrl.UpperCtrlBool.upperctrl_timeout_reset_enable;
        }
        else if(upper_ctrl_packet.ctrl_content == 0x03)
        {
            // 锁定状态切换
            g_TargetCtrl.UpperCtrlBool.upperctrl_lock_state_enable = !g_TargetCtrl.UpperCtrlBool.upperctrl_lock_state_enable;
        }
    }
    if(upper_ctrl_packet.ctrl_header == 0xFF)
    {
        Debugger.Debug_simulate_hit=true; // 模拟击打事件
    }

}