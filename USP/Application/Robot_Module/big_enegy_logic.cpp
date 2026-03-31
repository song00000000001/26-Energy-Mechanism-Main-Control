#include "global_data.h"
#include "can_comm_protocal.h"


// --- 大能量机关辅助函数 ---
// 生成两个不重复的随机目标(1-5)
void GenerateBETargets() {
    g_SystemState.BE_StateData.BE_Targets[0] = (rand() % 5) + 1;
    do {
        g_SystemState.BE_StateData.BE_Targets[1] = (rand() % 5) + 1;
    } while (g_SystemState.BE_StateData.BE_Targets[1] == g_SystemState.BE_StateData.BE_Targets[0]);
}

// 移除已击打的目标(用于连击窗口判定)
void RemoveTarget(uint8_t id) {
    if (g_SystemState.BE_StateData.BE_Targets[0] == id) g_SystemState.BE_StateData.BE_Targets[0] = 0;
    if (g_SystemState.BE_StateData.BE_Targets[1] == id) g_SystemState.BE_StateData.BE_Targets[1] = 0;
}

void updateBEArmorLight(){
    uint8_t target1 = g_SystemState.BE_StateData.BE_Targets[0];
    uint8_t target2 = g_SystemState.BE_StateData.BE_Targets[1];
    for(int i=1;i<=5;i++){
        if(i == target1 || i == target2){
            be_select_effect(i); // 选中效果
        }
        else{
            be_stage_effect(i); // 瞄准效果
        }
    }
}

void BE_reset() {
    g_SystemState.BE_StateData.BE_Group = 0;
    g_SystemState.BE_StateData.BE_State = BE_GENERATE_TARGET;
    g_SystemState.BE_StateData.BE_Targets[0] = 0;
    g_SystemState.BE_StateData.BE_Targets[1] = 0;
    g_SystemState.CurrentHitID = 0;
    g_SystemState.CurrentHitScores = 0;
    g_SystemState.BE_StateData.BE_ActivedArms = 0;
    g_SystemState.BE_StateData.BE_Scores = 0;
    all_off_effect(); // 熄灭所有装甲板
    vTaskDelay(50); // 确保CAN消息发送出去
    //my_printf(upper_uart_id, "BE reset\n");
}

void big_energy_logic() {

    uint32_t now = xTaskGetTickCount();

    // 状态机处理
    switch (g_SystemState.BE_StateData.BE_State)
    {
    case BE_GENERATE_TARGET: // GENERATE_TARGET
        if (g_SystemState.BE_StateData.BE_Group >= 5) {
            // 全部通关
            big_enegy_settlement(g_SystemState.BE_StateData.BE_Scores, g_SystemState.BE_StateData.BE_ActivedArms); // 结算，平均环数=轮数，激活灯臂数=2
            if(g_TargetCtrl.target_mode == tar_big_energy_continue){
                BE_reset();
            }
            else{
                g_TargetCtrl.target_mode = tar_success; // 结束
            }
        } 
        else {
            GenerateBETargets();
            updateBEArmorLight();
            g_SystemState.BE_StateData.BE_StateTimer = now;
            g_SystemState.BE_StateData.BE_State = BE_WAIT_HIT_1; // 切换到 WAIT_HIT_1
        }
        break;
        
    case BE_WAIT_HIT_1: // WAIT_HIT_1 (第一阶段判定：2.5s)
        // 超时失败
        if (now - g_SystemState.BE_StateData.BE_StateTimer > 2500) {
            BE_reset();
        }
        
        // 击打判定
        if (g_SystemState.CurrentHitID != 0) {
            uint8_t hitID = g_SystemState.CurrentHitID;
            g_SystemState.BE_StateData.BE_Scores+= g_SystemState.CurrentHitScores;
            g_SystemState.BE_StateData.BE_ActivedArms++; // 激活灯臂数加一
            hit2_feedback_to_uart(g_SystemState.BE_StateData.BE_Targets[0], g_SystemState.BE_StateData.BE_Targets[1], hitID, g_SystemState.CurrentHitScores);
            g_SystemState.CurrentHitID = 0; 
            g_SystemState.CurrentHitScores = 0;
            if (g_SystemState.BE_StateData.BE_Targets[0] == hitID || g_SystemState.BE_StateData.BE_Targets[1] == hitID) {
                // 击中其中一个，进入连击窗口
                // 打中的亮大符阶段态，另一个保持瞄准态
                //SendFanPacket(hitID, FAN_CMD_BIG_STAGE, g_TargetCtrl.TargetColor, g_SystemState.BE_StateData.BE_Group);
                RemoveTarget(hitID); // 剩下的是要打的
                updateBEArmorLight();
                // 进入 Stage 2，重置计时器
                g_SystemState.BE_StateData.BE_StateTimer = now;
                g_SystemState.BE_StateData.BE_State = BE_WAIT_HIT_2; 
            } 
            else {
                // 打错
                BE_reset();
            }
        }
        break;

    case BE_WAIT_HIT_2: // WAIT_HIT_2 (第二阶段连击：1s)
        // 超时结束 -> 成功（单杀）
        if (now - g_SystemState.BE_StateData.BE_StateTimer > 1000) {
            g_SystemState.BE_StateData.BE_Group++; // 晋级
            g_SystemState.BE_StateData.BE_State = BE_GENERATE_TARGET; 
        }
        
        // 击打判定
        if (g_SystemState.CurrentHitID != 0) {
            uint8_t hitID = g_SystemState.CurrentHitID;
            g_SystemState.BE_StateData.BE_Scores+= g_SystemState.CurrentHitScores;
            g_SystemState.BE_StateData.BE_ActivedArms++; // 激活灯臂数加一
            hit2_feedback_to_uart(g_SystemState.BE_StateData.BE_Targets[0], g_SystemState.BE_StateData.BE_Targets[1], hitID, g_SystemState.CurrentHitScores);
			g_SystemState.CurrentHitScores = 0;
			g_SystemState.CurrentHitID = 0;
            if (g_SystemState.BE_StateData.BE_Targets[0] == hitID || g_SystemState.BE_StateData.BE_Targets[1] == hitID) {
                // 击中剩下那个 -> 双杀成功
                RemoveTarget(hitID); // 剩下的是要打的
                updateBEArmorLight();
                g_SystemState.BE_StateData.BE_Group++;
                g_SystemState.BE_StateData.BE_State = BE_GENERATE_TARGET;
            } 
            else {
                // 连击阶段打错，也判负
                BE_reset();
            }
        }
        break;
        
    default:
        BE_reset();
        break;
    }
}