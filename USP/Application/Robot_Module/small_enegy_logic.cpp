#include "global_data.h"
#include "can_comm_protocal.h"

// 小能量机关辅助函数
// 小能量机关比较简单,只是在5个装甲板中随机选择一个点亮,然后在2.5秒内等待击打,如果超时未击中或者打错则重置
// 如果击中则亮起该arm,然后再剩下的中随机选一个,直到完成5轮
// 1. 生成随机目标
void generateSETarget() {
    //参考洗牌算法,生成不重复的1~5的随机序列
    static uint8_t ids[5] = {1, 2, 3, 4, 5};
    for (int i = 4; i > 0; i--) {
        int j = rand() % (i + 1);
        uint8_t temp = ids[i];
        ids[i] = ids[j];
        ids[j] = temp;
    }
    for(int i = 0; i < 5; i++){
        g_SystemState.SE_StateData.SE_TargetID_GROUP[i] = ids[i];
    }
}

void SE_reset() {
    g_SystemState.SE_StateData.SE_Group = 0; // 重置轮数
    g_SystemState.SE_StateData.SE_State = SE_GENERATE_TARGET; // 小能量状态机
    g_SystemState.CurrentHitID = 0;
    g_SystemState.CurrentHitScores = 0;
    g_SystemState.SE_StateData.SE_Scores = 0;
    all_off_effect(); // 熄灭所有装甲板
    vTaskDelay(50); // 确保CAN消息发送出去
    
    //my_printf(upper_uart_id, "SE reset\n");
}

void small_energy_logic() {

    uint32_t now = xTaskGetTickCount();

    // 状态机处理
    switch (g_SystemState.SE_StateData.SE_State)
    {
    case SE_GENERATE_TARGET: // 生成目标
        g_SystemState.SE_StateData.SE_Group = 0; // 重置轮数
        generateSETarget();
        g_SystemState.SE_StateData.SE_StateTimer = now;
        g_SystemState.SE_StateData.SE_State = SE_WAIT_HIT; // 切换到等待击打
        break;
        
    case SE_WAIT_HIT: // 等待击打 (2.5s)
        // 超时失败
        if (now - g_SystemState.SE_StateData.SE_StateTimer > 2500) {
            SE_reset(); // 重置小能量机关状态
			break;
        }
        
        g_SystemState.SE_StateData.SE_TargetID=g_SystemState.SE_StateData.SE_TargetID_GROUP[g_SystemState.SE_StateData.SE_Group];
        se_select_effect(g_SystemState.SE_StateData.SE_TargetID); // 选中效果

        // 击打判定
        if (g_SystemState.CurrentHitID != 0) {
            //获取id
            uint8_t hitID = g_SystemState.CurrentHitID;
            uint8_t hitScores = g_SystemState.CurrentHitScores;
            //清空id和得分，准备下一次击打判定
            g_SystemState.CurrentHitID = 0;
            g_SystemState.CurrentHitScores = 0;
            //累计得分
            g_SystemState.SE_StateData.SE_Scores += hitScores;
            //发送反馈
            hit_feedback_to_uart(g_SystemState.SE_StateData.SE_TargetID,hitID, hitScores);
            // 判断是否击中正确的目标
            if (hitID == g_SystemState.SE_StateData.SE_TargetID) {
                // 击中目标，进入下一轮
                se_hit_effect(hitID); // 击中效果
                g_SystemState.SE_StateData.SE_Group++; // 轮数加一
                if(g_SystemState.SE_StateData.SE_Group > 4) {
                    small_enegy_settlement(g_SystemState.SE_StateData.SE_Scores); // 结算，传入得分
                    g_SystemState.SE_StateData.SE_Scores = 0;
                    // 全部通关
                    if(g_TargetCtrl.target_mode == tar_small_energy_continue){
                        SE_reset(); // 重置状态准备下一次
                    }
                    else{
                        g_TargetCtrl.target_mode = tar_success; // 结束
                    }
                    break;
                }
                g_SystemState.SE_StateData.SE_StateTimer = now;
            } 
            else {
                // 打错，重置
                SE_reset();
                break;
            }
        }
        break;

    default:
        g_SystemState.SE_StateData.SE_State = SE_GENERATE_TARGET;
        break;
    }
}
