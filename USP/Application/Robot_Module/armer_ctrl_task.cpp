/* Robot_Module/armer_ctrl_task.cpp */

#include "internal.h"
#include "global_data.h"
#include "robot_config.h"
#include "can_comm_protocal.h"  
#include "ws2812_ctrl_driver.h"

typedef enum {
    LIGHT_EFFECT_OFF = 0,          // 全灭
    LIGHT_EFFECT_AIMING,           // 待击打瞄准态
    LIGHT_EFFECT_SMALL_HIT,        // 小符击中后
    LIGHT_EFFECT_BIG_STAGE,        // 大符阶段/非待击打灯臂阶段态
    LIGHT_EFFECT_SUCCESS,          // 激活成功
    LIGHT_EFFECT_TEST_SINGLE,   // 5: 单发覆盖测试
    LIGHT_EFFECT_TEST_ACCUM,    // 6: 累积点亮测试
} LightEffectId_t;

static LightEffectId_t arm_light_effect[5] = {LIGHT_EFFECT_OFF}; // 记录每个装甲板当前的灯效状态，0表示全灭

//由于灯板id范围是1~5,而随机目标也是1~5,和数组索引0~4不统一。但是为了不想改随机目标的生成逻辑，所以arm_light_effect数组的索引0~4分别对应装甲板1~5
static uint8_t getArmIndex(uint8_t arm_id) {
    if(arm_id >= 1 && arm_id <= 5) {
        return arm_id - 1; // 转换为0~4的索引
    }
    else{
        while(1); // 错误处理，for debug,用于排查不应该出现的id
        return 0; // 无效id
    }
}

static void ctrl_all_armors(LightEffectId_t cmd) {
    for(uint8_t i = 0; i < 5; i++) {
        arm_light_effect[i] = cmd;
    }
}

void all_off_effect() {
    ctrl_all_armors(LIGHT_EFFECT_OFF);
}

void all_on_effect() {
    ctrl_all_armors(LIGHT_EFFECT_SUCCESS);
}

void se_select_effect(uint8_t arm_id) {
    arm_light_effect[getArmIndex(arm_id)] = LIGHT_EFFECT_AIMING;
}


void se_hit_effect(uint8_t arm_id) {
    arm_light_effect[getArmIndex(arm_id)] = LIGHT_EFFECT_SMALL_HIT;
}

void be_select_effect(uint8_t arm_id) {
    arm_light_effect[getArmIndex(arm_id)] = LIGHT_EFFECT_AIMING;
}

void be_stage_effect(uint8_t arm_id) {
    arm_light_effect[getArmIndex(arm_id)] = LIGHT_EFFECT_BIG_STAGE;
}

void be_hit_effect(uint8_t arm_id) {
    arm_light_effect[getArmIndex(arm_id)] = LIGHT_EFFECT_SUCCESS;
}


void test_light_effect(uint8_t effect[5]) {
    // 用于测试的单发覆盖灯效，覆盖所有状态，优先级最高
    for(uint8_t i = 0; i < 5; i++) {
        arm_light_effect[i] = (LightEffectId_t)effect[i];
    }
}

uint16_t test_arm_send_delay=0;

void task_Rlight_armer(void *arg)
{
    /* Pre-Load for task */
    TickType_t xLastWakeTime_t;
    xLastWakeTime_t = xTaskGetTickCount();
    bool effect_changed = false; // 标志位，指示灯效或相关状态是否发生变化需要更新
    EnergySystemMode_t t_SysMode = g_SystemState.SysMode;
    light_color_enum t_color = g_TargetCtrl.TargetColor;
    uint8_t t_stage = g_SystemState.BE_StateData.BE_Group; // 当前阶段/组数，0-5
    LightEffectId_t last_effect[5] = {LIGHT_EFFECT_OFF}; // 记录上一次发送的灯效状态，初始为全灭
    light_color_enum last_color = color_off;
    uint8_t last_stage = 0;
    for (;;)
    {
        /* wait for next circle */
        vTaskDelayUntil(&xLastWakeTime_t, 10);

        //获取全局状态
        t_SysMode = g_SystemState.SysMode;
        t_color = g_TargetCtrl.TargetColor;
        t_stage = g_SystemState.BE_StateData.BE_Group; // 当前阶段/组数，0-5

        //根据颜色变化更新R标灯效
        if(t_color != last_color) {
            last_color = t_color;
            R_light(last_color);
            effect_changed = true;
        }
        
        //根据灯效变化和颜色以及阶段变化更新装甲板灯效
        if(memcmp(arm_light_effect, last_effect, sizeof(arm_light_effect)) != 0) {
            memcpy(last_effect, arm_light_effect, sizeof(arm_light_effect));
            effect_changed = true;
        }
        
        if(t_stage != last_stage) {
            last_stage = t_stage;
            effect_changed = true;
        }

        if(effect_changed) {
            for (uint8_t i = 0; i < 4; i++) {
                SendFanPacket(i+1,last_effect[i],t_color, t_stage);
				// for(uint8_t j=0; j<test_arm_send_delay; j++) {
                //     __NOP(); // 小延时，确保消息发送出去，具体时长根据总线负载情况调整
                // }
                vTaskDelay(1); // 每发一个包延时1ms，避免总线拥堵
            }
			// for(uint8_t i=0; i<100; i++) {
            //     __NOP(); // 小延时，确保消息发送出去，具体时长根据总线负载情况调整
            // }
            // SendFanPacket(5,last_effect[4],t_color, t_stage);
            effect_changed = false; // 重置标志位
        }
        

        #ifdef INCLUDE_uxTaskGetStackHighWaterMark
        //Stack_Remain.Armer_Ctrl_stack_remain = uxTaskGetStackHighWaterMark(NULL);
        #endif
    }
}