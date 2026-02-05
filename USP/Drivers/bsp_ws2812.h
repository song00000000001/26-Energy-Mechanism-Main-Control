#pragma once

#include "tim.h"			   
#include "internal.h"

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);	//DMA回调函数
void ws2312_show(uint8_t r, uint8_t g, uint8_t b); // 灯臂全部填充指定颜色