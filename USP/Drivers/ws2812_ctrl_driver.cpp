#include "ws2812_ctrl_driver.h"

//30 + Num * 3 * 8 + 30
#define WS2312_LED_NUM 50   // R标灯的 WS2812 LED 数量

#define PWM_DATA_LEN (WS2312_LED_NUM * 24) // 每个WS2812需要24个码元
#define WS2812_RESET_LEN 40 // 定义重置周期数（800KHz 下，1.25us/bit，40个0约 50us）
#define dma_data_len (PWM_DATA_LEN + WS2812_RESET_LEN) // DMA数据总长度

#define WS2312_0bit 38
#define WS2312_1bit 66

static uint32_t tim_pwm_dma_buff[dma_data_len] = {0};//PWM DMA数据缓存

void Buff_translate(uint8_t* color_buff,uint32_t* dma_row_ptr) //颜色数组转换为码元数组
{   
    uint32_t dat_idx = 0;
	for(uint32_t i = 0;i < (WS2312_LED_NUM*3);i++)
	{
        for(int8_t k = 7; k >= 0; k--) // MSB First: 高位先发
        {
            if ((color_buff[i] >> k) & 0x01) {
                dma_row_ptr[dat_idx++] = WS2312_1bit; 
            } else {
                dma_row_ptr[dat_idx++] = WS2312_0bit; 
            }
        }
	}
}


// DMA 完成回调函数
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    // 判定是哪个定时器触发的
    if(htim==arm_tim1) {
        // 传输完成后立即停止 DMA
        // 停止顺序：先停通道，如果有必要可以手动把 CCR 清零
        HAL_TIM_PWM_Stop_DMA(htim, arm_channel_1);
        // 强制清零 CCR，防止停止瞬间引脚保持高电平
        __HAL_TIM_SET_COMPARE(htim, arm_channel_1, 0);
    }
}         

// 灯臂填充指定颜色
/*
5*10,R标灯的 WS2812 灯珠排列如下（共50颗）：
9  10 29 30 49
8  11 28 31 48
7  12 27 32 47
6  13 26 33 46 
5  14 25 34 45
4  15 24 35 44
3  16 23 36 43
2  17 22 37 42
1  18 21 38 41
0  19 20 39 40
*  *  *    
*        *     
*        *    
*        *
*     *
*  *
*  *  
*     *
*        *   
*           *
*/
#define ws2312_map_len 22
uint8_t ws2312_map[ws2312_map_len] = {0,1,2,3,4,5,6,7,8,9,10,15,16,29,25,22,33,31,32,33,38,40};
void ws2312_show(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t temp_pixels[WS2312_LED_NUM * 3] = {0};


    for (int i = 0; i < ws2312_map_len; i++) 
    {
        temp_pixels[ws2312_map[i] * 3]     = g; // WS2812 典型为 GRB 顺序
        temp_pixels[ws2312_map[i] * 3 + 1] = r;
        temp_pixels[ws2312_map[i] * 3 + 2] = b;
    }
    // 将 RGB 数据转换为 PWM 码元
    Buff_translate(temp_pixels, tim_pwm_dma_buff);
    // 4. 非阻塞启动 4 路 DMA 传输
    HAL_TIM_PWM_Start_DMA(arm_tim1, TIM_CHANNEL_3, (uint32_t *)tim_pwm_dma_buff, dma_data_len);//主灯臂outside
    // __HAL_TIM_SET_COMPARE(arm_tim1,TIM_CHANNEL_4,50);//测试用
    // HAL_TIM_PWM_Start(arm_tim1,TIM_CHANNEL_4);
}



void R_light(light_color_enum color){
    switch(color){
        case color_red:
            ws2312_show(255, 0, 0); // 红色
            break;
        case color_blue:
            ws2312_show(0, 0, 255); // 蓝色
            break;
        case color_off:
        default:
            ws2312_show(0, 0, 0); // 关灯
            break;
    }
}

#define MATRIX_WIDTH  5
#define MATRIX_HEIGHT 10
#define TOTAL_LEDS    50

// 定义 R 标的逻辑位图 (1:亮, 0:灭)
// 基于你提供的 ASCII 图案整理，坐标系：y=0在底，y=9在顶
// x=0 在左，x=4 在右
const uint8_t R_Bitmap[MATRIX_HEIGHT][MATRIX_WIDTH] = {
    // x: 0  1  2  3  4
    {1, 0, 0, 0, 1}, // y=0 (底)
    {1, 0, 0, 1, 0}, // y=1
    {1, 0, 1, 0, 0}, // y=2
    {1, 1, 0, 0, 0}, // y=3
    {1, 1, 0, 0, 0}, // y=4
    {1, 0, 1, 0, 0}, // y=5
    {1, 0, 0, 1, 0}, // y=6
    {1, 0, 0, 1, 0}, // y=7
    {1, 0, 0, 1, 0}, // y=8
    {1, 1, 1, 0, 0}  // y=9 (顶)
};

// 辅助函数：将 (x,y) 坐标转换为 WS2812 的物理索引 (0-49)
// 考虑了贪吃蛇(S形)走线
int Get_LED_Index(int x, int y) {
    if (x < 0 || x >= MATRIX_WIDTH || y < 0 || y >= MATRIX_HEIGHT) return -1;
    
    // 偶数列(0,2,4)是从下往上 (0->9)
    if (x % 2 == 0) {
        return x * 10 + y;
    } 
    // 奇数列(1,3)是从上往下 (19->10)
    else {
        return x * 10 + (9 - y);
    }
}

// 核心重力跟随显示函数
// input_angle: 陀螺仪角度（单位：度）。通常传入 Roll 或 Pitch
void ws2312_show_gravity(float input_angle, uint8_t r, uint8_t g, uint8_t b) 
{
    uint8_t temp_pixels[TOTAL_LEDS * 3];
    // 清空缓冲区 (全黑)
    memset(temp_pixels, 0, sizeof(temp_pixels));

    // 旋转中心 (2.0, 4.5) 是 5x10 矩阵的几何中心
    float cx = 2.0f;
    float cy = 4.5f;

    // 将角度转换为弧度
    // 注意：为了抵消板子的倾斜，我们需要反向旋转图像，所以是 -input_angle
    float rad = -input_angle * (PI / 180.0f);
    float cos_a = cosf(rad);
    float sin_a = sinf(rad);

    // 遍历物理灯板上的每一个像素
    for (int phys_x = 0; phys_x < MATRIX_WIDTH; phys_x++) {
        for (int phys_y = 0; phys_y < MATRIX_HEIGHT; phys_y++) {
            
            // 1. 将物理坐标变换到相对于中心的坐标
            float dx = phys_x - cx;
            float dy = phys_y - cy;

            // 2. 逆向旋转坐标 (寻找该物理像素对应原始图像的哪个位置)
            // 旋转公式: 
            // x' = x*cos - y*sin
            // y' = x*sin + y*cos
            float src_x_f = dx * cos_a - dy * sin_a + cx;
            float src_y_f = dx * sin_a + dy * cos_a + cy;

            // 3. 四舍五入取整，找到最近的原始像素
            int src_x = (int)roundf(src_x_f);
            int src_y = (int)roundf(src_y_f);

            // 4. 检查变换后的坐标是否在 R_Bitmap 范围内且该点需要亮起
            if (src_x >= 0 && src_x < MATRIX_WIDTH && 
                src_y >= 0 && src_y < MATRIX_HEIGHT) 
            {
                if (R_Bitmap[src_y][src_x] == 1) {
                    // 获取物理索引
                    int idx = Get_LED_Index(phys_x, phys_y);
                    if (idx != -1) {
                        temp_pixels[idx * 3]     = g; // GRB
                        temp_pixels[idx * 3 + 1] = r;
                        temp_pixels[idx * 3 + 2] = b;
                    }
                }
            }
        }
    }

    // 硬件发送部分
    Buff_translate(temp_pixels, tim_pwm_dma_buff);
    HAL_TIM_PWM_Start_DMA(arm_tim1, TIM_CHANNEL_3, (uint32_t *)tim_pwm_dma_buff, dma_data_len);
}

// 封装后的控制函数
void R_light_Follow(float angle_deg, light_color_enum color) {
    switch(color){
        case color_red:
            ws2312_show_gravity(angle_deg, 255, 0, 0);
            break;
        case color_blue:
            ws2312_show_gravity(angle_deg, 0, 0, 255);
            break;
        case color_off:
        default:
            ws2312_show_gravity(0, 0, 0, 0);
            break;
    }
}