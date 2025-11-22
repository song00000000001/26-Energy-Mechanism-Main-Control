#include "srml_config.h"

#if USE_SRML_DIGITAL_POWER_V2

#include "Digital_Power_V2.h"

/**
 * @brief 串口接收数字电源数据
 * 
 * @param ptr 存放数据的变量地址
 * @param RecDataPtr 串口数据地址
 * @param RecLength 串口数据长度
 * @return true 通过CRC校验，成功接收数据
 * @return false 数据长度不对or不通过CRC校验，接收数据失败
 */
uint8_t RecFromDigitalPower(SendFromDigitalPowerPack_S* ptr, const void* RecDataPtr, uint8_t RecLength)
{
    if(RecLength >= sizeof(SendFromDigitalPowerPack_S) && std_lib::CRC16(RecDataPtr, RecLength) == 0)
    {
        memcpy(ptr, RecDataPtr, RecLength);
        return 1;
    }
    else
    {
        return 0;
    }
}
#endif /* !USE_SRML_DIGITAL_POWER_V2 */
