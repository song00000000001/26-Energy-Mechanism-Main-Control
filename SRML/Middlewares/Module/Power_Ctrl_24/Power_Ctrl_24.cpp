#include "srml_config.h"

#if USE_SRML_POW_CTRL_24
#include "Power_Ctrl_24.h"

__weak void PowerCtrl::distribute_steer_power(float total_power_limit, float &wheel_power_limit, float &steer_power_limit)
{
    // 目前是随便写的
    wheel_power_limit = total_power_limit;
    steer_power_limit = 0.3f * total_power_limit;
}

__weak float PowerCtrl::m6020_volatage_to_current(float volatage, float current_speed)
{
    return volatage * 0.75f - 55 * current_speed;
}

/**
 * @brief 估算电机功率
 * 
 * @param MODEL_PARAM 功率模型参数 
 * @param torque_current 电机力矩电流，单位通常为A或Nm，单位需要与模型拟合单位相同
 * @param speed 电机速度，单位需要与模型拟合单位相同
 * @return float 估算的电机功率，单位为W
 */
float PowerCtrl::get_motor_power(const power_model_param_t &MODEL_PARAM, float torque_current, float speed)
{
    // 用引用的形式简化符号，不会增加开销
    const float &k1 = MODEL_PARAM.k1;
    const float &k2 = MODEL_PARAM.k2;
    const float &k3 = MODEL_PARAM.k3;
    const float &I = torque_current;
    const float &w = speed;
    return k1 * w * I + k2 * I * I + k3 * w * w;
}

/**
 * @brief 反算电机的电流力矩电流限制
 * 
 * @param MODEL_PARAM 功率模型参数 
 * @param limit_power 限制功率，单位为W
 * @param current_speed 电机速度，单位需要与模型拟合单位相同
 * @return float 力矩电流限制，单位通常为A或Nm，单位需要与模型拟合单位相同
 */
float PowerCtrl::get_motor_current_limit(const power_model_param_t &MODEL_PARAM, float limit_power, float current_speed)
{
    float a = MODEL_PARAM.k2;
    float b = MODEL_PARAM.k1 * current_speed;
    float c = MODEL_PARAM.k3 * powf(current_speed, 2) - limit_power;
    float delta = b * b - 4 * a * c;
    if (delta < b * b)
        delta = b * b;
    return (-b + sqrtf(delta)) / (2 * a);
}
#endif /* USE_SRML_POW_CTRL_24 */