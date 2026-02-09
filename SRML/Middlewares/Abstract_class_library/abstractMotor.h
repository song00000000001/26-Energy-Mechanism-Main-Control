/**
 ******************************************************************************
 * Copyright (c) 2023 - ~, SCUT-RobotLab Development Team
 * @file abstractMotor.h
 * @author 余俊晖 (2460857175@qq.com)
 * @brief 抽象电机库，用于对接顶层计算得控制量与底层电机的实际输出
 *        方便将仿真用的理想顶层，对接到实车
 *        目前支持电机：MF9025_v2、HT04、2006（C610）、3508（C620）、GM6020
 * @version 1.0
 * @date 2023-03-04
 *
 ******************************************************************************
 * @attention
 *
 * if you had modified this file, please make sure your code does not have many
 * bugs, update the version Number, write dowm your name and the date, the most
 * important is make sure the users will have clear and definite understanding
 * through your new brief.
 *
 * <h2><center>&copy; Copyright (c) 2023 - ~, SCUT-RobotLab Development Team.
 * All rights reserved.</center></h2>
 ******************************************************************************
 */
#ifndef _ABSTRACTMOTOR_H_
#define _ABSTRACTMOTOR_H_

#ifdef __cplusplus

#include "srml_config.h"
#include "srml_std_lib.h"

#if USE_SRML_MOTOR_DJI
  #include "Drivers/Devices/Motor_Dji/motor_dji.h"
#endif
#if USE_SRML_MF9025_V2
  #include "Drivers/Devices/MF9025_V2/MF9025_V2.h"
#endif
#if USE_SRML_HT04
  #include "Drivers/Devices/HT_04/HT04.h"
#endif
#if USE_SRML_MOTOR_DM
  #include "Drivers/Devices/Motor_DM/motor_dm.h"
#endif
#if USE_SRML_CYBER_GEAR
  #include "Drivers/Devices/Motor_CyberGear/Motor_CyberGear.h"
#endif

#if USE_SRML_WEBOTS
#include <webots/Robot.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Motor.hpp>
typedef webots::Motor Webots_Motor;
typedef webots::PositionSensor Webots_PositionSensor;
#endif

#ifdef USE_AC6
    #define __used_attribute__
#else
    #define __used_attribute__ __attribute__((used))
#endif

/* 匿名命名空间，使基类无法被外部链接 */
namespace
{
    /**
     * @brief 电机抽象基类，用于对接顶层计算得控制量与底层电机的实际输出
     * 在继承类中，加入电机类的指针成员，绑定电机变量，定义几个纯虚函数，即可正常使用
     */
    // template <class motorType>
    template <class motorType>
    class abstractMotorBase
    {
    protected:
        virtual float getRawMotorTotalAngle() = 0; 
        virtual float getRawMotorAngle() = 0;      
        virtual float getRawMotorSpeed() = 0;
        virtual void setRawMotorCurrentOut(float out) = 0;

    public:

    #if USE_SRML_WEBOTS == 0
        template <typename... Args>
        abstractMotorBase(Args... arg) : motor(arg...){}
        motorType motor;
    #endif
    
        virtual bool update(uint32_t canRecID, uint8_t data[8]) = 0;

        float getMotorTotalAngle()
        {
            return getRawMotorTotalAngle() * angle_unit_convert * Polarity + baseAngle;
        }

        float getMotorAngle()
        {
            return getRawMotorAngle() * angle_unit_convert * Polarity + baseAngle;
        }

        float getMotorSpeed()
        {
            return getRawMotorSpeed() * speed_unit_convert * Polarity;
        }

        void setMotorCurrentOut(float out)
        {
            setRawMotorCurrentOut(out * out_unit_convert * Polarity);
        }

        // 极性，将电机实际转动方向的正负，对齐到人为设置的方向，自行设置
        // 对于绝大部分电机来说，输出正力矩时，速度增加，角度增加，所以三者极性是具有统一性的
        // 因此，只需要设置一个极性即可
        float Polarity = 1;

        float speed_unit_convert = 1; // 获取数据的单位转换，自行设置

        float angle_unit_convert = 1; // 获取数据的单位转换，自行设置
        float baseAngle = 0;          // 角度基础值，会在极性对齐、单位转换后，再加上这个值

        // 如果需要将顶层下发的力矩，转换为电流，再用电机类下发，就需要设置这个值
        // 如果不需要单位转换，就不用管
        float out_unit_convert = 1;
    };
}

/**
 * @brief 电机抽象模板类，仅仅为了可以特化而写，并无实际作用
 * 用模板是为了不用给所有类型的电机抽象类都设置一个类名
 * 模板主类不写实现，是为了避免在传入没有特化过的电机类型时，会出现无法设想的错误
 * @tparam motorType
 */
template <class motorType>
class abstractMotor
{
    private:
    abstractMotor(){}
};

#if USE_SRML_MF9025_V2
/**
 * @brief MF9025_v2电机抽象类
 */
template <>
class abstractMotor<MotorMF9025v2Classdef> : public abstractMotorBase<MotorMF9025v2Classdef>
{
protected:
    float getRawMotorTotalAngle() override __used_attribute__
    {
        return motor.getData().totalAngleLocal;
    }
    float getRawMotorAngle() override __used_attribute__
    {
        return motor.getData().singleAngle;
    }
    float getRawMotorSpeed() override __used_attribute__
    {
        return motor.getData().speed;
    }
    void setRawMotorCurrentOut(float out) override __used_attribute__
    {
        motor.iqCloseControl_Current(out);
    }

public:
    template <typename... Args>
    abstractMotor(Args... arg) : abstractMotorBase(arg...){}
    bool update(uint32_t canRecID, uint8_t data[8]) override
    {
        return motor.update(canRecID, data);
    }
};

/**
 * @warning 注意！虽然将9025与其它电机同时传参进来编译能通过，但有潜在bug风险
 * @todo 
 *  通过模板实现电机类的泛化使用，让编译器来推导电机类型从而使用不同的接口，这种方式本质是极其危险的！
 *  根本解决办法只能摒弃模板，手工检测电机类型，抽象出一层电机代理层
*/
template <class... MotorTypes>
CAN_COB &MotorMsgPack(CAN_COB &motor_msg, abstractMotor<MotorMF9025v2Classdef> &absMotor, MotorTypes &...motors)
{
    motor_msg = MotorMsgPack(motor_msg, absMotor.motor); //TODO 这里有潜在bug风险，根本解决办法只能摒弃模板
    return MotorMsgPack(motor_msg, motors...);
}
#endif /* USE_SRML_MF9025_V2 */

#if USE_SRML_MOTOR_DJI
template <>
class abstractMotor<Motor_C620> : public abstractMotorBase<Motor_C620>
{
protected:
    float getRawMotorTotalAngle() override __used_attribute__
    {
        return motor.getAngle();
    };
    float getRawMotorAngle() override __used_attribute__
    {
        return motor.getEncoder() / 8192.f * 360;
    };
    float getRawMotorSpeed() override __used_attribute__
    {
        return motor.getSpeed();
    };
    void setRawMotorCurrentOut(float out) override __used_attribute__
    {
        motor.Out = out;
    };

public:
    template <typename... Args>
    abstractMotor(Args... arg) : abstractMotorBase(arg...){}
    bool update(uint32_t canRecID, uint8_t data[8]) override
    {
        return motor.update(canRecID, data);
    }
};

template <>
class abstractMotor<Motor_C610> : public abstractMotorBase<Motor_C610>
{
protected:
    float getRawMotorTotalAngle() override __used_attribute__
    {
        return motor.getAngle();
    }
    float getRawMotorAngle() override __used_attribute__
    {
        return motor.getEncoder() / 8192.f * 360;
    }
    float getRawMotorSpeed() override __used_attribute__
    {
        return motor.getSpeed();
    }
    void setRawMotorCurrentOut(float out) override __used_attribute__
    {
        motor.Out = out;
    }

public:
    template <typename... Args>
    abstractMotor(Args... arg) : abstractMotorBase(arg...){}
    bool update(uint32_t canRecID, uint8_t data[8]) override
    {
        return motor.update(canRecID, data);
    }
};

template <>
class abstractMotor<Motor_GM6020> : public abstractMotorBase<Motor_GM6020>
{
protected:
    float getRawMotorTotalAngle() override __used_attribute__
    {
        return motor.getAngle();
    };
    float getRawMotorAngle() override __used_attribute__
    {
        return motor.getEncoder() / 8192.f * 360;
    };
    float getRawMotorSpeed() override __used_attribute__
    {
        return motor.getSpeed();
    };
    void setRawMotorCurrentOut(float out) override __used_attribute__
    {
        motor.setCurrentOut(out);
    };

public:
    template <typename... Args>
    abstractMotor(Args... arg) : abstractMotorBase(arg...){}
    bool update(uint32_t canRecID, uint8_t data[8]) override
    {
        return motor.update(canRecID, data);
    }

    void setMotorVoltageOut(float out)
    {
        motor.Out = (out * out_unit_convert * Polarity);
    }
};

/**
 * @brief 单个抽象电机单个can包发送
 * 
 * @tparam MotorType 
 * @param CanQueueHander 
 * @param absMotor 
 */
template <class MotorType>
void MotorMsgSend(QueueHandle_t CanQueueHander, abstractMotor<MotorType> &absMotor)
{
    // 调用motor_dji.h里的MotorMsgSend
    motor_dji::MotorMsgSend(CanQueueHander, absMotor.motor);
}


/**
 * @brief N个同一类型抽象电机打包成若干个can包（实际高位和低位两个can包）
 * @warning 电机数量不宜过多，否则部分电机数据有可能丢包
 * 
 * @tparam MotorType 
 * @tparam N 
 * @param motor_msg 
 * @return Motor_CAN_COB& 
 */
template <class MotorType, int N>
Motor_CAN_COB &MotorMsgPack(Motor_CAN_COB &motor_msg, abstractMotor<MotorType> (&absMotor)[N])
{
    for (int i = 0; i < N; i++)
    {
        motor_msg = motor_dji::MotorMsgPack(motor_msg, absMotor[i].motor);
    }
    return motor_msg;
}

/**
 * @brief 多个不同类型抽象电机打包成若干个can包
 * @warning 电机数量不宜过多，否则部分电机数据有可能丢包，同一路can总线一次性发送多个can包，有可能出现部分电机数据包被覆盖的情况
 * 
 * @tparam MotorType 
 * @tparam MotorTypes 
 * @param motor_msg 
 * @param absMotor 
 * @param motors 
 * @return Motor_CAN_COB& 
 */
template <class MotorType, class... MotorTypes>
Motor_CAN_COB &MotorMsgPack(Motor_CAN_COB &motor_msg, abstractMotor<MotorType> &absMotor, MotorTypes &...motors)
{
    motor_msg = motor_dji::MotorMsgPack(motor_msg, absMotor.motor);
    return MotorMsgPack(motor_msg, motors...);
}

/**
 * @brief 单个抽象电机打包成一个can包
 * 
 * @tparam MotorType 
 * @param motor_msg 
 * @param absMotor 
 * @return Motor_CAN_COB& 
 */
template <class MotorType>
Motor_CAN_COB &MotorMsgPack(Motor_CAN_COB &motor_msg, abstractMotor<MotorType> &absMotor)
{
    return motor_dji::MotorMsgPack(motor_msg, absMotor.motor);
}
#endif /* USE_SRML_MOTOR_DJI */

#if USE_SRML_HT04
/**
 * @brief HT04电机抽象类
 */
template <>
class abstractMotor<MotorHT04Classdef> : public abstractMotorBase<MotorHT04Classdef>
{
protected:
    float getRawMotorTotalAngle() override __used_attribute__ 
    {
        return 0; 
    }
    float getRawMotorAngle() override __used_attribute__
    {
        return motor.getRecData().position;
    }
    float getRawMotorSpeed() override __used_attribute__
    {
        return motor.getRecData().velocity;
    }
    void setRawMotorCurrentOut(float out) override __used_attribute__
    {
        motor.setTorque(out);
    }

    float getRawMotorTorque()
    {
        return motor.getRecData().torque;
    }

    void setRawMotorAngle(float angle, float kp, float kd)
    {
        motor.setPosition(angle, kp, kd);
    }
    void setRawMotorSpeed(float speed, float kd, float torque)
    {
        motor.setSpeed(speed, kd, torque);
    }

public:
    template <typename... Args>
    abstractMotor(Args... arg) : abstractMotorBase(arg...){}
    bool update(uint32_t canRecID, uint8_t data[8]) override
    {
        return motor.update(canRecID, data);
    }

    void init(QueueHandle_t _CAN_TxPort)
    {
        motor.bindCanQueueHandle(_CAN_TxPort);
        motor.startMotor();
    }

    void setMotorSpeed(float speed, float kd, float torque)
    {
        setRawMotorSpeed(speed / speed_unit_convert / Polarity, kd, torque / out_unit_convert / Polarity);
    }
    void setMotorAngle(float angle, float kp, float kd)
    {
        setRawMotorAngle((angle - baseAngle) / angle_unit_convert / Polarity, kp, kd);
    }

    float getMotorTorque()
    {
        return getRawMotorTorque() * Polarity;
    }

    void setZeroPosition()
    {
        motor.cmd_zero_position();
    }
};
#endif /* USE_SRML_HT04 */

#if USE_SRML_MOTOR_DM
/**
 * @brief 达妙电机抽象类
 */
template <>
class abstractMotor<Motor_DM_classdef> : public abstractMotorBase<Motor_DM_classdef>
{
protected:
    float getRawMotorTotalAngle() override __used_attribute__
    {
        return motor.getRecData().angle;
    }
    float getRawMotorAngle() override __used_attribute__
    {
        return motor.getRecData().position;
    }
    float getRawMotorSpeed() override __used_attribute__
    {
        return motor.getRecData().velocity;
    }
    void setRawMotorCurrentOut(float out) override __used_attribute__
    {
        //motor.setTorque(out);
        motor.Out = out; // 仅存储，等待后续 MotorMsgPack 打包发送
    }

    float getRawMotorTorque()
    {
        return motor.getRecData().torque;
    }

    void setRawMotorAngle(float angle, float kp, float kd)
    {
        motor.setPosition(angle, kp, kd);
    }
    void setRawMotorSpeed(float speed, float kd, float torque)
    {
        motor.setSpeed(speed, kd, torque);
    }

public:
    template <typename... Args>
    abstractMotor(Args... arg) : abstractMotorBase(arg...){}
    void init(QueueHandle_t _CAN_TxPort)
    {
        motor.bindCanQueueHandle(_CAN_TxPort);
        motor.startMotor();
    }

    bool update(uint32_t canRecID, uint8_t data[8]) override __used_attribute__
    {
        return motor.update(canRecID, data);
    }

    void setMotorSpeed(float speed, float kd, float torque)
    {
        setRawMotorSpeed(speed / speed_unit_convert / Polarity, kd, torque / out_unit_convert / Polarity);
    }
    void setMotorAngle(float angle, float kp, float kd)
    {
        setRawMotorAngle((angle - baseAngle) / angle_unit_convert / Polarity, kp, kd);
    }

    float getMotorTorque()
    {
        return getRawMotorTorque() * Polarity;
    }
};
#endif /* USE_SRML_MOTOR_DM */

#if USE_SRML_CYBER_GEAR
template <>
class abstractMotor<Motor_CyberGear_Classdef> : public abstractMotorBase<Motor_CyberGear_Classdef>
{
protected:
    float getRawMotorTotalAngle() override __used_attribute__
    {
        return motor.getRecData().angle;
    }
    float getRawMotorAngle() override __used_attribute__
    {
        return motor.getRecData().position;
    }
    float getRawMotorSpeed() override __used_attribute__
    {
        return motor.getRecData().velocity;
    }
    void setRawMotorCurrentOut(float out) override __used_attribute__
    {
        motor.setTorque(out);
    }

    float getRawMotorTorque()
    {
        return motor.getRecData().torque;
    }

    void setRawMotorAngle(float angle, float kp, float kd)
    {
        motor.setPosition(angle, kp, kd);
    }
    void setRawMotorSpeed(float speed, float kd, float torque)
    {
        motor.setSpeed(speed, kd, torque);
    }

public:
    template <typename... Args>
    abstractMotor(Args... arg) : abstractMotorBase(arg...){}
    void init(QueueHandle_t _CAN_TxPort)
    {
        motor.bindCanQueueHandle(_CAN_TxPort);
    }

    bool update(uint32_t canRecID, uint8_t data[8]) override __used_attribute__
    {
        return motor.update(canRecID, data);
    }

    void setMotorSpeed(float speed, float kd, float torque)
    {
        setRawMotorSpeed(speed / speed_unit_convert / Polarity, kd, torque / out_unit_convert / Polarity);
    }
    void setMotorAngle(float angle, float kp, float kd)
    {
        setRawMotorAngle((angle - baseAngle) / angle_unit_convert / Polarity, kp, kd);
    }

    float getMotorTorque()
    {
        return getRawMotorTorque() * Polarity;
    }
};
#endif /* USE_SRML_CYBER_GEAR */

#if USE_SRML_WEBOTS
/**
 * @brief webots电机抽象类
 */
template <>
class abstractMotor<Webots_Motor> : public abstractMotorBase<Webots_Motor>
{
protected:
    Webots_Motor *motor = nullptr;
    Webots_PositionSensor* motorPositionSensor = nullptr;
    float getRawMotorTotalAngle()
    {
        return motorPositionSensor->getValue();
    }

    float getRawMotorAngle()
    {
        return motorPositionSensor->getValue();
    }

    float speed;
    float last_pos = 0, last_time = 0;
    float getRawMotorSpeed()
    {
        return speed;
    }

    void setRawMotorCurrentOut(float out)
    {
        motor->setTorque(out);
    }

    void setRawMotorAngle(float angle)
    {
        motor->setPosition(angle);
    }

    void setRawMotorSpeed(float speed)
    {
        motor->setVelocity(speed);
    }

    bool isInit()
    {
        return (motor != nullptr && motorPositionSensor != nullptr);
    }

public:
    void bindMotor(Webots_Motor *_motor, Webots_PositionSensor *_positionSensor)
    {
        motor = _motor;
        motorPositionSensor = _positionSensor;
    }

    void update(float time)
    {
        float pos = motorPositionSensor->getValue();
        speed = (pos - last_pos) / (time - last_time);
        last_pos = pos;
        last_time = time;
    }

    void setMotorSpeed(float speed)
    {
        if (isInit() == 0)
            return;
        setRawMotorSpeed(speed / speed_unit_convert / Polarity);
    }

    void setMotorAngle(float angle)
    {
        if (isInit() == 0)
            return;
        setRawMotorAngle((angle - baseAngle) / angle_unit_convert / Polarity);
    }

    bool update(uint32_t canRecID, uint8_t data[8]) {
        return 0;
    }
};
#endif /* USE_SRML_WEBOTS */

#endif /* __cplusplus */

#endif /* _ABSTRACTMOTOR_H_ */
