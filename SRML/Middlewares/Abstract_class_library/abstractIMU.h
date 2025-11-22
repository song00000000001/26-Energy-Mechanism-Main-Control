 /**
 ******************************************************************************
 * Copyright (c) 2023 - ~, SCUT-RobotLab Development Team
 * @file abstractIMU.h
 * @author 余俊晖 (2460857175@qq.com)
 * @brief 抽象IMU库，用于对接顶层认为定义坐标系，与imu的实际坐标系之间的数据转化
 *        方便将仿真用的理想顶层，对接到实车
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
#ifndef ABSTRACTIMU_H
#define ABSTRACTIMU_H

#include "srml_config.h"

#if USE_SRML_WEBOTS
    #include <webots/Accelerometer.hpp>
    #include <webots/InertialUnit.hpp>
    #include <webots/Gyro.hpp>
    typedef webots::Accelerometer Webots_Acc;
    typedef webots::Gyro Webots_Gyro;
    typedef webots::InertialUnit Webots_IMU;
#endif

#if USE_SRML_MPU6050
  #include "Drivers/Devices/MPU6050/mpu6050.h"
#endif

#if USE_SRML_LPMS_BE2
  #include "Drivers/Devices/Lpms_Be2/LPMS_BE2.h"
#endif

#ifdef __cplusplus
/*线性参数*/
struct LinearDataStructdef
{
    float x;
    float y;
    float z;
};
/*角度参数*/
struct AngularDataStructdef
{
    float pitch;
    float yaw;
    float roll;
};

/**
 * @brief IMU直角坐标系枚举
 *
 */
enum AccCoordinate_t
{
    AccX_of_IMU = 0,
    AccY_of_IMU = 1,
    AccZ_of_IMU = 2
};

/**
 * @brief 角度极性结构体
 *
 */
struct AnglePolarity_t
{
    int8_t pitch = 1;
    int8_t roll = 1;
    int8_t yaw = 1;
};

/**
 * @brief IMU欧拉角坐标系枚举
 *
 */
enum EulerCoordinate_t
{
    Pitch_of_IMU = 0,
    Yaw_of_IMU = 1,
    Roll_of_IMU = 2
};

namespace
{
    /**
     * @brief 线性加速度极性结构体
     *
     */
    struct LinerPolarity_t
    {
        int8_t x = 1;
        int8_t y = 1;
        int8_t z = 1;
    };
    
    /**
     * @brief 加速度坐标对齐结构体
     *
     */
    struct AccCoordinateAlignment_t
    {
        AccCoordinate_t ideal_x = AccX_of_IMU;
        AccCoordinate_t ideal_y = AccY_of_IMU;
        AccCoordinate_t ideal_z = AccZ_of_IMU;
    };
    
    /**
     * @brief 欧拉角坐标对齐结构体
     *
     */
    struct EularCoordinateAlignment_t
    {
        EulerCoordinate_t ideal_Pitch = Pitch_of_IMU;
        EulerCoordinate_t ideal_Yaw = Yaw_of_IMU;
        EulerCoordinate_t ideal_Roll = Roll_of_IMU;
    };
    
    /**
     * @brief IMU抽象类基类，完成了绝大部分的逻辑操作
     * 在继承类中，加入imu类的指针成员，绑定imu变量，定义9个获取imu数据的纯虚函数，即可正常使用
     */
    class abstractIMUBaseClassdef
    {
    protected:
        // 抽象坐标系结构体，用于存储人定坐标系与imu坐标系的对应关系
        AccCoordinateAlignment_t acc_coordinates;
        EularCoordinateAlignment_t euler_coordinates;

        // 获取imu在世界（现实）坐标系下的加速度
        virtual float getACCX() = 0;
        virtual float getACCY() = 0;
        virtual float getACCZ() = 0;

        // 获取imu在世界（现实）坐标系下的欧拉角、及对应的角速度
        virtual float getEularPitch() = 0;
        virtual float getEularYaw() = 0;
        virtual float getEularRoll() = 0;

        virtual float getGyroPitch() = 0;
        virtual float getGyroYaw() = 0;
        virtual float getGyroRoll() = 0;

        virtual bool isInit() = 0; // 检测是否导入了imu指针，防止未导入时，因为指针为空指针，调用函数而进入硬件中断
    public:
        LinearDataStructdef accData;
        AngularDataStructdef eularData, gyroData;

        // 加速度极性，传入1或-1，用于将imu现实的极性对齐到人为定义的坐标系
        LinerPolarity_t accPolarity;
        // 欧拉角极性，传入1或-1，用于将imu现实的极性对齐到人为定义的坐标系
        AnglePolarity_t eularPolarity;
        // 欧拉角基准值，用于校准，会在校准极性后，加上此基准值
        AngularDataStructdef eularBaseData = {0, 0, 0};
        float angle_unit_convert = 1;

        // 绑定加速度坐标系，传入三个枚举值，分别代表x,y,z轴对应的imu坐标系
        // 若传入AccY_of_IMU, AccX_of_IMU, AccZ_of_IMU
        // 则代表将imu的y轴对应到人为定义的x轴，imu的x轴对应到人为定义的y轴，imu的z轴对应到人为定义的z轴
        void bindEulerCoordinate(EulerCoordinate_t _ideal_Pitch, EulerCoordinate_t _ideal_Yaw, EulerCoordinate_t _ideal_Roll)
        {
            euler_coordinates.ideal_Pitch = _ideal_Pitch;
            euler_coordinates.ideal_Yaw = _ideal_Yaw;
            euler_coordinates.ideal_Roll = _ideal_Roll;
        }

        // 绑定加速度坐标系，传入三个枚举值，分别代表pitch, yaw, roll轴对应的imu坐标系
        // 若传入Pitch_of_IMU, Roll_of_IMU, Yaw_of_IMU
        // 则代表将imu的pitch轴对应到人为定义的pitch轴，imu的Roll轴对应到人为定义的yaw轴，imu的Yaw轴对应到人为定义的roll轴
         void bindAccCoordinate(AccCoordinate_t _ideal_x, AccCoordinate_t _ideal_y, AccCoordinate_t _ideal_z)
        {

            acc_coordinates.ideal_x = _ideal_x;
            acc_coordinates.ideal_y = _ideal_y;
            acc_coordinates.ideal_z = _ideal_z;
        }

        // 获取imu世界坐标系数据，转化为人为定义的坐标系、对齐极性后的数据
        // 功能基本实现，继承类不用再写
        void update()
        {
            float tempAccData[3];
            tempAccData[AccX_of_IMU] = this->getACCX();
            tempAccData[AccY_of_IMU] = this->getACCY();
            tempAccData[AccZ_of_IMU] = this->getACCZ();

            accData.x = tempAccData[acc_coordinates.ideal_x] * accPolarity.x * angle_unit_convert;
            accData.y = tempAccData[acc_coordinates.ideal_y] * accPolarity.y * angle_unit_convert;
            accData.z = tempAccData[acc_coordinates.ideal_z] * accPolarity.z * angle_unit_convert;

            float tempGyroData[3];
            tempGyroData[Pitch_of_IMU] = this->getGyroPitch();
            tempGyroData[Yaw_of_IMU] = this->getGyroYaw();
            tempGyroData[Roll_of_IMU] = this->getGyroRoll();

            gyroData.pitch = tempGyroData[euler_coordinates.ideal_Pitch] * angle_unit_convert * eularPolarity.pitch;
            gyroData.yaw = tempGyroData[euler_coordinates.ideal_Yaw] * angle_unit_convert * eularPolarity.yaw;
            gyroData.roll = tempGyroData[euler_coordinates.ideal_Roll] * angle_unit_convert * eularPolarity.roll;

            float tempEularData[3];
            tempEularData[Pitch_of_IMU] = this->getEularPitch();
            tempEularData[Yaw_of_IMU] = this->getEularYaw();
            tempEularData[Roll_of_IMU] = this->getEularRoll();

            eularData.pitch = tempEularData[euler_coordinates.ideal_Pitch] * eularPolarity.pitch * angle_unit_convert + eularBaseData.pitch;
            eularData.yaw = tempEularData[euler_coordinates.ideal_Yaw] * eularPolarity.yaw * angle_unit_convert + eularBaseData.yaw;
            eularData.roll = tempEularData[euler_coordinates.ideal_Roll] * eularPolarity.roll * angle_unit_convert + eularBaseData.roll;
        }
    };
};
/**
 * @brief IMU抽象模板类，仅仅为了可以特化而写，并无实际作用
 * 模板主类不写实现，是为了避免在传入没有特化过的IMU类型时，会出现无法设想的错误
 * @tparam motorType
 */
template <class IMUtype>
class abstractIMUClassdef
{
    private:
    abstractIMUClassdef(){}
};

#if USE_SRML_LPMS_BE2
/**
 * @brief 特化模板类，用于阿路比imu
 */
template <>
class abstractIMUClassdef<LPMS_BE2_Lite_Classdef> : public abstractIMUBaseClassdef
{
private:
    float getACCX()
    {
        return imu->get_data().linearAccX;
    }
    float getACCY()
    {
        return imu->get_data().linearAccY;
    }
    float getACCZ()
    {
        return imu->get_data().linearAccZ;
    }

    float getEularPitch()
    {
        return imu->get_data().Euler_Pitch;
    }
    float getEularYaw()
    {
        return imu->get_data().Euler_Yaw;
    }
    float getEularRoll()
    {
        return imu->get_data().Euler_Roll;
    }

    /* 对于阿路比imu */
    /* pitch绑定caliGyroY */
    /* yaw绑定caliGyroZ */
    /* roll绑定caliGyroX */
    float getGyroPitch()
    {
        return imu->get_data().caliGyroY;
    }
    float getGyroYaw()
    {
        return imu->get_data().caliGyroZ;
    }
    float getGyroRoll()
    {
        return imu->get_data().caliGyroX;
    }

    bool isInit()
    {
        if (imu == nullptr)
            return 0;
        else
            return 1;
    }

public:
    LPMS_BE2_Lite_Classdef *imu = nullptr;
    void processRecData(uint8_t *data)
    {
        if (isInit() == 0)
            return;
        imu->Update(data);
    }
};
#endif  /* USE_SRML_LPMS_BE2 */

#if USE_SRML_MPU6050
/**
 * @brief 特化模板类，用于MPU6050
 */
template <>
class abstractIMUClassdef<mpu_rec_s> : public abstractIMUBaseClassdef
{
private:
    mpu_rec_s *mpu_s = nullptr;

    float getACCX()
    {
        return mpu_s->accel[0];
    }
    float getACCY()
    {
        return mpu_s->accel[1];
    }
    float getACCZ()
    {
        return mpu_s->accel[2];
    }

    float getEularPitch()
    {
        return mpu_s->pitch;
    }
    float getEularYaw()
    {
        return mpu_s->yaw;
    }
    float getEularRoll()
    {
        return mpu_s->roll;
    }

    float getGyroPitch()
    {
        return mpu_s->gyro[1];
    }
    float getGyroYaw()
    {
        return mpu_s->gyro[2];
    }
    float getGyroRoll()
    {
        return mpu_s->gyro[0];
    }

    bool isInit()
    {
        if (mpu_s == nullptr)
            return 0;
        else
            return 1;
    }

public:
    void bindIMU(mpu_rec_s *_mpu_s)
    {
        this->mpu_s = _mpu_s;
    }
    /* 注意，该update需要在任务里定期执行，频率与MPU6050的读取相关 */
    void update()
    {
        if (isInit() == 0)
            return;
        abstractIMUBaseClassdef::update();
    }
};
#endif /* USE_SRML_MPU6050 */

#if USE_SRML_WEBOTS
/**
 * @brief 特化模板类，用于webots
 */
template <>
class abstractIMUClassdef<Webots_IMU> : public abstractIMUBaseClassdef
{
private:
    Webots_Acc* acc = nullptr;
    Webots_Gyro* gyro = nullptr;
    Webots_IMU* imu = nullptr;

    float getACCX()
    {
        return (float)(acc->getValues())[2];
    }
    float getACCY()
    {
        return (float)(acc->getValues())[0];
    }
    float getACCZ()
    {
        return (float)(acc->getValues())[1];
    }

    float getEularPitch()
    {
        return -(float)(imu->getRollPitchYaw())[1];
    }
    float getEularYaw()
    {
        return (float)(imu->getRollPitchYaw())[2];
    }
    float getEularRoll()
    {
        return (float)(imu->getRollPitchYaw())[0];
    }

    float getGyroPitch()
    {
        return (float)(gyro->getValues())[1];
    }
    float getGyroYaw()
    {
        return (float)(gyro->getValues())[2];
    }
    float getGyroRoll()
    {
        return (float)(gyro->getValues())[0];
    }

    bool isInit()
    {
        if (acc == nullptr  ||
            gyro == nullptr ||
            imu == nullptr)
            return 0;
        else
            return 1;
    }

public:
    void bindIMU(Webots_Acc* _acc, Webots_Gyro* _gyro, Webots_IMU* _imu)
    {
        this->acc = _acc;
        this->gyro = _gyro;
        this->imu = _imu;
    }
    void update()
    {
        if (isInit() == 0)
            return;
        abstractIMUBaseClassdef::update();

    }
};
#endif /* USE_WEBOTS */


#endif /* __cplusplus */

#endif /* ABSTRACTIMU_H */
