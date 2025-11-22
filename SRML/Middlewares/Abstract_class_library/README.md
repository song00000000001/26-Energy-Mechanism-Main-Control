# 抽象类型库  

1.  抽象电机库，用于对接顶层计算得控制量与底层电机的实际输出, 方便将仿真用的理想顶层，对接到实车  
2.  抽象IMU库，用于对接顶层认为定义坐标系，与imu的实际坐标系之间的数据转化方便将仿真用的理想顶层，对接到实车
  
抽象类型的基类已经写好大部分操作，并构建了一个空的模板主类，如：
```
/**
 * @brief IMU抽象模板类，仅仅为了可以特化而写，并无实际作用
 * 模板主类不写实现，是为了避免在传入没有特化过的IMU类型时，会出现无法设想的错误
 * @tparam motorType 
 */
template <class IMUtype>
class abstractIMUClassdef{};
```
只需对模板主类进行模板特化，继承基类，实现基类中的纯虚函数，即可正常使用，如：
```
/**
 * @brief 特化模板类，用于阿路比imu
 */
template <>
class abstractIMUClassdef<LPMS_BE2_Typedef> : public abstractIMU::abstractIMUBaseClassdef
{
private:
    LPMS_BE2_Typedef *lpms = nullptr;
  
    virtual float getACCX() { return lpms->get_data().calAccX; }
    virtual float getACCY() { return lpms->get_data().calAccY; }
    virtual float getACCZ() { return lpms->get_data().calAccZ; }

    virtual float getPitch() { return lpms->get_data().Euler_Pitch; }
    virtual float getYaw() { return lpms->get_data().Euler_Yaw; }
    virtual float getRoll() { return lpms->get_data().Euler_Roll; }

    /* 对于阿路比imu */
    /* pitch绑定caliGyroY */
    /* yaw绑定caliGyroZ */
    /* roll绑定caliGyroX */
    virtual float getPitchVel() { return lpms->get_data().caliGyroY; }
    virtual float getYawVel() { return lpms->get_data().caliGyroZ; }
    virtual float getRollVel() { return lpms->get_data().caliGyroX; }

public:
    void bindIMU(LPMS_BE2_Typedef *lpms)
    {
        this->lpms = lpms;
    }
};
```
**需要对每种类型的电机进行个性化的特化，在添加新的电机特化模板类后，可以通过pull request更新到此库中**

## myMiddlewarew中间层使用电机抽象类示例  
**1、中间层中加入抽象类成员**  
```
class MyMiddlewareClassdef {
  private:
    abstractIMUClassdef<LPMS_BE2_Typedef> imu;
    abstractMotor<MotorHT04Classdef> jointMotor[4]={};
};
```
**2、通过定义初始化函数，传入需要跟抽象类成员绑定的指针**  
```
void MyMiddlewareClassdef::init(QueueHandle_t _USART_TxPort, uint8_t port_num,LPMS_BE2_Typedef *_lmps, MotorHT04Classdef _motor[4])
{
    USART_TxPort = _USART_TxPort;
    Usart_TxCOB.address = this->wheelTargetCurrent;
    Usart_TxCOB.port_num = port_num;
    Usart_TxCOB.len = 4;

    imu.bindIMU(_lmps);
    for (int i = 0; i < 4; i++)
    {
        jointMotor[i].bindMotor(&_motor[i]);
    }
}
```
**3、在构造函数中，配置各个抽象类的极性数据等**  
```
MyMiddlewareClassdef::MyMiddlewareClassdef()
{
    jointMotor[RF].baseAngle = +2.18439484f - 20.f / ratio_rad_to_degree;
    jointMotor[RF].Polarity = 1;

    jointMotor[RB].baseAngle = -2.23394012f + 200.f / ratio_rad_to_degree;
    jointMotor[RB].Polarity = 1;

    jointMotor[LF].baseAngle = 1.86672211f - 20.f / ratio_rad_to_degree;
    jointMotor[LF].Polarity = -1;

    jointMotor[LB].baseAngle = -1.4732666f + 200.f / ratio_rad_to_degree;
    jointMotor[LB].Polarity = -1;

    imu.bindAccCoordinate(abstractIMU::imuWorldAccX, abstractIMU::imuWorldAccY, abstractIMU::imuWorldAccZ);
    imu.bindEulerCoordinate(abstractIMU::imuWorldRoll, abstractIMU::imuWorldYaw, abstractIMU::imuWorldPitch);

    imu.accPolarity.x = 1;
    imu.accPolarity.y = 1;
    imu.accPolarity.z = 1;

    imu.eularPolarity.pitch = 1;
    imu.eularPolarity.roll = 1;
    imu.eularPolarity.yaw = 1;

    imu.eularBaseData.pitch = 0;
    imu.eularBaseData.roll = 0;
    imu.eularBaseData.yaw = 0;
}
```
**4、使用抽象类api获取数据**
```
LinearDataStructdef* MyMiddlewareClassdef::getAccData()
{
    return imu.getAccData();
}

AngularDataStructdef* MyMiddlewareClassdef::getEularData()
{
    return imu.getEularData();
}

AngularDataStructdef* MyMiddlewareClassdef::getAngleVelData()
{
    return imu.getAngleVelData();
}
```
