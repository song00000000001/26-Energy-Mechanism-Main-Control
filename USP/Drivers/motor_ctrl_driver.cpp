#include "motor_ctrl_driver.h"
#include "robot_config.h"
#include "global_data.h"

// 调参符号，用于完成单位标定
volatile float deliver_ratio = 52.0f;
volatile float motor_speed_filter=1.0f; // 
// 构造函数
motor_ctrl_driver::motor_ctrl_driver(uint8_t id):
    mymotor(id)
{
    // 电机参数初始化 (极性、减速比)
    mymotor.Polarity = 1;
    // 输出轴单位统一为 SI：rad / rad/s
    mymotor.angle_unit_convert = 1;//(2.0f * PI) / (65535.0f * deliver_ratio);
    mymotor.speed_unit_convert = 1;//mymotor.angle_unit_convert * 500.0f; // 2ms 刷新周期

    mymotor_mode = MODE_SPEED; 
}

void motor_ctrl_driver::set_motor_reduction_ratio(float reduction_ratio)
{
    deliver_ratio = reduction_ratio;
    //mymotor.angle_unit_convert = (2.0f * PI) / (65535.0f * deliver_ratio);
    //mymotor.speed_unit_convert = mymotor.angle_unit_convert * 500.0f; // 2ms 刷新周期
}

#if dm_motor_ctrl_mode
// ================= 动作接口 =================
void motor_ctrl_driver::adjust()
{
    
    if(mymotor_mode==MODE_ANGLE){
        // 串级PID: 位置环 -> 速度环
        target_motor_angle=std_lib::constrain(target_motor_angle,mymotor_limit.lower_limit,mymotor_limit.upper_limit);
        mymotor_pid_pos.Target = target_motor_angle;
        mymotor_pid_pos.Current = get_motor_angle();//位置从自己解包获取
        mymotor_pid_pos.Adjust();
        //速度环的输入为角度环输出
        mymotor_pid_spd.Target = mymotor_pid_pos.Out;
        //速度环
        mymotor_pid_spd.Current = get_motor_speed();//速度从自己解包做差获取
        mymotor_pid_spd.Adjust();
    }
    else if(mymotor_mode==MODE_SPEED){
        mymotor_pid_spd.Target = target_motor_speed;
        mymotor_pid_spd.Current = get_motor_speed();
        mymotor_pid_spd.Adjust();
    }
    else{//意外情况,一般不会进入。这里的接口外部不应该调用，而是通过motor_output的enable参数控制。
        mymotor_pid_pos.Target=mymotor_pid_pos.Current;//速度从自己解包做差获取
        mymotor_pid_pos.Out=0;
        mymotor_pid_spd.Target=0;
        mymotor_pid_spd.Out=0;
        mymotor_pid_pos.clean_intergral();
        mymotor_pid_spd.clean_intergral();
        mymotor.setMotorCurrentOut(0);
    }
}

// 输出所有电机控制电流
void motor_ctrl_driver::motor_output(bool enable){
    if(enable==false){
        mymotor_pid_pos.Target=mymotor_pid_pos.Current;
        mymotor_pid_pos.Out=0;
        mymotor_pid_spd.Target=0;
        mymotor_pid_spd.Out=0;
        mymotor_pid_pos.clean_intergral();
        mymotor_pid_spd.clean_intergral();
        mymotor.setMotorCurrentOut(0);
        return;
    }
    mymotor.setMotorCurrentOut(mymotor_pid_spd.Out);
}

// ================= 状态查询 =================

float motor_ctrl_driver::get_motor_angle(){
    float angle = (float)(dm_motor_recdata.angle * (2.0f * PI) / (65535.0f * deliver_ratio)); // 输出轴角度(rad)
    return angle;
}

static float last_speed = 0;

float motor_ctrl_driver::get_motor_speed(){
    // if(abs(mymotor_pid_spd.Out)<2)
    //     return 0;
    //float speed = (float)(dm_motor_recdata.d_angle * (2.0f * PI) / (65535.0f * deliver_ratio) * 500.0f); // 输出轴角速度(rad/s)
    //float speed = dm_motor_recdata.velocity; 
    float speed=dm_motor_recdata.d_angle;
    return speed; 
}

// 设置电机目标
void motor_ctrl_driver::set_motor_target_speed(float speed){
    target_motor_speed = speed;
}

void motor_ctrl_driver::set_motor_target_angle(float angle){
    target_motor_angle=angle;
}
// 设置电机角度限幅
void motor_ctrl_driver::set_motor_angle_limit(float lower_limit,float upper_limit){
    mymotor_limit.lower_limit = lower_limit;
    mymotor_limit.upper_limit = upper_limit;
}
// 设置电机模式
void motor_ctrl_driver::set_motor_mode(Control_Mode_e mode){
    mymotor_mode = mode;
}

#else

// 设置电机目标速度
void motor_ctrl_driver::motor_pack_dm10010(CAN_COB &txPack,float speed){
    txPack = {Can_STDID, 0, 8};
    txPack.ID = 0x200+mymotor.ID;
    memcpy(txPack.Data, &speed, 8);//浮点型,低位在前
}

void motor_ctrl_driver::enable_motor(bool enable){
    if(enable)
        mymotor.startMotor();
    else
        mymotor.stopMotor();
}

#endif

bool motor_ctrl_driver::update(uint32_t _unuse_id, uint8_t data[8])
{
    //if ((ID) != (data[0] & 0x0F))
        //return false;
    
    //dm_motor_recdata.state = (data[0]) >> 4;
    encoder = (uint16_t)(data[0] << 8) | data[1];
	dm_motor_recdata.velocity =(data[2] << 4) | (data[3] >> 4); // (-45.0,45.0)
    dm_motor_recdata.torque = ((data[4] & 0xF) << 8) | data[5];   // (-10.0,10.0)
    dm_motor_recdata.T_mos = (float)(data[6]);
    dm_motor_recdata.state = data[7]; // 电机状态

    if (encoder_is_init)
    {
        if (encoder - last_encoder > encoder_max / 2)
            round_cnt--;
        else if (encoder - last_encoder < -encoder_max / 2)
            round_cnt++;
    }
    else
    {
        encoder_offset = encoder;
        encoder_is_init = true;
    }
    last_encoder = encoder;
    dm_motor_recdata.angle  = round_cnt * encoder_max + encoder - encoder_offset;
    dm_motor_recdata.d_angle = dm_motor_recdata.angle - last_angle;
    last_angle = dm_motor_recdata.angle;
    return true;
}

