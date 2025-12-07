
#include "internal.h"
#include "global_data.h"
#include "robot_config.h"

/**
 * @brief Yaw轴控制任务
 * @parma None
 * @return None
 */



//尝试合并装填任务,不知道能不能直接在can发送队列里连续发两次
static myPID load_pid_ang;									// 装填电机的编码器环
static myPID load_pid_spd;									// 装填电机的速度环


void key_check(){
    // 初始化5*pid参数
    //Launcher.init();
    
    // 初始状态设为自检
    Robot.Status.current_state = SYS_CHECKING;
    // 记录哪几个开关已经检测过了 (Bitmask)
    static uint8_t check_progress = 0; 
    if(READ_SW_YAW_L){
        check_progress |= MASK_YAW_L;
    }
    if(READ_SW_YAW_R){
        check_progress |= MASK_YAW_R;
    }
    if(READ_SW_DELIVER_L){
        check_progress |= MASK_DELIVER_L;
    }
    if(READ_SW_DELIVER_R){
        check_progress |= MASK_DELIVER_R;
    }
    if(READ_SW_IGNITER){
        check_progress |= MASK_IGNITER;
    }
    // 3. 处理跳过逻辑
    if (Robot.Cmd.skip_check) {
        check_progress = MASK_ALL_PASSED; // 强制全满
    }

    // 4. 更新全局标志位
    Robot.Flag.Check.limit_sw_ok = (check_progress == MASK_ALL_PASSED);
   
}

void stop_all_motor(){
    Launcher.stop(); 
    Yawer.disable();
    loadermotor.setMotorCurrentOut(0);
    load_pid_ang.clean_intergral();
    load_pid_spd.clean_intergral();
    Launcher.fire_lock();
}

void yaw_calibration(){
    //进行校准，校准完成后，自动改变校准标志
    Yawer.init();
    //计算电机pid
    Yawer.adjust();
}

//yaw摇杆控制函数
void yaw_remote_control_task(){

    if(DR16.GetS2() == SW_MID) // 右拨杆中档，手动模式
    {
        Robot.Status.yaw_control_state = MANUAL_AIM;
    }
    else if(DR16.GetS2() == SW_DOWN) // 右拨杆朝下，修正模式
    {
        Robot.Status.yaw_control_state = CORRECT_AIM;
    }//右拨杆朝上,取消使能
    else{
        Robot.Status.yaw_control_state = disable_motor;
    }
}

void yaw_state_machine(){
    
    static float yaw_target = 0;//, yaw_goal = 0, igniter_target_pos = 0, igniter_goal_pos = 0;
    static float yaw_correct_angle;        //yaw轴修正角
    static float default_yaw_target[2]; // 默认前哨站和基地角度

    switch (Robot.Status.yaw_control_state)
    {
    case MANUAL_AIM:
        yaw_target -= DR16.Get_LX_Norm() * 0.002f;
        yaw_target = std_lib::constrain(yaw_target, -10.2f, 10.2f);
        Yawer.update(yaw_target);
        break;
    case CORRECT_AIM:
        //根据目标选择修正角度
        //固定修正值模式
        Yawer.update(yaw_correct_angle + default_yaw_target[HitTarget]); // 更改Yaw轴角度
        break;
    case VISION_AIM:
        //视觉模式
        //todo
        {/*todo
        测试时，暂时不管视觉
        storage_base_angle = default_yaw_target[HitTarget];

        if (vision_recv_pack.ros == 1)
        {
            yaw_target += 0.0003;
        }
        if (vision_recv_pack.ros == 2)
        {
            yaw_target -= 0.0003;
        }
        if (vision_recv_pack.ros == 0)
        {
            yaw_target += 0;
        }
        yaw_target = std_lib::constrain(yaw_target, -10.2f, 10.2f);
        Yawer.update(yaw_target);
        default_yaw_target[HitTarget] = yaw_target;
        //计算电机pid
        Yawer.adjust();
        */
       }
        break;
    case disable_motor:
    default:
        Yawer.disable();
        break;
    }

}

void yaw_and_load_motor_control(){
    //这里为了方便,脱离状态机单独处理可能的输出。
    if( Yawer.is_Yaw_Init() == 1 && Robot.Status.yaw_control_state!=disable_motor&& Robot.Status.current_state != SYS_OFFLINE)
    {
        //计算电机pid
        Yawer.adjust();
    }
    else
    {
        Yawer.disable();
    }

    if(Robot.Status.current_state != SYS_OFFLINE)
    {
        //装填任务
        //计算装填电机pid
        load_pid_ang.Current = loadermotor.getMotorTotalAngle();//换成getTotalAngle试试,测试后发现,total才是绝对,直接get的会有跳变
        load_pid_ang.Adjust();

        load_pid_spd.Target = load_pid_ang.Out;
        load_pid_spd.Current = loadermotor.getMotorSpeed();
        load_pid_spd.Adjust();

        //设置电机输出
        loadermotor.setMotorCurrentOut(load_pid_spd.Out);
    }
    else
    {
        load_pid_ang.clean_intergral();
        load_pid_spd.clean_intergral();
        loadermotor.setMotorCurrentOut(0);
    }
}

void Yaw_Task(void *arg)
{
    Motor_CAN_COB Tx_Buff;
	Motor_CAN_COB Tx_Buff1;
	TickType_t xLastWakeTime_t;

    //初始化参数可以考虑集成到launcher中
    load_pid_ang.SetPIDParam(10, 0, 0, 100, 1200);
	load_pid_spd.SetPIDParam(50, 1, 0, 1000, 16000);
    load_pid_ang.Target = 120+5;    //安全位置

    //测试用,只执行一次
    // 测试舵机动作
    test_servo_action(); 

    // 任务频率控制
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1);
	for (;;)
	{
		vTaskDelayUntil(&xLastWakeTime_t, xFrequency);

        if (DR16.GetStatus() != DR16_ESTABLISHED) {
            Robot.Status.current_state = SYS_OFFLINE;
        }
        
        //s2摇杆解析
        yaw_remote_control_task();
        //独立状态机运行，但受到状态管控，主要是并行任务
        //根据s2摇杆切换失能，手动，修正，视觉（暂无）模式
        yaw_state_machine();
        //yaw轴和装填电机控制，都有失能保护，yaw轴还有初始化保护
        yaw_and_load_motor_control();

        //主状态机
        switch (Robot.Status.current_state)
        {
        case SYS_OFFLINE:
            {
                //断电
                stop_all_motor();
                // 恢复条件：遥控器重连
                if (DR16.GetStatus() == DR16_ESTABLISHED) {
                    Robot.Status.current_state = SYS_CHECKING;
                }
                break;
            }
        case SYS_CHECKING:
            {
     
                stop_all_motor();
                key_check();
                         
                // 5个按键手动检查全部通过则进入校准状态
                if (Robot.Flag.Check.limit_sw_ok) {
                    Robot.Status.current_state = SYS_CALIBRATING;
                }
        
            }
            break;
        
        case SYS_CALIBRATING:
            {
                yaw_calibration();
                //校准完毕跳转待机状态
                if (Yawer.is_Yaw_Init() == 1){
                    Robot.Status.current_state = SYS_STANDBY;
                }
            }
            break;
        
        case SYS_STANDBY:
        case SYS_AUTO_PREP:
        case SYS_AUTO_FIRE:
        default:
            break;
        }

	    /*打包发送*/
        MotorMsgPack(Tx_Buff1, Yawer.YawMotor);
        xQueueSend(CAN2_TxPort, &Tx_Buff1.Id1ff, 0);
     
        //不知道能不能这样写?
        //装填电机gm6020-id2，用此命令 
        MotorMsgPack(Tx_Buff, loadermotor);
        xQueueSend(CAN2_TxPort, &Tx_Buff.Id2ff, 0); 
       

	}
}