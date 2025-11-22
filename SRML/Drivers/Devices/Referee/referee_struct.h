/**
 ******************************************************************************
 * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
 * @file    referee_struct.h
 * @author  余俊晖 2460857175@QQ.COM
 * @brief   Header file
 ******************************************************************************
 * @attention
 *
 * if you had modified this file, please make sure your code does not have many
 * bugs, update the version Number, write dowm your name and the date, the most
 * important is make sure the users will have clear and definite understanding
 * through your new brief.
 *
 * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
 * All rights reserved.</center></h2>
 ******************************************************************************
 */
#pragma once

#define ROBOT_COM_PACK 64

#ifdef __cplusplus
extern "C"
{
#endif
    /* Includes ------------------------------------------------------------------*/
    /* Private macros ------------------------------------------------------------*/
    /* Private type --------------------------------------------------------------*/
    /* Exported macros -----------------------------------------------------------*/
    /* Exported types ------------------------------------------------------------*/
#pragma pack(1)
    /* 己方机器人ID及当前机器人客户端ID */
    typedef struct
    {
        uint8_t hero;
        uint8_t engineer;
        uint8_t infantry_3;
        uint8_t infantry_4;
        uint8_t infantry_5;
        uint8_t aerial;
        uint8_t sentry;
        uint8_t dart;  // 飞镖发射架
        uint8_t radar; // 雷达站
        uint8_t local; // 本机器人ID

        uint8_t robot_where; // 蓝方 or 红方机器人
        uint16_t client;     // 对应的客户端ID
    } __attribute__((packed)) RC_ID;

    /* 数据帧帧头结构体 */
    typedef struct
    {
        uint8_t SOF;
        uint16_t DataLength;
        uint8_t Seq;
        uint8_t CRC8;
        uint16_t CmdID;
    } __attribute__((packed)) FrameHeader;

    /* 交互数据帧的统一数据段段头 */
    typedef struct
    {
        uint16_t data_cmd_id;
        uint16_t send_ID;
        uint16_t receiver_ID;
    } __attribute__((packed)) DataHeader;

    /* ----------------------------------比赛数据帧的数据段框架--------------------------------------- */
    /**
      @brief  比赛状态数据：0x0001，1Hz
    */
    typedef struct
    {
        uint8_t game_type : 4;
        uint8_t game_progress : 4;
        uint16_t stage_remain_time;

        uint64_t SyncTimeStamp;
    } __attribute__((packed)) ext_game_status_t;

    /**
       @brief 比赛结果数据：0x0002，在比赛结束后发送
    */
    typedef struct
    {
        uint8_t winner;
    } __attribute__((packed)) ext_game_result_t;

    /**
       @brief 机器人血量数据：0x0003，1Hz
    */
    typedef struct
    {
        uint16_t red_1_robot_HP;
        uint16_t red_2_robot_HP;
        uint16_t red_3_robot_HP;
        uint16_t red_4_robot_HP;
        uint16_t red_5_robot_HP;
        uint16_t red_7_robot_HP;
        uint16_t red_outpost_HP;
        uint16_t red_base_HP;

        uint16_t blue_1_robot_HP;
        uint16_t blue_2_robot_HP;
        uint16_t blue_3_robot_HP;
        uint16_t blue_4_robot_HP;
        uint16_t blue_5_robot_HP;
        uint16_t blue_7_robot_HP;
        uint16_t blue_outpost_HP;
        uint16_t blue_base_HP;
    } __attribute__((packed)) ext_game_robot_HP_t;

    /**
       @brief 飞镖发射状态：0x0004，飞镖发射后发送（05.06删去）
    */
    typedef struct
    {
        uint8_t dart_belong;           // 发射方
        uint16_t stage_remaining_time; // 发射时比赛剩余时间
    } __attribute__((packed)) ext_dart_status_t;

    /**
       @brief 人工智能挑战赛加成与惩罚区状态：0x0005，1Hz
    */
    typedef struct
    {
        uint8_t F1_zone_status : 1;
        uint8_t F1_zone_buff_debuff_status : 3;
        uint8_t F2_zone_status : 1;
        uint8_t F2_zone_buff_debuff_status : 3;
        uint8_t F3_zone_status : 1;
        uint8_t F3_zone_buff_debuff_status : 3;
        uint8_t F4_zone_status : 1;
        uint8_t F4_zone_buff_debuff_status : 3;
        uint8_t F5_zone_status : 1;
        uint8_t F5_zone_buff_debuff_status : 3;
        uint8_t F6_zone_status : 1;
        uint8_t F6_zone_buff_debuff_status : 3;

        uint16_t red1_bullet_felt;
        uint16_t red2_bullet_felt;
        uint16_t blue1_bullet_felt;
        uint16_t blue2_bullet_felt;

    } __attribute__((packed)) ext_ICRA_buff_debuff_zone_status_t;

    /**
       @brief 场地事件数据：0x0101，事件改变发送
    */
    typedef struct
    {
        uint32_t event_type;
    } __attribute__((packed)) ext_event_data_t;

    /**
       @brief 补给站动作标识：0x0102	动作改变后发送
    */
    typedef struct
    {
        uint8_t supply_projectile_id;
        uint8_t supply_robot_id;
        uint8_t supply_projectile_step;
        uint8_t supply_projectile_num;
    } __attribute__((packed)) ext_supply_projectile_action_t;

    /**
       @brief 裁判警告信息：0x0104	警告发生后发送
    */
    typedef struct
    {
        uint8_t level;
        uint8_t foul_robot_id;
        uint8_t count;
    } __attribute__((packed)) ext_referee_warning_t;

    /**
       @brief 飞镖发射口倒计时：0x0105，1Hz
    */
    typedef struct
    {
        uint8_t dart_remaining_time;
        uint16_t dart_info;
    } __attribute__((packed)) ext_dart_remaining_time_t;

    /**
       @brief 哨兵自主决策信息：0x0120，1Hz
    */
    typedef struct
    {
        uint32_t sentry_cmd;
    } __attribute__((packed)) sentry_cmd_t;

    /**
       @brief 雷达自主决策信息：0x0121，1Hz
    */
    typedef struct
    {
        uint8_t radar_cmd;
    } __attribute__((packed)) radar_cmd_t;

    /**
       @brief 当前比赛机器人状态：0x0201，10Hz
    */
    typedef struct
    {
        uint8_t robot_id;
        uint8_t robot_level;
        uint16_t current_HP;
        uint16_t max_HP;
        uint16_t shooter_barrel_cooling_value;
        uint16_t shooter_barrel_heat_limit;
        uint16_t classis_power_limit;
        uint8_t mains_power_gimbal_output : 1;
        uint8_t mains_power_chassis_output : 1;
        uint8_t mains_power_shooter_output : 1;
    } __attribute__((packed)) ext_game_robot_status_t;

    /**
       @brief 实时功率热量数据：0x0202，50Hz
    */
    typedef struct
    {
        uint16_t chassis_volt;
        uint16_t chassis_current;
        float chassis_power;
        uint16_t chassis_power_buffer;
        uint16_t shooter_id1_17mm_cooling_heat;
        uint16_t shooter_id2_17mm_cooling_heat;
        uint16_t shooter_id1_42mm_cooling_heat;
    } __attribute__((packed)) ext_power_heat_data_t;

    /**
       @brief 机器人位置：0x0203,10Hz
    */
    typedef struct
    {
        float x;
        float y;
        //   float z;
        float angle;
    } __attribute__((packed)) ext_game_robot_pos_t;

    /**
       @brief 机器人增益：0x0204，状态改变后发送
    */
    typedef struct
    {
        //   uint8_t power_rune_buff;
        uint8_t recovery_buff;      // 机器人回血增益
        uint8_t cooling_buff;       // 机器人枪口冷却倍率
        uint8_t defence_buff;       // 机器人防御增益
        uint8_t vulnerability_buff; // 机器人负防御增益
        uint16_t attack_buff;       // 机器人攻击增益
    } __attribute__((packed)) ext_buff_t;

    /**
       @brief 空中机器人能量状态：0x0205，10Hz
    */
    typedef struct
    {
        uint8_t attack_time;
        uint8_t time_remain;
    } __attribute__((packed)) aerial_robot_energy_t;

    /**
       @brief 伤害状态：0x0206，受到伤害后发送
    */
    typedef struct
    {
        uint8_t armor_id : 4;
        uint8_t hurt_type : 4;
    } __attribute__((packed)) ext_robot_hurt_t;

    /**
       @brief 实时射击信息：0x0207，射击后发送
    */
    typedef struct
    {
        uint8_t bullet_type;
        uint8_t shooter_id;
        uint8_t bullet_freq;
        float bullet_speed;
    } __attribute__((packed)) ext_shoot_data_t;

    /**
       @brief 子弹剩余发射数：0x0208，10Hz
       @note  关于发送范围，官方2021.05.06的协议自相矛盾。目前在对抗赛服务端上测试过可发送范围：所有机器人
    */
    typedef struct
    {
        uint16_t bullet_remaining_num_17mm;
        uint16_t bullet_remaining_num_42mm;
        uint16_t coin_remaining_num;
    } __attribute__((packed)) ext_bullet_remaining_t;

    /**
       @brief 机器人RFID状态：0x0209，1Hz
    */
    typedef struct
    {
        uint32_t rfid_status;                    /* 每个位代表不同地点的RFID状态 */
    } __attribute__((packed)) ext_rfid_status_t; /*RFID状态不完全代表对应的增益或处罚状态，例如敌方已占领的高低增益点，不能获取对应的增益效果*/

    /**
        @brief 飞镖机器人客户端指令数据：0x020A，10Hz 只对飞镖发送
    */
    typedef struct
    {
        uint8_t dart_launch_opening_status;
        uint8_t dart_attack_target;
        uint16_t target_change_time;
        uint16_t operate_launch_cmd_time;
    } __attribute__((packed)) ext_dart_client_cmd_t;

    /**
        @brief 己方机器人位置，命令码0x020B
    */
    typedef struct
    {
        float hero_x;       // 己方英雄机器人位置 X 轴坐标，单位：m
        float hero_y;       // 己方英雄机器人位置 Y 轴坐标，单位：m
        float engineer_x;   // 己方工程机器人位置 X 轴坐标，单位：m
        float engineer_y;   // 己方工程机器人位置 X 轴坐标，单位：m
        float standard_3_x; // 己方工程机器人位置 X 轴坐标，单位：m
        float standard_3_y; // 己方 3 号步兵机器人位置 Y 轴坐标，单位：m
        float standard_4_x; // 己方 4 号步兵机器人位置 X 轴坐标，单位：m
        float standard_4_y; // 己方 4 号步兵机器人位置 X 轴坐标，单位：m
        float standard_5_x; // 己方 4 号步兵机器人位置 X 轴坐标，单位：m
        float standard_5_y; // 己方 4 号步兵机器人位置 X 轴坐标，单位：m
    } __attribute__((packed)) ground_robot_position_t;

    /**
        @brief 对方机器人被标记进度，命令码0x020C
    */
    typedef struct
    {
        uint8_t mark_hero_progress;       // 对方英雄机器人被标记进度：0~120
        uint8_t mark_engineer_progress;   // 对方英雄机器人被标记进度：0~120
        uint8_t mark_standard_3_progress; // 对方 3 号步兵机器人被标记进度：0~120
        uint8_t mark_standard_4_progress; // 对方 4 号步兵机器人被标记进度：0~120
        uint8_t mark_standard_5_progress; // 对方 5 号步兵机器人被标记进度：0~120
        uint8_t mark_sentry_progress;     // 对方哨兵机器人被标记进度：0~120
    } __attribute__((packed)) radar_mark_data_t;

    /**
        @brief 哨兵自主决策信息同步，命令码0x020D
    */
    typedef struct
    {
        uint32_t sentry_info;
        uint16_t sentry_info_2;
    } __attribute__((packed)) sentry_info_t;

    /**
        @brief 雷达自主决策信息同步，命令码0x020E
    */
    typedef struct
    {
        uint8_t radar_info;
    } __attribute__((packed)) radar_info_t;

    /* ----------------------------------裁判系统客户端交互部分--------------------------------------- */
    /**
       @brief 交互数据段通用段头结构体定义：自定义数据的命令ID为：0x0301，10Hz
    */
    typedef struct
    {
        uint16_t data_cmd_id;
        uint16_t sender_ID;
        uint16_t receiver_ID;
    } __attribute__((packed)) ext_student_interactive_header_data_t;

    /**
       @brief 学生机器人间通信：命令ID为0x0301，内容ID：在0x0200~0x02FF内自由选择，10Hz
       @brief 这里定义了完整的数据段格式，虽然看起来有些冗余，但方便处理其他机器人的发过来的数据报
    */
    typedef struct
    {
        uint16_t data_cmd_id; // 数据段段首
        uint16_t sender_ID;
        uint16_t receiver_ID;
        uint8_t data[112]; //!< 长度需要小于113个字节（官方手册约定）
    } __attribute__((packed)) robot_interactive_data_t;

    // typedef struct
    // {
    //    float x[6];
    //    float y[6];
    // }__attribute__((packed))ext_client_enemy_position_t;

    typedef struct
    {
        uint8_t data[ROBOT_COM_PACK]; // 接收到的数据
    } __attribute__((packed)) my_interactive_data_t;

    /**
       @brief 自定义控制器交互数据：0x0302，30Hz；
       @brief 注意！该交互数据包数据段没有段首，直接就是数据部分
    */
    typedef struct
    {
        uint8_t data[30]; //!< 长度需要小于30个字节（官方手册约定）
    } __attribute__((packed)) custom_controller_interactive_data_t;

    /**
       @brief 小地图下发数据：0x0303 触发时发送；
    */
    typedef struct
    {
        float target_position_x;
        float target_position_y;
        uint8_t commd_keyboard;
        uint16_t target_robot_ID;
        uint8_t cmd_source;
    } __attribute__((packed)) ext_mini_map_command_t;

    /**
       @brief 键鼠信息，通过图传发送到机器人（图传发送端的3pin接口）：0x0304；
    */
    typedef struct
    {
        int16_t mouse_x;
        int16_t mouse_y;
        int16_t mouse_z;
        int8_t left_button_down;
        int8_t right_button_down;
        uint16_t keyboard_value;
        uint16_t reserved;
    } __attribute__((packed)) ext_VT_command_t;

    /**
       @brief 雷达站下发数据帧内容定义，将被己方操作手以第一人称看到，0x0305
    */
    typedef struct
    {
        uint16_t target_robot_ID; // 敌方机器人的坐标
        float target_position_x;
        float target_position_y;
        float toward_angle;
    } __attribute__((packed)) ext_client_map_command_t;

    /**
       @brief 操作手可使用自定义控制器模拟键鼠操作选手端，0x0306
    */
    typedef struct
    {
        uint16_t key_value;
        uint16_t x_position : 12;
        uint16_t mouse_left : 4;
        uint16_t y_position : 12;
        uint16_t mouse_right : 4;
        uint16_t reserved;
    } __attribute__((packed)) custom_client_data_t;

    /**
       @brief 哨兵机器人可向己方空中机器人选手端发送路径坐标数据，该路径会在其小地图上显示，0x0307
    */
    typedef struct
    {
        uint8_t intention;
        uint16_t start_position_x;
        uint16_t start_position_y;
        int8_t delta_x[49];
        int8_t delta_y[49];
        uint16_t sender_id;
    } __attribute__((packed)) map_sentry_data_t;

    /**
       @brief 己方机器人可通过常规链路向己方任意选手端发送自定义的消息，该消息会在己方选手端特定位置显示,0x0308
    */
    typedef struct
    {
        uint16_t sender_id;
        uint16_t receiver_id;
        uint8_t user_data[30];
    } __attribute__((packed)) map_custom_data_t;
    /* ----------------------------------客户端绘图相关--------------------------------------- */
    /* 绘制图形操作，数据段 */
    typedef struct
    {
        uint8_t graphic_name[3];   // 图形名称，作为客户端中的索引
        uint32_t operate_tpye : 3; // 图形操作（空、增加、修改、删除）
        uint32_t graphic_tpye : 3; // 图形类型（何种图案或数字）
        uint32_t layer : 4;        // 图层数
        uint32_t color : 4;        // 颜色
        uint32_t start_angle : 9;  // 起始角度
        uint32_t end_angle : 9;    // 终止角度
        uint32_t width : 10;       // 线宽
        uint32_t start_x : 11;     // 起点坐标
        uint32_t start_y : 11;
        uint32_t radius : 10; // 字体大小或半径
        uint32_t end_x : 11;  // 终点坐标
        uint32_t end_y : 11;
    } __attribute__((packed)) graphic_data_struct_t;

    /* 删除指定图层操作，数据段 */
    typedef struct
    {
        uint8_t operate_tpye;
        uint8_t layer;
    } __attribute__((packed)) ext_client_custom_graphic_delete_t;

#pragma pack()
    /* Exported function declarations --------------------------------------------*/

#ifdef __cplusplus
}
#endif

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
