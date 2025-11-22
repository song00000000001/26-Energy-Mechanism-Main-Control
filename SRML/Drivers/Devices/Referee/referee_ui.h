/**
 ******************************************************************************
 * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
 * @file    referee_ui.h
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

#define UI_X_VeryLeft 240
#define UI_X_Left 480
#define UI_X_LittleLeft 720
#define UI_X_Middle 960
#define UI_X_LittleRight 1200
#define UI_X_Right 1440
#define UI_X_VeryRight 1680

#define UI_Y_VeryHigh 954
#define UI_Y_High 810
#define UI_Y_LittleHigh 675
#define UI_Y_Middle 540
#define UI_Y_LittleLow 405
#define UI_Y_Low 270
#define UI_Y_VeryLow 135

#ifdef __cplusplus
extern "C"
{
#endif
    /* Includes ------------------------------------------------------------------*/
    /* Private macros ------------------------------------------------------------*/
    /* Private type --------------------------------------------------------------*/
    /* Exported macros -----------------------------------------------------------*/

    /* Exported types ------------------------------------------------------------*/
    /* 图形配置操作指令，参照官方手册P23 */
    typedef enum
    {
        NULL_OPERATION = 0U,
        ADD_PICTURE = 1U,
        MODIFY_PICTURE = 2U,
        CLEAR_ONE_PICTURE = 3U,
    } drawOperate_e;

    /* 图形清除操作指令 */
    typedef enum
    {
        CLEAR_ONE_LAYER = 1U,
        CLEAR_ALL = 2U,
    } clearOperate_e;

    /* 图形绘制类型 */
    typedef enum
    {
        LINE = 0U,
        RECTANGLE = 1U,
        CIRCLE = 2U,
        OVAL = 3U,
        ARC = 4U,
        _FLOAT = 5U,
        _INT = 6U,
        _CHAR = 7U,
    } graphic_tpye_e;

    /* 图形色彩配置 */
    typedef enum
    {
        RED = 0U,
        BLUE = 0U,
        YELLOW,
        GREEN,
        ORANGE,
        PURPLE,
        PINK,
        DARKGREEN,
        BLACK,
        WHITE
    } colorType_e;

    /* Exported function declarations --------------------------------------------*/

#ifdef __cplusplus
}
#endif

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
