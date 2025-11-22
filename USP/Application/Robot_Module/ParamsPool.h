#pragma once

#ifdef __cplusplus
extern "C"{
#endif
/* Includes ------------------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private type --------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/
#define INIT_SPEED_DELIVER            6000
#define INIT_SPEED_IGNITER            -4500
#define INIT_SPEED_YAW                -500


#define SW_DELIVER_L_Pin              GPIO_PIN_4
#define SW_DELIVER_L_GPIO_Port        GPIOC
#define SW_DELIVER_R_Pin              GPIO_PIN_5
#define SW_DELIVER_R_GPIO_Port        GPIOA
#define SW_IGNITER_Pin                GPIO_PIN_14
#define SW_IGNITER_GPIO_Port          GPIOC

#define SW_YAW_R_Pin                  GPIO_PIN_6
#define SW_YAW_R_GPIO_Port            GPIOA
#define SW_YAW_L_Pin                  GPIO_PIN_7
#define SW_YAW_L_GPIO_Port            GPIOA


#define POLARITY_DELIVER_L            -1
#define POLARITY_DELIVER_R            1
#define POLARITY_IGNITER              1
#define POLARITY_YAW                  1

#define ID_DELIVER_L                  4
#define ID_DELIVER_R                  3
#define ID_IGNITER                    1
#define ID_YAW                        2
/* Exported types ------------------------------------------------------------*/
/* Exported function declarations --------------------------------------------*/

#ifdef  __cplusplus
}
#endif

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/