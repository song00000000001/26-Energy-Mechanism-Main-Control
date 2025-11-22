/**
 ******************************************************************************
 * @file      asuwave.h
 * @author    M3chD09 rinngo17@foxmail.com
 * @brief     Header file of asuwave.c
 * @version   1.0
 * @date      6th Apr 2021
 ******************************************************************************

 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _ASUWAVE_H_
#define _ASUWAVE_H_

/* Includes ------------------------------------------------------------------*/
#include "srml_std_lib.h"

#ifdef __cplusplus

namespace AsuwaveMonitor{
    void init(uint8_t _uart_index, uint32_t (*f)(void));
    void send(void);
}

#endif

#endif /* _ASUWAVE_H_ */
