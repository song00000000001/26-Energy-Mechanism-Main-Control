/*
 * app.h
 */

#ifndef _APP_H_
#define _APP_H_

#include "stdbool.h"
#ifdef __cplusplus
#include "stdint.h"
extern "C" {
#endif

void System_Device_Init(void);
void System_Task_Init(void);
void Launch_Callback();
void packDecoder(uint8_t * _addr,uint8_t len);
uint8_t getLoadStatus();
bool Is_Launching();
#ifdef __cplusplus
}
#endif

#endif /* _APP_H_ */
