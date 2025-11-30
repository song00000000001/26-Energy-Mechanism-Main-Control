#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "stdbool.h"
#ifdef __cplusplus
	#include "stdint.h"
	extern "C" {
#endif

void System_Device_Init(void);
void System_Task_Init(void);
//void Launch_Callback();
//uint8_t getLoadStatus();
//bool Is_Launching();
#ifdef __cplusplus
}
#endif

#endif /* _CONFIG_H_ */