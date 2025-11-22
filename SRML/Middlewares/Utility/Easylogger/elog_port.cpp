/*
 * This file is part of the EasyLogger Library.
 *
 * Copyright (c) 2015, Armink, <armink.ztl@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * 'Software'), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Function: Portable interface for each platform.
 * Created on: 2015-04-28
 */
#include "srml_config.h"

#if USE_SRML_EASYLOG
  #include "Middlewares/Utility/Easylogger/elog.h"

#if USE_SRML_UART
  #include "Drivers/Components/drv_uart.h"
#endif

#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

static uint32_t elog_transmit(uint8_t* buff, uint16_t len);

/* SRML的发送串口 */
static USART_COB txbuf;
static Easylog_Transmit_Type easylog_type = EASY_OPENLOG;
static SemaphoreHandle_t easylog_mutex = NULL;

void Easylog_init(uint8_t _port_num, Easylog_Transmit_Type _type)
{
    txbuf.port_num = _port_num;
    easylog_type = _type;
		easylog_mutex = xSemaphoreCreateMutex();
		
    elog_init();

    /* set EasyLogger log format */
    elog_set_fmt(ELOG_LVL_ASSERT, ELOG_FMT_ALL);
    elog_set_fmt(ELOG_LVL_ERROR, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_WARN, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_INFO, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_DEBUG, ELOG_FMT_ALL & ~ELOG_FMT_FUNC);
    elog_set_fmt(ELOG_LVL_VERBOSE, ELOG_FMT_ALL);
    
    /* start EasyLogger */
    elog_start();
}

/**
 * EasyLogger port initialize
 *
 * @return result
 */
ElogErrCode elog_port_init(void)
{
    return ELOG_NO_ERR;
}

/**
 * EasyLogger port deinitialize
 *
 */
void elog_port_deinit(void)
{
}

/**
 * output log port interface
 *
 * @param log output of log
 * @param size log size
 */
void elog_port_output(const char* log, size_t size)
{
		if ((easylog_type == EASY_OPENLOG && xTaskGetTickCount() >= 3000) || easylog_type == EASY_UART)
		{
			elog_transmit((uint8_t*)log, size);
		}
}

/**
 * output lock
 */
void elog_port_output_lock(void)
{
	if(easylog_mutex != NULL)
	{
		xSemaphoreTake(easylog_mutex, portMAX_DELAY);
	}
}

/**
 * output unlock
 */
void elog_port_output_unlock(void)
{
	if(easylog_mutex != NULL)
	{
		xSemaphoreGive(easylog_mutex);
	}
}

/**
 * get current time interface
 *
 * @return current time
 */
const char* elog_port_get_time(void)
{
    static char time_string[12];
    uint32_t running_time_ms = xTaskGetTickCount();
    float running_time_s = running_time_ms / 1000.f;

    sprintf(time_string, "%8.3f", running_time_s);

    return (const char*)time_string;
}

/**
 * get current process name interface
 *
 * @return current process name
 */
const char* elog_port_get_p_info(void)
{
    return "";
}

/**
 * get current thread name interface
 *
 * @return current thread name
 */
const char* elog_port_get_t_info(void)
{
    return "";
}

static uint32_t elog_transmit(uint8_t* buff, uint16_t len)
{
    txbuf.address = (uint8_t*)buff;
    txbuf.len = len;

    if (txbuf.port_num != 0)
        SRML_UART_Transmit_DMA(&txbuf);

    return 0;
}

#endif
