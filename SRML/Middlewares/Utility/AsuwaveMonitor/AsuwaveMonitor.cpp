/**
 ******************************************************************************
 * @file      asuwave.c
 * @author    M3chD09 rinngo17@foxmail.com
 * @brief     
 * @version   1.0
 * @date      6th Apr 2021
 ******************************************************************************
  ==============================================================================
                               How to use this Utility
  ==============================================================================
    [..]
      Initialize the asuwave using asuwave_init() function:
        Specify the UART to communicate with computer.
        Specify the function to obtain system tick,
          which can be xTaskGetTickCount if using FreeRTOS.
    [..]
      Register RecHandle() in UART callback function.
    [..]
      Call asuwave_subscribe() in a FreeRTOS task:
        A delay of 10ms is OK.
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "srml_config.h"

#if USE_SRML_ASUWAVE_MONITOR
#include "AsuwaveMonitor.hpp"
#include "Drivers/Components/drv_uart.h"
#include "Middlewares/Protocol/SerialLineIP.hpp"
/* Private variables ---------------------------------------------------------*/
#define MaxVarNum 9

typedef uint32_t (*getTick_f)(void);

/**
 * @brief  action information definition
 */
enum ASUWAVE_ACT
{
  ASUWAVE_ACT_SUBSCRIBE = 0x01,
  ASUWAVE_ACT_SUBSCRIBERETURN,
  ASUWAVE_ACT_UNSUBSCRIBE,
  ASUWAVE_ACT_UNSUBSCRIBERETURN,
  ASUWAVE_ACT_READ,
  ASUWAVE_ACT_READRETURN,
  ASUWAVE_ACT_WRITE,
  ASUWAVE_ACT_WRITERETURN
};

/**
 * @brief  error information definition
 */
enum ASUWAVE_ERROR
{
  ASUWAVE_ERROR_NOSUCHADDRREG = 0xf9,
  ASUWAVE_ERROR_FULLADDR,
  ASUWAVE_ERROR_NOSUCHDATANUM,
  ASUWAVE_ERROR_NOSUCHADDR,
  ASUWAVE_ERROR_NOSUCHACT,
  ASUWAVE_ERROR_NOSUCHBOARD
};

/** 
 * @brief asuwave data to receive structure definition
 */
typedef struct
{
  uint8_t act; /*!< The action to be performed or the error information to be sent.
   This parameter can be any value of @ref ASUWAVE_ACT or @ref ASUWAVE_ERROR */

  uint8_t dataNum; /*!< Size amount of data to be operated.
   This parameter must be a number between Min_Data = 0 and Max_Data = 8. */
  
  uint8_t id; 

  uint32_t addr; /*!< The address of MCU to be operated.
   This parameter must be a number between Min_Data = 0x20000000 and Max_Data = 0x80000000. */

  uint64_t dataBuf; /*!< The data read or to be written.
   This parameter must be a variable of type uint64_t */

  uint8_t carriageReturn; /*!< The carriage return.
   This parameter must be '\n' */

} __attribute__((packed)) asuwave_rx_t;

/**
 * @brief asuwave data to transmit structure definition
 */
typedef struct
{
  uint32_t tick; /*!< The tick count.
   This parameter must be a variable of type uint32_t */

  uint8_t dataBuf[4 * MaxVarNum]; /*!< The data read or to be written.
   This parameter must be a variable of type uint32_t */

  uint8_t act; /*!< The action to be performed or the error information to be sent.
  This parameter can be any value of @ref ASUWAVE_ACT or @ref ASUWAVE_ERROR */

  uint8_t carriageReturn; /*!< The carriage return.
   This parameter must be '\n' */

} __attribute__((packed)) asuwave_tx_t;

/** 
 * @brief asuwave data to receive union definition
 */
typedef union
{
  asuwave_rx_t body;
  uint8_t buff[sizeof(asuwave_rx_t)];
} asuwave_rxu_t;

/** 
 * @brief asuwave data to transmit union definition
 */
typedef union
{
  asuwave_tx_t body;
  uint8_t buff[sizeof(asuwave_tx_t)];
} asuwave_txu_t;

/* definition of the list of address to read */
typedef struct
{
  uint8_t dataNum;
  uint32_t addr;
} list_addr_t;

static getTick_f getTick;
static list_addr_t list_addr[MaxVarNum];
static USART_COB TxPack;
/* function prototypes -------------------------------------------------------*/

static void SEND(uint8_t* addr, uint8_t len)
{
  /* Pack using SLIP */
  pair<uint8_t*, int> packet = SerialLineIP::Pack(addr, len);
  /* Send return data */
  TxPack.address = packet.first;
  TxPack.len = packet.second;
  SRML_UART_Transmit_DMA(&TxPack);
}

static void RETURN_ACK(uint8_t act)
{
  static uint8_t ack[2] = {0, '\n'};
  /* Prepare the data to send */
  ack[0] = act;

  /* Send the error message */
  SEND(ack, 2);
}

/**
 * @brief  Register the address.
 * @param  asuwave_rxu: received asuwave data union.
 * @retval the index of the list_addr to register.
 */
static void ADDR_REGISTER(asuwave_rxu_t *asuwave_rxu)
{
  list_addr[asuwave_rxu->body.id].dataNum = asuwave_rxu->body.dataNum;
  list_addr[asuwave_rxu->body.id].addr = asuwave_rxu->body.addr;
}

/**
 * @brief  Unregister the address.
 * @param  asuwave_rxu: received asuwave data union.
 * @retval the index of the list_addr to unregister.
 */
static void ADDR_UNREGISTER(asuwave_rxu_t *asuwave_rxu)
{
  if(list_addr[asuwave_rxu->body.id].addr == asuwave_rxu->body.addr)
  {
    list_addr[asuwave_rxu->body.id].dataNum = 0;
    list_addr[asuwave_rxu->body.id].addr = 0;
  }else{
    RETURN_ACK(ASUWAVE_ERROR_NOSUCHADDR);
  }
}

/**
 * @brief  Writes the given data buffer to the flash of the given address.
 * @param  asuwave_rxu: received asuwave data union.
 * @retval None
 */
static void WRITE(asuwave_rxu_t *asuwave_rxu)
{
  /* Write data buffer */
  uint8_t n = asuwave_rxu->body.dataNum;
  if (n > 8 || (n & (n - 1)))
  {
    RETURN_ACK(ASUWAVE_ERROR_NOSUCHDATANUM);
    return;
  }
  memcpy((void*)asuwave_rxu->body.addr, (uint8_t*)&(asuwave_rxu->body.dataBuf), n);
  RETURN_ACK(ASUWAVE_ACT_WRITERETURN);
}

/**
 * @brief  asuwave callback.
 * @param  data_buf: received buffer array.
 * @param  length: the length of array.
 * @retval None
 */
uint32_t RecHandle(uint8_t *data_buf, uint16_t length)
{
  asuwave_rxu_t asuwave_rxu;
  if (length != sizeof(asuwave_rxu.buff)) return 1;
  memcpy(&asuwave_rxu.buff, data_buf, length);

  /* Check if it is a valid action information and execute */
  switch (asuwave_rxu.body.act)
  {
    case ASUWAVE_ACT_SUBSCRIBE:
      if (asuwave_rxu.body.id >= MaxVarNum)
        RETURN_ACK(ASUWAVE_ERROR_FULLADDR);
      else
        ADDR_REGISTER(&asuwave_rxu);
      break;
    case ASUWAVE_ACT_WRITE:
      WRITE(&asuwave_rxu);
      break;
    case ASUWAVE_ACT_UNSUBSCRIBE:
      ADDR_UNREGISTER(&asuwave_rxu);
      break;
    default:
      RETURN_ACK(ASUWAVE_ERROR_NOSUCHACT);
      return 1;
  }

  return 0;
}

/**
 * @brief  Subscribes the variable in flash memory of the given address.
 * @param  None
 * @retval None
 */
void AsuwaveMonitor::send(void)
{
  static asuwave_txu_t asuwave_txu;
  /* Prepare the data to send */
  asuwave_txu.body.tick = getTick();
  asuwave_txu.body.act = ASUWAVE_ACT_SUBSCRIBERETURN;
  asuwave_txu.body.carriageReturn = '\n';

  uint32_t addr;
  for (int i = 0; i < MaxVarNum; i++)
  {
    if (list_addr[i].dataNum == 0 || list_addr[i].addr == 0) continue;

    addr = list_addr[i].addr;
    /* Reads the variable2 */
    for (int j = 0; j < list_addr[i].dataNum; j++)
    {
      *(asuwave_txu.body.dataBuf + 4*i + j) = *(__IO uint8_t*) addr++;
    }
  }

  SEND(asuwave_txu.buff, sizeof(asuwave_txu_t));
}

/**
 * @brief  Register uart device.
 * @param  *huart: pointer of uart IRQHandler.
 * @param  *f: function pointer to obtain system tick, which can be xTaskGetTickCount if using FreeRTOS
 * @retval None
 */
void AsuwaveMonitor::init(uint8_t _uart_index, uint32_t (*f)(void))
{
	TxPack.port_num = _uart_index;
  Usart_Rx_Callback_Register(_uart_index, RecHandle);
  getTick = f;
}
#endif  /* USE_SRML_ASUWAVE */
