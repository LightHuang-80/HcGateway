/*
 * fsuart.cpp
 *
 *  Created on: Sep 18, 2020
 *      Author: Administrator
 */
#include "FreeRTOS.h"
#include "main.h"
#include "ros.h"
#include "UsartWithBuffer.h"
#include "STM32Hardware.h"

extern ros::NodeHandle* ROSNodeH;
extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
extern "C" void HAL_UART_IdleCallback(UART_HandleTypeDef *huart);

#define RXFIFO_BUFSIZE 2048
char g_RxFifoBuf[RXFIFO_BUFSIZE] __attribute__((section(".dma_buffer"))) ;

#define UART_TXDMA_BUFSIZE 1024
char g_UARTTxDMABuf[UART_TXDMA_BUFSIZE] __attribute__((section(".dma_buffer"))) ;

/* @brief Receive buffer. */
#define UART_RXDMA_BUFSIZE 1024
char g_UARTDMABuffer[UART_RXDMA_BUFSIZE] __attribute__((section(".dma_buffer")));

int32_t g_RxDMACmplt = 0;

UsartWithBuffer::UsartWithBuffer(UART_HandleTypeDef* uart, int _baud)
{
  uart_ = uart;
  CO_fifo_init(&rxBuffer_, g_RxFifoBuf, RXFIFO_BUFSIZE);
}

void UsartWithBuffer::init()
{
  HAL_UART_Receive_DMA(uart_, (uint8_t*)g_UARTDMABuffer, UART_RXDMA_BUFSIZE);
  g_RxDMACmplt = 0;
}

bool UsartWithBuffer::putdata(uint8_t* data, size_t len)
{
  while(uart_->gState != HAL_UART_STATE_READY) {}

  memcpy(g_UARTTxDMABuf, data, len);
  HAL_UART_Transmit_DMA(uart_, (uint8_t*)g_UARTTxDMABuf, len);

  return true;
}

bool UsartWithBuffer::dataAvailable()
{
  return true;
}

/* Upgrade the getch
 * 1. Add a check dma frequence control
 * 2. Extend the fifo rx buffer size
 * 3. Use circular DMA mode
 * */
#define CHECK_DMA_FREQ  4

int32_t UsartWithBuffer::getch()
{
	static uint32_t lastReadPos = 0;
	char ch  = 0;

	static unsigned char checked = 0;

	bool_t getc_result = CO_fifo_getc(&rxBuffer_, &ch);
	if (++checked < CHECK_DMA_FREQ && getc_result) {
		return 0x00000000 | ch;
	}

	checked = 0;
	/*check the rx dma buffer*/

	__disable_irq();
	uint32_t rxBufPos = UART_RXDMA_BUFSIZE - __HAL_DMA_GET_COUNTER(uart_->hdmarx);
	__enable_irq();

	if (rxBufPos != lastReadPos) {
      if (rxBufPos > lastReadPos) {
        uint32_t count = rxBufPos - lastReadPos;
        CO_fifo_write(&rxBuffer_, (const char*)&g_UARTDMABuffer[lastReadPos], count, NULL);
      }else{
        /*Over run received*/
    	uint32_t count = UART_RXDMA_BUFSIZE - lastReadPos;
    	CO_fifo_write(&rxBuffer_, (const char*)&g_UARTDMABuffer[lastReadPos], count, NULL);

    	if (rxBufPos > 0){
    	  CO_fifo_write(&rxBuffer_, (const char*)&g_UARTDMABuffer[0], rxBufPos, NULL);
    	}
      }
      lastReadPos = rxBufPos;

      if (!getc_result){
        /*re-pick char from fifo buffer*/
    	getc_result = CO_fifo_getc(&rxBuffer_, &ch);
      }
    }

	if (lastReadPos == UART_RXDMA_BUFSIZE) {
		lastReadPos = 0;
	}

	if (getc_result)
	  return 0x00000000 | ch;

	/*Empty fifo and DMA buffer*/
	return -1;
}

/*
int32_t UsartWithBuffer::getch()
{
  static uint32_t lastFlushPtr = 0;
  int ret = -1;
  char ch = 0;

  if (!CO_fifo_getc(&rxBuffer_, &ch)){

    __disable_irq();
    uint32_t rxBufLen = UART_RXDMA_BUFSIZE - __HAL_DMA_GET_COUNTER(uart_->hdmarx);
    __enable_irq();

    if (rxBufLen > 0) {
      if (rxBufLen > lastFlushPtr) {
        // write data into rx fifo
        uint32_t count = rxBufLen - lastFlushPtr;
        CO_fifo_write(&rxBuffer_, (const char*)&g_UARTDMABuffer[lastFlushPtr], count, NULL);
        lastFlushPtr = rxBufLen;

        //re-read one char
        CO_fifo_getc(&rxBuffer_, &ch);
        ret = 0x0000000 | ch;
      }
    }

    bool dmaRestart = false;

    __disable_irq();
    if (rxBufLen == UART_RXDMA_BUFSIZE || rxBufLen == 0) {
      //DMA receive completed
      //Restart DMA receive
      if (g_RxDMACmplt && (uart_->RxState == HAL_UART_STATE_READY && HAL_IS_BIT_CLR(uart_->Instance->CR3, USART_CR3_DMAR))) {
        dmaRestart = true;
      }
    }
    __enable_irq();

    if (dmaRestart) {
	  HAL_UART_Receive_DMA(uart_, (uint8_t*)g_UARTDMABuffer, UART_RXDMA_BUFSIZE);
	  g_RxDMACmplt = 0;
	  lastFlushPtr = 0;
    }

  }else{
	ret = 0x00000000 | ch;
  }

  return ret;
}
*/

void UsartWithBuffer::onInterrupt(UART_HandleTypeDef *huart, UsartEventType intType)
{
	if(intType == UsartEventType::EVENT_TRANSMIT_COMPLETE) {

	}else if (intType == UsartEventType::EVENT_RECEIVE) {

	}
}

void UsartWithBuffer::onRxInterrupt(UART_HandleTypeDef *huart, char* buffer, size_t count)
{
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART4) {
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  g_RxDMACmplt = 1;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->ErrorCode & HAL_UART_ERROR_ORE) {
	/*work around, STM32 UART Over run error recover*/

	volatile uint8_t sysRxChar; // clear the regular interrupt
    sysRxChar = (uint8_t) huart->Instance->RDR; // not sure whether it is correct, but it does not work
    (void) sysRxChar;

    __HAL_UART_CLEAR_OREFLAG(huart);
    huart->ErrorCode |= HAL_UART_ERROR_ORE;
    g_RxDMACmplt = 1;
  }
}

void HAL_UART_IdleCallback(UART_HandleTypeDef *huart)
{
	volatile uint32_t temp;

	if(huart->Instance == UART4 && __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET) {
		__HAL_UART_CLEAR_IDLEFLAG(huart);

		temp = huart->Instance->ISR; // read isr can clear status register
		temp = huart->Instance->RDR;
		(void) temp;
	}
}
