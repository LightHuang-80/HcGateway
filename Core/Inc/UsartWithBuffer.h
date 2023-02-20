#pragma once

#include <stdlib.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "stm32h7xx_hal.h"
#include "CO_fifo.h"

enum class UsartEventType : uint8_t {
    EVENT_RECEIVE,
    EVENT_READY_TO_TRANSMIT,
    EVENT_TRANSMIT_COMPLETE,
    EVENT_PARITY_ERROR,
    EVENT_IDLE,
    EVENT_LINE_BREAK_DETECT,
    EVENT_ERROR,
    EVENT_CLEAR_TO_SEND,
  };

class UsartWithBuffer {

protected:
  /*
   * Default : The UART4 peripheral configured with the interrupt feature
   */
  UART_HandleTypeDef *uart_;

  CO_fifo_t rxBuffer_;

  const static int UART_BUFSIZE = 1024;

public:

  /*
   * Use the constructor base initialiser to set up the USART at 57600
   */

  UsartWithBuffer(UART_HandleTypeDef* uart, int _baud = 115200);

  void init();

  bool TXBuffer_Empty()
  {
	return false;
  }

  bool TXBuffer_FreeSpace()
  {
	  return true;
  }

  bool putch(uint8_t data)
  {
    putdata(&data, 1);
    return true;
  }

  bool putdata(uint8_t* data, size_t len);

  /* Send a string */
  void puts(char *s)
  {
	  putdata((uint8_t*)s, strlen(s));
  }

  bool dataAvailable();

  int32_t getch();

  /*
   * Interrupt callback function.
   */
  void onInterrupt(UART_HandleTypeDef *huart, UsartEventType intType);

  void onRxInterrupt(UART_HandleTypeDef *huart, char* buffer, size_t count);
};
