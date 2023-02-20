#pragma once

#include "main.h"
#include "UsartWithBuffer.h"

extern UART_HandleTypeDef huart4;

class STM32Hardware {

  using SERIAL_CLASS = UsartWithBuffer;

  public:
  STM32Hardware(SERIAL_CLASS* _com , long baud = 57600){
    com = _com;
    baud_ = baud;
  }

    STM32Hardware()
    {
      com = new SERIAL_CLASS(&huart4, 115200);
      baud_ = 115200;
    }

    STM32Hardware(STM32Hardware& h){
      this->baud_ = h.baud_;
    }

    void setBaud(long baud){
      this->baud_= baud;
    }

    int getBaud(){return baud_;}

    void init(){
      com->init();
    }

    int read(){
      if(com->dataAvailable()){
	    return com->getch();
      }
      else{
	    return -1;
      }
    };

    void write(uint8_t* data, int length){
       com->putdata(data, length);
    }

    void OnTxCpltCallback(UART_HandleTypeDef *huart){
    	com->onInterrupt(huart, UsartEventType::EVENT_TRANSMIT_COMPLETE);
    }

    void OnRxCallback(UART_HandleTypeDef *huart, char* buffer, size_t count){
    	com->onRxInterrupt(huart, buffer, count);
    }

    unsigned long time(){return HAL_GetTick();}

  protected:
    SERIAL_CLASS* com;
    long baud_;
};
