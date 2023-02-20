/*
 * CANStm32.c
 *
 *  Created on: 2020��5��21��
 *      Author: Light.Huang
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "stm32h7xx_hal.h"

#include "CANopen.h"
#include "CO_driver_target.h"
#include "CO_OD.h"

static FDCAN_TxHeaderTypeDef TxHeader;

static void STM_PrepareTxHeader(FDCAN_TxHeaderTypeDef* txHeader, uint32_t ident, uint32_t len)
{
	/* Prepare Tx Header */
	txHeader->Identifier = (ident >> 2) & 0x7FF;
	txHeader->IdType = FDCAN_STANDARD_ID;

	/*RTR
	if (txHeader->Identifier & 0x02)
		txHeader->TxFrameType = FDCAN_REMOTE_FRAME;
	else
		txHeader->TxFrameType = FDCAN_DATA_FRAME;
*/
	txHeader->TxFrameType = FDCAN_DATA_FRAME;
	/*FDCan data length convert*/
	txHeader->DataLength = len << 16;
	txHeader->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txHeader->BitRateSwitch = FDCAN_BRS_OFF;
	txHeader->FDFormat = FDCAN_CLASSIC_CAN;
	txHeader->TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	txHeader->MessageMarker = 0;
}

/******************************************************************************/
void CO_CANsetConfigurationMode(void *CANptr){
    /* Put CAN module in configuration mode */
}


/******************************************************************************/
void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule){
    /* Put CAN module in normal mode */
  if (HAL_FDCAN_ActivateNotification((FDCAN_HandleTypeDef*)CANmodule->CANptr,
		  //FDCAN_IT_RX_BUFFER_NEW_MESSAGE |
		  FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
    CANmodule->CANnormal = false;
  }

  if (HAL_FDCAN_Start((FDCAN_HandleTypeDef*)CANmodule->CANptr) != HAL_OK){
	CANmodule->CANnormal = false;
  }else{
    CANmodule->CANnormal = true;
  }
}


/******************************************************************************/
CO_ReturnError_t CO_CANmodule_init(
        CO_CANmodule_t         *CANmodule,
        void                   *CANptr,
        CO_CANrx_t              rxArray[],
        uint16_t                rxSize,
        CO_CANtx_t              txArray[],
        uint16_t                txSize,
        uint16_t                CANbitRate)
{
    uint16_t i;

    /* verify arguments */
    if(CANmodule==NULL || rxArray==NULL || txArray==NULL){
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* Configure object variables */
    CANmodule->CANptr = CANptr;
    CANmodule->rxArray = rxArray;
    CANmodule->rxSize = rxSize;
    CANmodule->txArray = txArray;
    CANmodule->txSize = txSize;
    CANmodule->CANnormal = false;
    CANmodule->useCANrxFilters = false;/* microcontroller dependent */
    CANmodule->bufferInhibitFlag = false;
    CANmodule->firstCANtxMessage = true;
    CANmodule->CANtxCount = 0U;
    CANmodule->errOld = 0U;
    CANmodule->em = NULL;

    for(i=0U; i<rxSize; i++){
        rxArray[i].ident = 0U;
        rxArray[i].mask = 0xFFFFU;
        rxArray[i].object = NULL;
        rxArray[i].CANrx_callback = NULL;
    }

    for(i=0U; i<txSize; i++){
        txArray[i].bufferFull = false;
    }

    /*STM32 CAN initialize*/
    CO_CANmodule_disable(CANmodule);
    HAL_FDCAN_MspDeInit((FDCAN_HandleTypeDef*)CANmodule->CANptr);
    HAL_FDCAN_MspInit((FDCAN_HandleTypeDef*)CANmodule->CANptr);

    uint32_t Prescaler;

    switch(CANbitRate){
    case 1000: Prescaler = 1; break;
    case 500:  Prescaler = 2; break;
    case 250:  Prescaler = 4; break;   // default is 8
    case 125:  Prescaler = 8; break;
    case 100:  Prescaler = 16; break;
    case 50:   Prescaler = 32; break;
    case 20:   Prescaler = 80; break;
    case 10:   Prescaler = 160; break;
    default:
    	Prescaler = 8;
    }

    /* Bit time configuration:
        fdcan_ker_ck               = 40 MHz    // from MX clock configuration
        Time_quantum (tq)          = 100 ns     // tq = NominalPrescaler x (1/fdcan_ker_ck), NominalPrescaler = 4
        Synchronization_segment    = 1 tq
        Propagation_segment        = 23 tq
        Phase_segment_1            = 8 tq
        Phase_segment_2            = 8 tq
        Synchronization_Jump_width = 8 tq
        Bit_length                 = 40 tq = 4 µs
        Bit_rate                   = 250 KBit/s
      */

    /* Set the CAN2.0 max baudrate 1MBit/s*/
    /* Version 1: set FDCAN baudrate = 500KB, suitful for CAN2.0 device*/

    FDCAN_HandleTypeDef* fdcanptr;
    fdcanptr = (FDCAN_HandleTypeDef*)CANmodule->CANptr;
    fdcanptr->Instance = FDCAN2;
    fdcanptr->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    fdcanptr->Init.Mode = FDCAN_MODE_NORMAL;

    fdcanptr->Init.AutoRetransmission = ENABLE;
    fdcanptr->Init.TransmitPause = DISABLE;
    fdcanptr->Init.ProtocolException = DISABLE;
    fdcanptr->Init.NominalPrescaler = 2; //4; //Prescaler;
    fdcanptr->Init.NominalSyncJumpWidth =8;
    fdcanptr->Init.NominalTimeSeg1 = 31; //14; //31;          /* NominalTimeSeg1 = Propagation_segment + Phase_segment_1 */
    fdcanptr->Init.NominalTimeSeg2 = 8; //5; //8;

    // V0: 31,8
    // V1: 33,6
    // V2: 10,1

    fdcanptr->Init.DataPrescaler = 2; //4; //Prescaler;
    fdcanptr->Init.DataSyncJumpWidth = 8;
    fdcanptr->Init.DataTimeSeg1 = 31; //14; //31;
    fdcanptr->Init.DataTimeSeg2 = 8; //5; //8;

    fdcanptr->Init.MessageRAMOffset = 0;
    fdcanptr->Init.StdFiltersNbr = 1;
    fdcanptr->Init.ExtFiltersNbr = 0;

    fdcanptr->Init.RxFifo0ElmtsNbr = 1;
    fdcanptr->Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
    fdcanptr->Init.RxFifo1ElmtsNbr = 0;
    fdcanptr->Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
    fdcanptr->Init.RxBuffersNbr = 0;
    fdcanptr->Init.RxBufferSize = FDCAN_DATA_BYTES_8;

    fdcanptr->Init.TxEventsNbr = 0;
    fdcanptr->Init.TxBuffersNbr = 0;
    fdcanptr->Init.TxFifoQueueElmtsNbr = 1;
    fdcanptr->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    fdcanptr->Init.TxElmtSize = FDCAN_DATA_BYTES_8;

    if (HAL_FDCAN_Init(fdcanptr) != HAL_OK){
    	CANmodule->CANnormal = false;
    	return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    FDCAN_FilterTypeDef sFilterConfig;

    /* Configure Rx filter */
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;

    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0;
    sFilterConfig.FilterID2 = 0;
    sFilterConfig.RxBufferIndex = 0;

    if (HAL_FDCAN_ConfigFilter(fdcanptr, &sFilterConfig) != HAL_OK) {
      /* Filter configuration Error */
    }

    /* Configure global filter to reject all non-matching frames */
    HAL_FDCAN_ConfigGlobalFilter(fdcanptr, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    return CO_ERROR_NO;
}


/******************************************************************************/
void CO_CANmodule_disable(CO_CANmodule_t *CANmodule){
    /* turn off the module */
  CANmodule->CANnormal = false;
  HAL_FDCAN_DeactivateNotification((FDCAN_HandleTypeDef*)CANmodule->CANptr,
		  	  	  FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
		  		  FDCAN_IT_RX_FIFO1_NEW_MESSAGE |
		  		  FDCAN_IT_TX_FIFO_EMPTY);
  HAL_FDCAN_Stop((FDCAN_HandleTypeDef*)CANmodule->CANptr);
}

/******************************************************************************/
CO_ReturnError_t CO_CANrxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        uint16_t                mask,
        bool_t                  rtr,
        void                   *object,
        void                  (*CANrx_callback)(void *object, void *message))
{
    CO_ReturnError_t ret = CO_ERROR_NO;

    if((CANmodule!=NULL) && (object!=NULL) && (CANrx_callback!=NULL) && (index < CANmodule->rxSize)){
        /* buffer, which will be configured */
        CO_CANrx_t *buffer = &CANmodule->rxArray[index];

        //printf("RxBuffer init, index(%d) ident: %d, mask: %d \n", index, ident, mask);

        /* Configure object variables */
        buffer->object = object;
        buffer->CANrx_callback = CANrx_callback;

        /* CAN identifier and CAN mask, bit aligned with CAN module. Different on different microcontrollers. */
        buffer->ident = ident << 2;
        if (rtr) buffer->ident |= 0x02;
        buffer->mask = ((mask & 0x07FF) << 2) | 0x02;

        /* Set CAN hardware module filter and mask. */
        if(CANmodule->useCANrxFilters){

        }
    }
    else{
        ret = CO_ERROR_ILLEGAL_ARGUMENT;
    }

    return ret;
}


/******************************************************************************/
CO_CANtx_t *CO_CANtxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        bool_t                  rtr,
        uint8_t                 noOfBytes,
        bool_t                  syncFlag)
{
    CO_CANtx_t *buffer = NULL;

    if((CANmodule != NULL) && (index < CANmodule->txSize)){
        /* get specific buffer */
        buffer = &CANmodule->txArray[index];

        /* CAN identifier, DLC and rtr, bit aligned with CAN module transmit buffer.
         * Microcontroller specific. */

        buffer->ident = (ident & 0x07FF) << 2;
        if (rtr) buffer->ident |= 0x02;

        buffer->DLC = noOfBytes;

        buffer->bufferFull = false;
        buffer->syncFlag = syncFlag;
    }

    return buffer;
}

/******************************************************************************/
CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer){
    CO_ReturnError_t err = CO_ERROR_NO;

    /* Verify overflow */
    if(buffer->bufferFull){
        if(!CANmodule->firstCANtxMessage){
            /* don't set error, if bootup message is still on buffers */
            CO_errorReport((CO_EM_t*)CANmodule->em, CO_EM_CAN_TX_OVERFLOW, CO_EMC_CAN_OVERRUN, buffer->ident);
        }
        err = CO_ERROR_TX_OVERFLOW;
    }

    CO_LOCK_CAN_SEND();

    /* if CAN TX buffer is free, copy message to it */
    if(CANmodule->CANtxCount == 0 && (HAL_FDCAN_GetTxFifoFreeLevel((FDCAN_HandleTypeDef*)CANmodule->CANptr) > 0)){
        CANmodule->bufferInhibitFlag = buffer->syncFlag;
        /* copy message and txRequest */

        STM_PrepareTxHeader(&TxHeader, buffer->ident, buffer->DLC);

		if (HAL_FDCAN_AddMessageToTxFifoQ((FDCAN_HandleTypeDef*)CANmodule->CANptr, &TxHeader, &buffer->data[0]) != HAL_OK){
          err = CO_ERROR_TX_OVERFLOW;
        }else{
          // do nothing
        }

    }
    /* if no buffer is free, message will be sent by interrupt */
    else{
        buffer->bufferFull = true;
        CANmodule->CANtxCount++;
    }
    CO_UNLOCK_CAN_SEND();

    return err;
}

void CO_CANpolling_Tx(CO_CANmodule_t *CANmodule)
{
	/*No pending tx msg*/
	if (HAL_FDCAN_GetTxFifoFreeLevel((FDCAN_HandleTypeDef*)CANmodule->CANptr) == 0)
		return;

	/* First CAN message (bootup) was sent successfully */
	CANmodule->firstCANtxMessage = false;

	/* Clear flag from previous message */
	CANmodule->bufferInhibitFlag = false;

	/* Are there any new messages waiting to be send */
	if(CANmodule->CANtxCount > 0U) {
		uint16_t i;             /* index of transmitting message */

		/* first buffer */
		CO_CANtx_t *buffer = &CANmodule->txArray[0];

		/* search through whole array of pointers to transmit message buffers. */
		for(i = CANmodule->txSize; i > 0U; i--) {
			/* if message buffer is full, send it. */
			if(buffer->bufferFull) {

				/* Copy message to CAN buffer */
				CANmodule->bufferInhibitFlag = buffer->syncFlag;

		        STM_PrepareTxHeader(&TxHeader, buffer->ident, buffer->DLC);

		        if (HAL_FDCAN_AddMessageToTxFifoQ((FDCAN_HandleTypeDef*)CANmodule->CANptr, &TxHeader, &buffer->data[0]) != HAL_OK){

		        }else{
                	buffer->bufferFull = false;
                	CANmodule->CANtxCount--;
                }

				break;                      /* exit for loop */
			} else {
				/*do nothing*/;
			}
			buffer++;
		}/* end of for loop */

		/* Clear counter if no more messages */
		if(i == 0U) {
			CANmodule->CANtxCount = 0U;
		}else {
			/*do nothing*/;
		}
	}
}

/******************************************************************************/
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule){
    uint32_t tpdoDeleted = 0U;

    CO_LOCK_CAN_SEND();
    /* Abort message from CAN module, if there is synchronous TPDO.
     * Take special care with this functionality. */
    if(/*messageIsOnCanBuffer && */CANmodule->bufferInhibitFlag){
        /* clear TXREQ */
        CANmodule->bufferInhibitFlag = false;
        tpdoDeleted = 1U;
    }
    /* delete also pending synchronous TPDOs in TX buffers */
    if(CANmodule->CANtxCount != 0U){
        uint16_t i;
        CO_CANtx_t *buffer = &CANmodule->txArray[0];
        for(i = CANmodule->txSize; i > 0U; i--){
            if(buffer->bufferFull){
                if(buffer->syncFlag){
                    buffer->bufferFull = false;
                    CANmodule->CANtxCount--;
                    tpdoDeleted = 2U;
                }
            }
            buffer++;
        }
    }
    CO_UNLOCK_CAN_SEND();


    if(tpdoDeleted != 0U){
        CO_errorReport((CO_EM_t*)CANmodule->em, CO_EM_TPDO_OUTSIDE_WINDOW, CO_EMC_COMMUNICATION, tpdoDeleted);
    }
}


/******************************************************************************/
void CO_CANverifyErrors(CO_CANmodule_t *CANmodule)
{
    CO_EM_t* em = (CO_EM_t*)CANmodule->em;

    /* get error counters from module. Id possible, function may use different way to
     * determine errors. */
    uint32_t err = HAL_FDCAN_GetError((FDCAN_HandleTypeDef*)(CANmodule->CANptr));

    if(CANmodule->errOld != err){
        CANmodule->errOld = err;

        if(err & FDCAN_IR_BO){
        	/* bus off, 离线状态 */
            CO_errorReport(em, CO_EM_CAN_TX_BUS_OFF, CO_EMC_BUS_OFF_RECOVERED, err);
        }
        else{
        	/* not bus off, 在线恢复 */
            CO_errorReset(em, CO_EM_CAN_TX_BUS_OFF, err);

            if(err & FDCAN_IR_EW){
            	/* bus warning */
                CO_errorReport(em, CO_EM_CAN_BUS_WARNING, CO_EMC_NO_ERROR, err);
            }

            if(err & FDCAN_IR_EP){
            	/* RX/TX bus passive, 被动错误 */
                CO_errorReport(em, CO_EM_CAN_RX_BUS_PASSIVE, CO_EMC_CAN_PASSIVE, err);

                if(!CANmodule->firstCANtxMessage){
                	CO_errorReport(em, CO_EM_CAN_TX_BUS_PASSIVE, CO_EMC_CAN_PASSIVE, err);
                }
            }
            else{
                CO_errorReset(em, CO_EM_CAN_RX_BUS_PASSIVE, err);

                bool_t isError = CO_isError(em, CO_EM_CAN_TX_BUS_PASSIVE);
                if(isError){
                    CO_errorReset(em, CO_EM_CAN_TX_BUS_PASSIVE, err);
                    CO_errorReset(em, CO_EM_CAN_TX_OVERFLOW, err);
                }
            }

            if(err == HAL_FDCAN_ERROR_NONE){       /* no error */
                CO_errorReset(em, CO_EM_CAN_BUS_WARNING, err);
            }
        }

        if(err & FDCAN_IR_ELO){
        	/* CAN RX bus overflow */
        	/* Skip*/
            // CO_errorReport(em, CO_EM_CAN_RXB_OVERFLOW, CO_EMC_CAN_OVERRUN, err);
        }
    }
}

void CO_CANinterrupt(CO_CANmodule_t *CANmodule)
{
    /* receive interrupt */
    if(1){
        CO_CANrxMsg_t rcvMsg;      /* pointer to received message in CAN module */
        uint16_t index;             /* index of received message */

        CO_CANrx_t *buffer = NULL;  /* receive message buffer from CO_CANmodule_t object. */
        bool_t msgMatched = false;

        HAL_FDCAN_GetRxMessage((FDCAN_HandleTypeDef*)CANmodule->CANptr, FDCAN_RX_FIFO0, &rcvMsg.header, &rcvMsg.data[0]);

        rcvMsg.DLC   = (rcvMsg.header.DataLength >> 16);
        rcvMsg.ident = rcvMsg.header.Identifier;

        /*
        uint8_t cob = (rcvMsg.ident & 0xFFFFFF80) >> 7;
        uint8_t nid = (rcvMsg.ident & 0x7F);
        printf("Rcv RX msg, ident: %d, cob: %d, node: %d, dlc: %d, data[0]: 0x%02x\n",
        		rcvMsg.ident,
				cob, nid,
				rcvMsg.DLC,
				rcvMsg.data[0]);
		*/
        /*Expand with CANopen rx/tx buffer setting*/
        rcvMsg.ident <<= 2;
        rcvMsg.ident |= (rcvMsg.header.RxFrameType == FDCAN_REMOTE_FRAME) ? 0x02 : 0x00;

        if(CANmodule->useCANrxFilters){
            /* CAN module filters are used. Message with known 11-bit identifier has */
            /* been received */
            index = 0;  /* get index of the received message here. Or something similar */
            if(index < CANmodule->rxSize){
                buffer = &CANmodule->rxArray[index];
                /* verify also RTR */
                if(((rcvMsg.ident ^ buffer->ident) & buffer->mask) == 0U){
                    msgMatched = true;
                }
            }
        }
        else{
            /* CAN module filters are not used, message with any standard 11-bit identifier */
            /* has been received. Search rxArray form CANmodule for the same CAN-ID. */
            buffer = &CANmodule->rxArray[0];
            for(index = CANmodule->rxSize; index > 0U; index--){
                if(((rcvMsg.ident ^ buffer->ident) & buffer->mask) == 0U){
                    msgMatched = true;
                    break;
                }
                buffer++;
            }
        }

        /* Call specific function, which will process the message */
        if(msgMatched && (buffer != NULL) && (buffer->CANrx_callback != NULL)){
            buffer->CANrx_callback(buffer->object, (void*) &rcvMsg);
        }

        /* Clear interrupt flag */
    }


    /* transmit interrupt */
    else if(0){
        /* Clear interrupt flag */

        /* First CAN message (bootup) was sent successfully */
        CANmodule->firstCANtxMessage = false;
        /* clear flag from previous message */
        CANmodule->bufferInhibitFlag = false;
        /* Are there any new messages waiting to be send */
        if(CANmodule->CANtxCount > 0U){
            uint16_t i;             /* index of transmitting message */

            /* first buffer */
            CO_CANtx_t *buffer = &CANmodule->txArray[0];
            /* search through whole array of pointers to transmit message buffers. */
            for(i = CANmodule->txSize; i > 0U; i--){
                /* if message buffer is full, send it. */
                if(buffer->bufferFull){
                    buffer->bufferFull = false;
                    CANmodule->CANtxCount--;

                    /* Copy message to CAN buffer */
                    CANmodule->bufferInhibitFlag = buffer->syncFlag;
                    /* canSend... */
                    break;                      /* exit for loop */
                }
                buffer++;
            }/* end of for loop */

            /* Clear counter if no more messages */
            if(i == 0U){
                CANmodule->CANtxCount = 0U;
            }
        }
    }
    else{
        /* some other interrupt reason */
    }
}

void HAL_FDCAN_RxBufferNewMessageCallback(FDCAN_HandleTypeDef *hfdcan)
{
	CO_CANinterrupt(CO->CANmodule[0]);
}

void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan)
{

}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
		/* Retreive Rx messages from RX FIFO0 */

		CO_CANinterrupt(CO->CANmodule[0]);

		if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)  {
			/* Notification Error */
		}
	}
}
