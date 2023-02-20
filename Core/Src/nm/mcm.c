/*
 * mcm.c
 *
 *  Created on: 2020年9月27日
 *      Author: Administrator
 */

/* Motion/Motor control management */
#include <stdio.h>
#include "FreeRTOS.h"

#include "301/CO_driver.h"
#include "301\CO_SDOserver.h"
#include "301\CO_Emergency.h"
#include "301\CO_NMT_Heartbeat.h"
#include "301\CO_HBconsumer.h"

#include "CO_OD.h"
#include "CANopen.h"
#include "c402def.h"
#include "dsstate.h"

#include "bm.h"
#include "pdom.h"
#include "mcm.h"
#include "log.h"

MCM_DriverData_t g_MotorDriverData[NUM_NODES];

CO_OD_entryRecord_t OD_record6041[1] = {
		{(void*)&g_MotorDriverData[0].statusWord, 0xFE, 2}
};

CO_OD_entryRecord_t OD_record6061[1] = {
		{(void*)&g_MotorDriverData[0].mode, 0xFE, 1}
};

CO_OD_entryRecord_t OD_record6063[1] = {
		{(void*)&g_MotorDriverData[0].currentPos, 0xFE,4}
};

CO_OD_entryRecord_t OD_record606C[1] = {
		{(void*)&g_MotorDriverData[0].currentVel, 0xFE,4}
};


void MCM_OnNodeReset(uint8_t nodeId, uint8_t monitoredIdx)
{
	g_MotorDriverData[monitoredIdx].mcStatus = DS402_Status_SwitchONDisabled;
	g_MotorDriverData[monitoredIdx].nmtState = CO_NMT_INITIALIZING;

	g_MotorDriverData[monitoredIdx].controlWord = 0;
	g_MotorDriverData[monitoredIdx].statusWord = 0;
	g_MotorDriverData[monitoredIdx].mode = DS402_OperMode_NoMode;
	g_MotorDriverData[monitoredIdx].displayMode = DS402_OperMode_NoMode;

	g_MotorDriverData[monitoredIdx].posDriveEnable = 0;  // can set target position
	g_MotorDriverData[monitoredIdx].homeFinished = 0;    // does the home phase finished

	g_MotorDriverData[monitoredIdx].nodeId = nodeId;

	PDOM_resetCache(monitoredIdx);
}

/* Initialize motor driver data
 * */
void MCM_Init()
{
	uint8_t idx = 0;
	for (; idx < NUM_NODES; idx++) {
		memset(&g_MotorDriverData[idx], 0, sizeof(MCM_DriverData_t));

		MCM_OnNodeReset(0, idx);
		g_MotorDriverData[idx].mcStatus = DS402_Status_NotReadyToSwitchON;
		g_MotorDriverData[idx].nodeId = CO->HBcons->monitoredNodes[idx].nodeId;
	}
}

void MCM_setupNodesRPDOs()
{
    /* RPDO , number of RPDO = 6(number nodes) X 3 channels
     * CO_NO_RPDO = 6 * 3
     * 6041|6061, 6063(Actual pos), 606C(Actual vel)*/

	CO_ReturnError_t err;

    for (uint8_t n = 0; n < CO_NO_HB_CONS; n++) {

    	uint8_t nodeId = (OD_consumerHeartbeatTime[n] >> 16U) & 0xFFU;

    	/*Setting the RPDO mapping*/
    	OD_record6041[0].pData = (void*)&(g_MotorDriverData[n].statusWord);
    	OD_record6061[0].pData = (void*)&(g_MotorDriverData[n].displayMode);
    	OD_record6063[0].pData = (void*)&(g_MotorDriverData[n].currentPos);
    	OD_record606C[0].pData = (void*)&(g_MotorDriverData[n].currentVel);

    	for (uint8_t i = 0; i < 3; i++) {
    		CO_CANmodule_t *CANdevRx = CO->CANmodule[0];
    		uint16_t CANdevRxIdx = CO_RXCAN_RPDO + n*3 + i;

    		err = CO_RPDO_init(CO->RPDO[n*3 + i],
                           CO->em,
                           CO->SDO[0],
                           (void *)CO->SYNC,
                           &CO->NMT->operatingState,
						   nodeId,
                           ((i < 4) ? (CO_CAN_ID_RPDO_1 + i * 0x100) : 0),
                           0,
                           (CO_RPDOCommPar_t*)&OD_RPDOCommunicationParameter[i],
                           (CO_RPDOMapPar_t *)&OD_RPDOMappingParameter[i],
                           OD_H1400_RXPDO_1_PARAM + i,
                           OD_H1600_RXPDO_1_MAPPING + i,
                           CANdevRx,
                           CANdevRxIdx);

    		if (err) break;
    	}
    }
}

void MCM_updateNodeActualValues(char* msg, size_t len)
{
	uint8_t nodeIdx = 0; // the 1st joint

	uint16_t flag = 0;
	flag |= 1 << (2 * nodeIdx + 1);  // position
	flag |= 0 << (2 * nodeIdx);      // velocity

	nodeIdx = 1;  // 2nd joint
	flag |= 1 << (2 * nodeIdx + 1);  // position

	// encode to message, total 6 drivers
	snprintf(msg, len, "%04x%08lx%08lx%08lx%08lx%08lx%08lx%08lx%08lx%08lx%08lx%08lx%08lx", flag,
			g_MotorDriverData[0].currentPos, g_MotorDriverData[0].currentVel,
			g_MotorDriverData[1].currentPos, g_MotorDriverData[1].currentVel,
			g_MotorDriverData[2].currentPos, g_MotorDriverData[2].currentVel,
			g_MotorDriverData[3].currentPos, g_MotorDriverData[3].currentVel,
			g_MotorDriverData[4].currentPos, g_MotorDriverData[4].currentVel,
			g_MotorDriverData[5].currentPos, g_MotorDriverData[5].currentVel);
}

void MCM_sendCommand(uint8_t nodeId, uint8_t monitoredIdx, DS402_Command_t command)
{
	/* Build control word*/
	uint16_t controlWord = DS402_buildControlWordByCommand(g_MotorDriverData[monitoredIdx].controlWord, command);

	/*Send */
	MTPDO_map_item_t item;
	item.nodeId = nodeId;
	item.tpdoIdx = 0;	// 6040 control word
	memcpy((void*)item.data, &controlWord, sizeof(uint16_t));
	memcpy((void*)&item.data[2], (void*)&g_MotorDriverData[monitoredIdx].mode, sizeof(int8_t));

	PDOM_sendItem(&item);
}

/*
 * Operational:
 * Master sends operation Shutdown to exit state Switch on Disabled
 *  */
static void MCM_PowerEnable(uint8_t nodeId, uint8_t monitoredIdx, DS402_Status_t status)
{
	if (status == DS402_Status_SwitchONDisabled) {
		/*Send shutdown command*/
		MCM_sendCommand(nodeId, monitoredIdx, DS402_Command_Shutdown);
	}else if (status == DS402_Status_ReadyToSwitchON) {
		MCM_sendCommand(nodeId, monitoredIdx, DS402_Command_SwitchON);
	}else if (status == DS402_Status_SwitchedON) {
		MCM_sendCommand(nodeId, monitoredIdx, DS402_Command_EnableOperation);
	}
}

void MCM_OnNodeModeUpdate(uint8_t nodeId, uint8_t monitoredIdx, MCM_Status_t mcmStatus)
{

}

static void MCM_OnNodeStatusWordUpdate(uint8_t nodeId, uint8_t monitoredIdx, MCM_Status_t mcmStatus)
{
	uint16_t statusWord = g_MotorDriverData[monitoredIdx].statusWord;
	DS402_Status_t status = DS402_getStatus(statusWord);

	if (mcmStatus == MCM_NODE_PowerEnabled) {
		if (status != DS402_Status_OperationEnable) {
			MCM_PowerEnable(nodeId, monitoredIdx, status);
			return;
		}
	}
}

static void MCM_processRPDO0(CO_RPDO_t *RPDO, uint8_t monitoredIdx, MCM_Status_t mcmStatus)
{
	if (!RPDO->valid || !(*RPDO->operatingState == CO_NMT_OPERATIONAL)) {
        CO_FLAG_CLEAR(RPDO->CANrxNew[0]);
        return;
    }

	uint8_t update = 0;
	uint8_t bufNo = 0;
	while(CO_FLAG_READ(RPDO->CANrxNew[bufNo])){
	    int16_t i;
	    uint8_t* pPDOdataByte;
	    uint8_t** ppODdataByte;

	    i = RPDO->dataLength;
	    pPDOdataByte = &RPDO->CANrxData[bufNo][0];
	    ppODdataByte = &RPDO->mapPointer[0];

	    /* Copy data to Object dictionary. If between the copy operation CANrxNew
	     * is set to true by receive thread, then copy the latest data again. */
	    CO_FLAG_CLEAR(RPDO->CANrxNew[bufNo]);
	    for(; i>0; i--) {
	        **(ppODdataByte++) = *(pPDOdataByte++);
	    }

	    update = 1;
    }

	if (update) {
		MCM_OnNodeStatusWordUpdate(RPDO->nodeId, monitoredIdx, mcmStatus);
	}
}

static void MCM_processMode(uint8_t nodeId, uint8_t monitoredIdx, MCM_Status_t mcmStatus)
{
	uint16_t statusWord = g_MotorDriverData[monitoredIdx].statusWord;
	DS402_Status_t status = DS402_getStatus(statusWord);

	if (mcmStatus == MCM_NODE_PowerEnabled) {
		if (status == DS402_Status_OperationEnable) {

			switch (g_MotorDriverData[monitoredIdx].displayMode) {
			case DS402_OperMode_NoMode:{
				/*开启 home mode*/
				g_MotorDriverData[monitoredIdx].posDriveEnable = 0;
				g_MotorDriverData[monitoredIdx].mode = DS402_OperMode_Homing;
				MCM_sendCommand(nodeId, monitoredIdx, DS402_Command_EnableOperation);
			}
			break;

			/* Remote模式和Host的模式命令同步后*/
			case DS402_OperMode_Homing: {

				g_MotorDriverData[monitoredIdx].posDriveEnable = 0;

				/*display mode有滞后问题，需要先判断mode 命令*/
				if (g_MotorDriverData[monitoredIdx].mode == DS402_OperMode_Homing) {

					/*启动Home模式*/
					if ((g_MotorDriverData[monitoredIdx].controlWord & 0x0010) != 0x0010){
						/*Start the home operation bit*/
						g_MotorDriverData[monitoredIdx].controlWord |= 0x1F;
						MCM_sendCommand(nodeId, monitoredIdx, DS402_Command_EnableOperation);
						return;
					}

					/*检测Bit10和Bit12， 当到达home offset后，启动PP模式*/
					if ((statusWord & 0x1400) == 0x1400) {
						/*Homing, Read the reach point bit,
						 * Bit 10 (Stopped)=1 and either: Bit 12 (Homing attained)=1 or Bit 13 (Homing Error)=1*/
						g_MotorDriverData[monitoredIdx].mode = DS402_OperMode_ProfilePosition;

						g_MotorDriverData[monitoredIdx].homeFinished = 1;

						/*De-assert start bit (set bit 4 low)*/
						g_MotorDriverData[monitoredIdx].controlWord &= ~0x1F;
						g_MotorDriverData[monitoredIdx].controlWord |= 0x0F;
						MCM_sendCommand(nodeId, monitoredIdx, DS402_Command_EnableOperation);
						return;
					}
				}
			}break;

			case DS402_OperMode_ProfilePosition: {

				/*Read bit10 ,.......bit11 bit12*/
				/*bit12:0, set the point, send 0x607a target point, send control command 0x21f*/
				/*bit12:1, point ack, send control command, 0x0f*/
				/*         read the bit10, if target point reached*/

				if ((statusWord & (1<<12)) != 0) {

					if ((g_MotorDriverData[monitoredIdx].controlWord & 0x210) != 0) {

						/*De-assert start bit (set bit 4 low)*/
						g_MotorDriverData[monitoredIdx].controlWord &= ~0x210;
						g_MotorDriverData[monitoredIdx].controlWord |= 0x00F;
						MCM_sendCommand(nodeId, monitoredIdx, DS402_Command_EnableOperation);
					}

				}else {

					if ((g_MotorDriverData[monitoredIdx].controlWord & 0x21F) != 0x21F) {
						/*依据node point queue模式*/
						/* Complex queue mode:  control | 0x21f
						 * Queue mode:          control | 0x01f
						 * Bit 9:  (0) Complete current set point before evaluating next (come to stop),
							   	   (1) go to current set point at velocity, then transition to new accel/velocity
							       	   to reach next setpoint.*/
						g_MotorDriverData[monitoredIdx].controlWord |= 0x21F;
						MCM_sendCommand(nodeId, monitoredIdx, DS402_Command_EnableOperation);
					}
				}

				g_MotorDriverData[monitoredIdx].posDriveEnable = 1;

			}break;

			default:
				break;
			}/*end switch*/
		}
	}
}

static void MCM_processRPDO1(CO_RPDO_t *RPDO, uint8_t monitoredIdx, MCM_Status_t mcmStatus)
{
	CO_RPDO_process(RPDO, false);
}

static void MCM_processRPDO2(CO_RPDO_t *RPDO, uint8_t monitoredIdx, MCM_Status_t mcmStatus)
{
	CO_RPDO_process(RPDO, false);
}

/* ===================================================
 * Motion control / status process, handle by nm task
 * =================================================== */
void MCM_process(uint8_t nodeId, uint8_t monitoredIdx, MCM_Status_t mcmStatus)
{
	/*Motor control manage node running mode*/
	MCM_processMode(nodeId, monitoredIdx, mcmStatus);

	/*Motor control manage status, mode RPDO*/
   	MCM_processRPDO0(CO->RPDO[monitoredIdx*3], monitoredIdx, mcmStatus);

   	/*Motor control manage position, velocity RPDO*/
   	MCM_processRPDO1(CO->RPDO[monitoredIdx*3+1], monitoredIdx, mcmStatus);
   	MCM_processRPDO2(CO->RPDO[monitoredIdx*3+2], monitoredIdx, mcmStatus);
}

