/*
 * mcm.h
 *
 *  Created on: 2020年9月29日
 *      Author: Administrator
 */

#ifndef SRC_NM_MCM_H_
#define SRC_NM_MCM_H_

#include "c402def.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct MCM_DriverData_ {
	volatile uint16_t statusWord;	// last word received from driver, 0x6041
	volatile uint16_t controlWord;   // last word sent to driver
	volatile uint32_t errorCode;     // last error code

	volatile int8_t   mode;	        // mode command
	volatile int8_t   displayMode;   // mode from driver, 0x6061

	volatile int8_t   posDriveEnable;  // can set target position
	volatile int8_t   homeFinished;  // does the home phase finished

	uint32_t currentPos;	// current position, 0x6063
	uint32_t currentVel;    // current velocity, 0x606c

	uint32_t homeOffset;	// home offset position

	CO_NMT_internalState_t nmtState;	// Cia301 nmt state
	DS402_Status_t 		   mcStatus;	// Cia402 status

	int8_t   nodeId;
	uint8_t  homeMethod;	// home method

}MCM_DriverData_t;

typedef enum MCM_Status_ {
	MCM_NODE_PowerDisabled = 0x0,
	MCM_NODE_PowerEnabled,
	MCM_NODE_Fault,
}MCM_Status_t;

void MCM_Init();
void MCM_setupNodesRPDOs();
void MCM_sendCommand(uint8_t monitoredIdx, uint8_t nodeId, DS402_Command_t command);
void MCM_OnNodeReset(uint8_t nodeId, uint8_t monitoredIdx);
void MCM_process(uint8_t nodeId, uint8_t bmIdx, MCM_Status_t mcmStatus);
void MCM_updateNodeActualValues(char* msg, size_t len);
#ifdef __cplusplus
}
#endif

#endif /* SRC_NM_MCM_H_ */
