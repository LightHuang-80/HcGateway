/*
 * nodemanage.c
 *
 *  Created on: 2020年8月4日
 *      Author: Administrator
 */


/**/

#include <stdio.h>
#include <stdlib.h>

#include "main.h"

#include "CANopen.h"
#include "CO_driver_target.h"
#include "CO_OD.h"
#include "nm.h"
#include "bm.h"
#include "mcm.h"
#include "pdom.h"
#include "log.h"

int  g_NodeId;

/* callback for emergency messages */
static void EmergencyRxCallback(const uint16_t ident,
                                const uint16_t errorCode,
                                const uint8_t errorRegister,
                                const uint8_t errorBit,
                                const uint32_t infoCode)
{
	/*
	int16_t nodeIdRx = ident ? (ident&0x7F) : CO_ownNodeId;

    log_printf(LOG_NOTICE, DBG_EMERGENCY_RX, nodeIdRx, errorCode,
               errorRegister, errorBit, infoCode);
	*/

	// change the node state

}

const char* InitStr = "initializing";
const char* PreStr  = "pre-operational";
const char* OprStr  = "operational";
const char* StpStr  = "stopped";
const char* UknStr  = "unknown";
/* return string description of NMT state. */
static const char *NmtState2Str(CO_NMT_internalState_t state)
{
    switch(state) {
        case CO_NMT_INITIALIZING:    return InitStr;
        case CO_NMT_PRE_OPERATIONAL: return PreStr;
        case CO_NMT_OPERATIONAL:     return OprStr;
        case CO_NMT_STOPPED:         return StpStr;
        default:                     return UknStr;
    }

    return UknStr;
}

/* callback for NMT change messages */
static void NmtChangedCallback(CO_NMT_internalState_t state)
{
	LOG_Print(LOG_InfoLevel, "Local master state changed, state(%d): %s\n", state, NmtState2Str(state));
	if (state == CO_NMT_PRE_OPERATIONAL) {
		printf("Some error happen.\n");
	}
}

static void HeartbeatMonitorCallback(void* object)
{
}

static void RemoteNodeResetCallback(uint8_t nodeId, uint8_t idx, void *object)
{
	CO_NMT_internalState_t state;
	CO_HBconsumer_getNmtState(CO->HBcons, idx, &state);

	LOG_Print(LOG_InfoLevel, "Remote node(%d) reset, state(%d): %s\n", nodeId, state, NmtState2Str(state));

	BM_OnNodeNMTStateChanged(nodeId, idx, state, CO_HBconsumer_UNKNOWN);
}

static void HeartbeatTimeoutCallback(uint8_t nodeId, uint8_t idx, void *object)
{
	CO_NMT_internalState_t state;
	CO_HBconsumer_getNmtState(CO->HBcons, idx, &state);

	BM_OnNodeNMTStateChanged(nodeId, idx, state, CO_HBconsumer_TIMEOUT);
}

static void HeartbeatStartCallback(uint8_t nodeId, uint8_t idx, void *object)
{

}

/* callback for monitoring Heartbeat remote NMT state change */
static void HeartbeatNmtChangedCallback(uint8_t nodeId,
										uint8_t monitoredIdx,
                                        CO_NMT_internalState_t state,
                                        void *object)
{
	CO_HBconsumer_state_t hbState = CO_HBconsumer_getState(CO->HBcons, monitoredIdx);

	//printf("node(%d) Heartbeat nmt changed, state(%d), hbstate(%d)\n", nodeId, state, hbState);
	/*State changed in bundle resource management*/
	BM_OnNodeNMTStateChanged(nodeId, monitoredIdx, state, hbState);
}

uint8_t NM_New()
{
	uint8_t result = 0;
	CO_ReturnError_t err;

	/* Allocate memory for CANopen objects */
	err = CO_new(NULL);
	if (err != CO_ERROR_NO) {
		result = -1;
		return result;
	}

	return 0;
}

uint8_t NM_Init(void *CANdevice0Index, uint8_t nodeId)
{
	uint8_t result = 0;
	CO_ReturnError_t err;

	/* Enter CAN configuration. */
	CO_CANsetConfigurationMode((void *)CANdevice0Index);

	err = CO_CANinit((void *)CANdevice0Index, 500);
	if(err != CO_ERROR_NO) {
		result = -2;
		return result;
	}

	g_NodeId = nodeId;
	err = CO_CANopenInit(g_NodeId);
	if(err != CO_ERROR_NO) {
		result = -3;
		return result;
	}

	CO_EM_initCallbackRx(CO->em, EmergencyRxCallback);

	/* Implement a simple node management*/
	CO_NMT_initCallbackChanged(CO->NMT, NmtChangedCallback);

	/* monitor the client nodes*/
	CO_HBconsumer_initCallbackPre(CO->HBcons, NULL, HeartbeatMonitorCallback);

	for (uint8_t idx = 0; idx < CO->HBcons->numberOfMonitoredNodes; ++idx) {
		CO_HBconsumer_initCallbackHeartbeatStarted(CO->HBcons, idx,
				(void*)&CO->HBcons->monitoredNodes[idx],
				HeartbeatStartCallback);

		CO_HBconsumer_initCallbackTimeout(CO->HBcons, idx,
				(void*)&CO->HBcons->monitoredNodes[idx],
				HeartbeatTimeoutCallback);

		CO_HBconsumer_initCallbackRemoteReset(CO->HBcons, idx,
				(void*)&CO->HBcons->monitoredNodes[idx],
				RemoteNodeResetCallback);
	}

	CO_HBconsumer_initCallbackNmtChanged(CO->HBcons, NULL, HeartbeatNmtChangedCallback);

	/*Motor control management initialize*/
	MCM_Init();
	MCM_setupNodesRPDOs();

	/* PDO management and initialize parameters setup*/
	PDOM_Init();
	PDOM_setupTPDOParams();

	/* Node management initialize*/
	BM_Init();
	BM_SetupNodeProfiles();

	/* start CAN */
	CO_CANsetNormalMode(CO->CANmodule[0]);

	return result;
}
