/*
 * bm.c
 *
 *  Created on: 2020年8月5日
 *      Author: Administrator
 */


/*
 * Bundle resource management
 * 1. exchange the SDO entries
 * 2. 402 gateway
 * */

/*
 *  Bundle resource control node status
 *   Uninitialize  --> Processing --> Initialized --> Operational --> Release/Broken
 *                         ^                                              |
 *                         |-----------------------------------------------
 *                         |
 *                         |------------------------------Stop
 * */

/*
 * Sync data SDO objects:
 * 			INTEGER32  left(min) position limit/right(max) position limit
 * 			uint16_t ratio
 * 			uint16_t time window
 * 			uint16_t accel and decel
 * 			uint16_t quick stop decel
 * 			UNSIGNED32 speed(velocity)
 * */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "list.h"
#include "semphr.h"
#include "portable.h"

#include "CANopen.h"
#include "CO_driver_target.h"
#include "CO_OD.h"
#include "301\CO_NMT_Heartbeat.h"

#include "c402def.h"
#include "dspp.h"
#include "bm.h"
#include "mcm.h"
#include "log.h"

const int32_t BM_NEW_VERSION = 0x00000048;

typedef enum BM_Status{
	BM_NODE_Uninitialize,
	BM_NODE_Processing,
	BM_NODE_Initialized,
	BM_NODE_Operational,
	BM_NODE_Release,
	BM_NODE_Stop,
}BM_Status_t;

typedef struct BM_NodeProfile{
	uint8_t              nodeId;
	BM_Status_t          bmStatus;
	uint32_t             version;
	MCM_Status_t         mcmStatus;
}BM_NodeProfile_t;

BM_NodeProfile_t g_BMNodeProfile[NUM_NODES];

typedef struct BM_NodeDriverProfile{
	unsigned char motordir;   // 1 = cw, 0 = ccw
	unsigned char switchdir;  // which side the switch installed, 1 or 0
	unsigned char veltocw;    // 1 = velocity same as dir, or 0
	unsigned char homestate;  // 1 = low trigger, 0 = high trigger
	uint32_t speed;               // motor speed
	uint32_t home_speed;           // homing speed
	uint32_t home_offset;          // software zero position
	float    lower_limit_range;  // lower limit position to origin
	float    ratio;               // speed ratio
	float    acce_ratio;          // acceleration ratio
}BM_NodeDriverProfile_t;

BM_NodeDriverProfile_t g_BMNodeDriverProfile[NUM_NODES] = {
		{1, 0, 0, 0, 5400,  3200, 3200, 40000.0,  25396.544, 1.0},   // 9523.896
		{1, 0, 0, 1, 6400,  3200, 3200, 14400.0,  25396.544, 1.0},  // 6349.136
		{1, 1, 0, 1, 5600,  2400, 2400, 2400.0,   19120.0, 1.0},
		{1, 1, 0, 0, 3200,  1000, 1200, 4000.0,   3200.0, 1.0},
		{1, 1, 0, 0, 5600,  1000, 2000, 6000.0,   5000.0, 1.0},
		{1, 1, 0, 0, 5600,  1000, 2000, 4000.0,   5600.0, 1.0}
};

/* Cia 301 bundle resource command
 * */
#define BM_CMD_ProfileSync   0x01
#define BM_CMD_ProfileReset  0x02
#define BM_CMD_ProfileQuery  0x03
#define BM_CMD_PreOptional	 0x04
#define BM_CMD_Optional	 	 0x05

/* Cia 402 bundle resource command
 * */
#define BM_CMD_MotionControlStart	 0x11
#define BM_CMD_MotionControlStop	 0x12

List_t BM_NMT_MsgFifo;
SemaphoreHandle_t BM_NMT_Semaphore;

typedef enum BM_ProcessState_ {
	BM_Idle,
	BM_Ready,
	BM_Busy,
	BM_Error
}BM_ProcessState_t;

extern QueueHandle_t g_BMMsgQueue;
BM_NodeMessage       g_BMInProcessMsg;
BM_ProcessState_     g_BMState;

/* =====================================
 * BM NMT list management
 * =====================================*/
void BM_PushCmd(uint8_t  dstNodeId, uint8_t bmIdx, uint8_t  msgCode, uint16_t sz, uint8_t* data)
{
	BM_NodeMessage msg;
	msg.dstNodeId = dstNodeId;
	msg.bmIdx     = bmIdx;
	msg.msgCode   = msgCode;
	msg.msgSize   = sz;
	msg.data      = data;

	xQueueSend(g_BMMsgQueue, &msg, 0);
}

bool_t BM_PopCmd(BM_NodeMessage* msg)
{
	BaseType_t result = xQueueReceive(g_BMMsgQueue, msg, 0);
	if (result == pdPASS) {
		return true;
	}
	return false;
}

/* ===================================== */

void BM_Init()
{
	g_BMState = BM_Idle;
}

void BM_SetupNodeProfiles()
{
	/* Initialize the nodes
	 * Make sure the NUM_NODES equal the NMT monitored node
	 * */
	for (uint8_t i = 0; i < NUM_NODES && i < CO->HBcons->numberOfMonitoredNodes; i++) {
		g_BMNodeProfile[i].nodeId    = CO->HBcons->monitoredNodes[i].nodeId;
		g_BMNodeProfile[i].bmStatus  = BM_NODE_Uninitialize;
		g_BMNodeProfile[i].mcmStatus = MCM_NODE_PowerDisabled;

		g_BMNodeProfile[i].version   = BM_NEW_VERSION;  //0.0.0.47
	}
}

/* ===============================================
 * Bundle resource nmt event callback
 * */
void BM_OnNodeInitialize(uint8_t nodeId, uint8_t monitoredIdx)
{
	if (monitoredIdx >= NUM_NODES) return;
	if (g_BMNodeProfile[monitoredIdx].nodeId != nodeId) return;

	LOG_Print(LOG_InfoLevel, "%d in initialize\n", nodeId);

	/* Should re-initialize node if it not initialized the profile data*/
	g_BMNodeProfile[monitoredIdx].bmStatus = BM_NODE_Uninitialize;

	BM_PushCmd(nodeId, monitoredIdx, BM_CMD_PreOptional, 0, NULL);
}

void BM_OnNodePreOptional(uint8_t nodeId, uint8_t monitoredIdx)
{
	if (monitoredIdx >= NUM_NODES) return;
	if (g_BMNodeProfile[monitoredIdx].nodeId != nodeId) return;

	LOG_Print(LOG_InfoLevel, "%d in preoptional\n", nodeId);

	/*Reset the bundle resource status of node*/
	if (g_BMNodeProfile[monitoredIdx].bmStatus == BM_NODE_Initialized ||
		g_BMNodeProfile[monitoredIdx].bmStatus == BM_NODE_Stop){
		g_BMNodeProfile[monitoredIdx].bmStatus = BM_NODE_Release;
	}else if (g_BMNodeProfile[monitoredIdx].bmStatus == BM_NODE_Processing) {
		g_BMNodeProfile[monitoredIdx].bmStatus = BM_NODE_Uninitialize;
	}

	BM_PushCmd(nodeId, monitoredIdx, BM_CMD_ProfileQuery, 0, NULL);
}

void BM_OnNodeActived(uint8_t nodeId, uint8_t monitoredIdx)
{
	if (monitoredIdx >= NUM_NODES) return;
	if (g_BMNodeProfile[monitoredIdx].nodeId != nodeId) return;

	LOG_Print(LOG_InfoLevel, "%d in opt\n", nodeId);

	/* Should re-initialize node if it not initialized the profile data*/
	if (g_BMNodeProfile[monitoredIdx].bmStatus != BM_NODE_Initialized) {
		/* The node nmt state enters optional before profile synced*/
		BM_PushCmd(nodeId, monitoredIdx, BM_CMD_ProfileQuery, 0, NULL);
	}else {
		/* Start motion control*/
		g_BMNodeProfile[monitoredIdx].mcmStatus = MCM_NODE_PowerEnabled;
	}
}

void BM_OnNodeReleased(uint8_t nodeId, uint8_t monitoredIdx)
{
	if (monitoredIdx >= NUM_NODES) return;
	if (g_BMNodeProfile[monitoredIdx].nodeId != nodeId) return;

	if (g_BMNodeProfile[monitoredIdx].bmStatus == BM_NODE_Initialized ||
		g_BMNodeProfile[monitoredIdx].bmStatus == BM_NODE_Stop){
		g_BMNodeProfile[monitoredIdx].bmStatus = BM_NODE_Release;
	}else if (g_BMNodeProfile[monitoredIdx].bmStatus == BM_NODE_Processing) {
		g_BMNodeProfile[monitoredIdx].bmStatus = BM_NODE_Uninitialize;
	}

	MCM_OnNodeReset(nodeId, monitoredIdx);
}

void BM_OnNodeStop(uint8_t nodeId, uint8_t monitoredIdx)
{
	if (monitoredIdx >= NUM_NODES) return;
	if (g_BMNodeProfile[monitoredIdx].nodeId != nodeId) return;

	/* Make sure the node is initialized before*/
	if (g_BMNodeProfile[monitoredIdx].bmStatus == BM_NODE_Initialized ||
		g_BMNodeProfile[monitoredIdx].bmStatus == BM_NODE_Release) {
		g_BMNodeProfile[monitoredIdx].bmStatus = BM_NODE_Stop;
	}else {
		/* Step back*/
		g_BMNodeProfile[monitoredIdx].bmStatus = BM_NODE_Uninitialize;
	}
}

void BM_OnNodeReset(uint8_t nodeId, uint8_t monitoredIdx)
{
	if (monitoredIdx >= NUM_NODES) return;
	if (g_BMNodeProfile[monitoredIdx].nodeId != nodeId) return;

	g_BMNodeProfile[monitoredIdx].bmStatus = BM_NODE_Uninitialize;

	MCM_OnNodeReset(nodeId, monitoredIdx);
}

void BM_OnNodeReady(uint8_t nodeId, uint8_t monitoredIdx)
{
	if (monitoredIdx >= NUM_NODES) return;
	if (g_BMNodeProfile[monitoredIdx].nodeId != nodeId) return;

	g_BMNodeProfile[monitoredIdx].bmStatus = BM_NODE_Initialized;

	LOG_Print(LOG_InfoLevel, "%d synced\n", nodeId);

	CO_NMT_internalState_t nmtState;
	CO_HBconsumer_getNmtState(CO->HBcons, monitoredIdx, &nmtState);

	if (nmtState != CO_NMT_OPERATIONAL) {
		/* Node profile parameters sync finished.*/
		BM_PushCmd(nodeId, monitoredIdx, BM_CMD_Optional, 0, NULL);
	}else {
		/* Start motion control*/
		g_BMNodeProfile[monitoredIdx].mcmStatus = MCM_NODE_PowerEnabled;
	}
}

void BM_OnNodeNMTStateChanged(uint16_t nodeId, uint8_t monitoredIdx, CO_NMT_internalState_t state, CO_HBconsumer_state_t hbState)
{
	if (hbState == CO_HBconsumer_TIMEOUT ||
		hbState == CO_HBconsumer_UNKNOWN ){
		BM_OnNodeReleased(nodeId, monitoredIdx);
	}else {
		if (state == CO_NMT_INITIALIZING){
			BM_OnNodeInitialize(nodeId, monitoredIdx);
		}else if (state == CO_NMT_OPERATIONAL) {
			BM_OnNodeActived(nodeId, monitoredIdx);
		}else if (state == CO_NMT_PRE_OPERATIONAL) {
			BM_OnNodePreOptional(nodeId, monitoredIdx);
		}else if (state == CO_NMT_STOPPED){
			BM_OnNodeStop(nodeId, monitoredIdx);
		}
	}
}
/* =============================================== */


/* =============================================
 * BM node profile management
   ============================================= */
CO_SDOclient_return_t BM_processProfileCmd(uint8_t bmIdx, uint16_t sdo_idx, uint8_t subsdo_idx)
{
	CO_SDOclient_return_t result = CO_SDOcli_wrongArguments;

	if (g_BMNodeProfile[bmIdx].bmStatus == BM_NODE_Uninitialize ||
			g_BMNodeProfile[bmIdx].bmStatus == BM_NODE_Release ||
			g_BMNodeProfile[bmIdx].bmStatus == BM_NODE_Stop){

		/*Set up the SDO client, and initialize the SDO procedure*/
		result = CO_SDOclient_setup(CO->SDOclient[0],
					0x600 + g_BMNodeProfile[bmIdx].nodeId,
					0x580 + CO->NMT->nodeId,
					g_BMNodeProfile[bmIdx].nodeId);

		if (result != CO_SDOcli_ok_communicationEnd) {
			return result;
		}

		/*Sync the DS402 profile data, entry id 0xC120(manufactory definition)
		 * To finish find OD's attribute and length before send the download command
		 * */
		result = CO_SDOclientDownloadInitiate(CO->SDOclient[0],
				sdo_idx,     /*SDO index, ds402 profile big package*/
				subsdo_idx,    	    /*SDO sub-index, profile data*/
				376,        /*size of entry*/
				1000,       /*timeout value*/
				true        /*enable block functional*/);

		if (result != CO_SDOcli_ok_communicationEnd &&
			result != CO_SDOcli_waitingServerResponse &&
			result != CO_SDOcli_blockDownldInProgress){
				g_BMNodeProfile[bmIdx].bmStatus = BM_NODE_Uninitialize;
				return result;
		}else {
			LOG_Print(LOG_InfoLevel, "Prepare the fifo buffer, and start download initiating to node(%d)\n", g_BMNodeProfile[bmIdx].nodeId);

			/* Define the new version in profile data*/
			memcpy(&CO_OD_ROM.ds402Profile.data[0], &BM_NEW_VERSION, sizeof(int32_t));

			/* Setup the driver profile*/
			memcpy((unsigned char*)&CO_OD_ROM.ds402Profile.data[1], (unsigned char*)&g_BMNodeDriverProfile[bmIdx], sizeof(BM_NodeDriverProfile_t));

			size_t count = 376;
			size_t bufCount = CO_SDOclientDownloadBufWrite(CO->SDOclient[0],
													(const char*)&CO_OD_ROM.ds402Profile.data[0],
													count);
			if (count != bufCount) {
				g_BMNodeProfile[bmIdx].bmStatus = BM_NODE_Uninitialize;
				result = CO_SDOcli_transmittBufferFull;
			}
		}

		if (result == CO_SDOcli_ok_communicationEnd){
			result = CO_SDOcli_blockDownldInProgress;
			g_BMNodeProfile[bmIdx].bmStatus = BM_NODE_Processing;
		}

	}else if (g_BMNodeProfile[bmIdx].bmStatus == BM_NODE_Processing){

		CO_SDO_abortCode_t abort;
		size_t  transCount;
		uint32_t nextTime;

		result = CO_SDOclientDownload(CO->SDOclient[0],
								100,     /*timeDifference_us*/
								false,    /*abort*/
								&abort,  /*abort code*/
								&transCount, /*transmitted size*/
								&nextTime   /*timerNext_us*/);

		if (abort != CO_SDO_AB_NONE){
			if (result == CO_SDOcli_ok_communicationEnd){
				result = CO_SDOcli_endedWithClientAbort;
			}else {
				LOG_Print(LOG_InfoLevel, "Node(%d)'s profile data has initialized.\n", g_BMNodeProfile[bmIdx].nodeId);
			}
		}
	}

	return result;
}

CO_SDOclient_return_t BM_processQueryCmd(uint8_t nodeId, uint16_t sdo_idx,  uint8_t sdo_subIdx, uint16_t SDOtimeoutTime_ms, bool_t blockEnable)
{
	CO_SDOclient_return_t result = CO_SDOclient_setup(CO->SDOclient[0],
						0x600 + nodeId,
						0x580 + CO->NMT->nodeId,
						nodeId);

	if (result != CO_SDOcli_ok_communicationEnd){
		return result;
	}

	result = CO_SDOclientUploadInitiate(CO->SDOclient[0],
			sdo_idx,     /*SDO index, ds402 profile big package*/
			sdo_subIdx,    	    /*SDO sub-index, profile data*/
			SDOtimeoutTime_ms,       /*timeout value*/
			blockEnable        /*enable block functional*/);
	if (result != CO_SDOcli_ok_communicationEnd) {
		return result;
	}

	return result;
}

/* =============================================
 * BM query version slave node
 * ============================================= */
CO_SDOclient_return_t BM_processVersionCmd(uint8_t bmIdx, uint16_t sdo_idx, uint8_t subsdo_idx)
{
	CO_SDOclient_return_t ret = CO_SDOcli_wrongArguments;

	if (g_BMNodeProfile[bmIdx].bmStatus == BM_NODE_Uninitialize ||
			g_BMNodeProfile[bmIdx].bmStatus == BM_NODE_Release ||
			g_BMNodeProfile[bmIdx].bmStatus == BM_NODE_Stop) {

		/* Create a upload init*/
		uint16_t SDOtimeoutTime_ms = 1000;

		uint16_t entryNo = CO_OD_find(CO->SDO[0], sdo_idx);
		if (entryNo == 0xFFFF) {
			return ret;
		}
		uint16_t len = CO_OD_getLength(CO->SDO[0], entryNo, subsdo_idx);
		bool_t  blockEnable = (len > 4) ? true : false;

		ret = BM_processQueryCmd(g_BMNodeProfile[bmIdx].nodeId, sdo_idx, subsdo_idx, SDOtimeoutTime_ms, blockEnable);
		if (ret == CO_SDOcli_ok_communicationEnd) {
			g_BMNodeProfile[bmIdx].bmStatus = BM_NODE_Processing;
			/* Upload in progress handle*/
			ret = CO_SDOcli_blockUploadInProgress;
		}
		return ret;
	}else if (g_BMNodeProfile[bmIdx].bmStatus == BM_NODE_Processing) {

		uint32_t 			timeDifference_us = 300;
		CO_SDO_abortCode_t 	SDOabortCode;
		size_t 				sizeIndicated;
		size_t 				sizeTransferred;
		uint32_t 			timerNext_us;

		ret = CO_SDOclientUpload(CO->SDOclient[0], timeDifference_us,
			&SDOabortCode,
			&sizeIndicated,
            &sizeTransferred,
            &timerNext_us);

		if (SDOabortCode != CO_SDO_AB_NONE) {
			if (ret == CO_SDOcli_ok_communicationEnd) {
				ret = CO_SDOcli_endedWithClientAbort;
			}
		}
	}
	return ret;
}

void BM_processCmd()
{
	if (g_BMState == BM_Idle) {
		if (!BM_PopCmd(&g_BMInProcessMsg))
			return;
	}

	g_BMState = BM_Busy;

	uint8_t bmIdx = g_BMInProcessMsg.bmIdx;

	CO_SDOclient_return_t ret;
	switch(g_BMInProcessMsg.msgCode) {
	case BM_CMD_ProfileQuery: {
		/* Version & profile data OD (0xC120)*/
		ret = BM_processVersionCmd(bmIdx, 0xC120, 3);
		if (ret == CO_SDOcli_ok_communicationEnd){
			/* Compare the bundle resource version of node */
			int32_t slaveVersion;
			size_t sz = CO_SDOclientUploadBufRead(CO->SDOclient[0], (char*)&slaveVersion, 4);

			if (sz == 4 && slaveVersion == BM_NEW_VERSION){
				/* Slave node already initialized, the version is latest.
				 * And enter initialized phase */
				BM_OnNodeReady(g_BMInProcessMsg.dstNodeId, g_BMInProcessMsg.bmIdx);
			}else {
				g_BMNodeProfile[bmIdx].bmStatus = BM_NODE_Uninitialize;
				BM_PushCmd(g_BMInProcessMsg.dstNodeId, bmIdx, BM_CMD_ProfileSync, 0, NULL);
			}
		}

		if (ret != CO_SDOcli_blockUploadInProgress && ret != CO_SDOcli_waitingServerResponse) {
			g_BMState = BM_Idle;
		}
	}
	break;
	case BM_CMD_ProfileSync: {
		ret = BM_processProfileCmd(bmIdx, 0xC120, 4);
		if (ret == CO_SDOcli_ok_communicationEnd){
			/* Slave node already initialized, the version is latest.
			 * And enter initialized phase */
			BM_OnNodeReady(g_BMInProcessMsg.dstNodeId, g_BMInProcessMsg.bmIdx);
		}

		if (ret != CO_SDOcli_blockDownldInProgress && ret != CO_SDOcli_waitingServerResponse) {
			g_BMState = BM_Idle;
		}
	}
	break;
	case BM_CMD_PreOptional: {
		uint8_t nodeId = g_BMInProcessMsg.dstNodeId;
		LOG_Print(LOG_InfoLevel, "Send preopt to %d\n", nodeId);
		CO_NMT_sendCommand(CO->NMT, CO_NMT_ENTER_PRE_OPERATIONAL, nodeId);

		g_BMState = BM_Idle;
	}
	break;

	case BM_CMD_Optional: {
		uint8_t nodeId = g_BMInProcessMsg.dstNodeId;
		LOG_Print(LOG_InfoLevel, "Send opt to %d\n", nodeId);
		CO_NMT_sendCommand(CO->NMT, CO_NMT_ENTER_OPERATIONAL, nodeId);

		g_BMState = BM_Idle;
	}
	break;

	case BM_CMD_MotionControlStart: {
		g_BMNodeProfile[bmIdx].mcmStatus = MCM_NODE_PowerEnabled;

		g_BMState = BM_Idle;
	}
	break;

	default:{
		g_BMState = BM_Idle;
	}
	break;
	}
}

void BM_processMCM()
{
	uint8_t idx = 0;
	for(; idx < NUM_NODES; idx++) {
		MCM_process(g_BMNodeProfile[idx].nodeId, idx, g_BMNodeProfile[idx].mcmStatus);
	}
}

uint8_t BM_Process()
{
	BM_processCmd();
	BM_processMCM();

	return 0;
}
