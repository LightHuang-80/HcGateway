/*
 * dmap.cpp
 *
 *  Created on: 2020年9月23日
 *      Author: Administrator
 */

#include <stdio.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "queue.h"

#include "301/CO_driver.h"
#include "301\CO_SDOserver.h"
#include "301\CO_Emergency.h"
#include "301\CO_NMT_Heartbeat.h"
#include "301\CO_HBconsumer.h"
#include "CO_OD.h"
#include "pdom.h"
#include "CANopen.h"
#include "bm.h"
#include "mcm.h"

QueueHandle_t gPDOQueue;

/* Cia402 data map*/
/* 1. Control word			U16	[6040]
 * 2. Target position		I32 [607A]
 * 3. Profile velocity		U32 [6081]
 * 4. Accelerate			U32 [6083]
 * 5. Decelerate			U32 [6084]
 * 6. Quick stop decelerate	U32
 * 7. Profile mode			I8  [6060]
 * */

/* Mapping object
 *
 * Bundle format                                  Device         Host
 * 1. ControlBit(U16)[0x6040] | Mode(I8)[0x6060]  -------      1400/1600  TPDO1
 * 2. Target Pos(I32)[0x607A]                     -------      1401/1601  TPDO2
 * 3. ProfileVel(U32)[0x6081]                     -------      1402/1602  TPDO3
 * 4. Accelerate(U32)[0x6083) | Decelerate(U32)[0x6084]--      1403/1603  TPDO4
 *
 * */


/* Map TPDO 0, 0x200 + nodeId, 6040, 6060
 * | 0x00 0x00 | 0x00
 *  controlbit   mode
 *
 * Map TPDO 1, 0x300 + nodeId, 607A
 * | 0x00 0x00 0x00 0x00 |
 *  Target position
 *
 * Map TPDO 2, 0x400 + nodeId, 6081
 * | 0x00 0x00 0x00 0x00 |
 *   Profile velocity
 *
 * Map TPDO 3, 0x500 + nodeId, 6083, 6084
 * | 0x00 0x00 0x00 0x00 | 0x00 0x00 0x00 0x00 |
 *      Accelerate           Decelerate
 *
 * */
/* Send buffer*/
uint8_t gTPDOData[4][8];

/* TPDO communicate parameters*/
typedef struct TPDO_Comm_Params_ {
	uint32_t  COB_IDUsedByTPDO;

	uint32_t  inhibitTimeCounter;
	uint32_t  eventTimerCounter;
	uint32_t  syncCounter;

	uint16_t  inhibitTime;
	uint16_t  eventTimer;
	uint8_t   transmissionType;
	uint8_t   SYNCStartValue;

	uint8_t   data[8];
} TPDO_Comm_Params_t;

/* 6 nodes, 4 TPDO channels */
TPDO_Comm_Params_t g_TPDOCommParams[NUM_NODES][CO_NO_TPDO];

extern MCM_DriverData_t g_MotorDriverData[NUM_NODES];

const uint8_t k1Byte  = 1;
const uint8_t k2Bytes = 2;
const uint8_t k4Bytes = 4;

CO_OD_entryRecord_t OD_record6040[2] = {
		{(void*)&k1Byte, 0x06, 1},
		{(void*)&gTPDOData[0][0], 0xEE,2}
};

/*0x6060 Profile mode*/
CO_OD_entryRecord_t OD_record6060[2] = {
		{(void*)&k1Byte, 0x06, 1},
		{(void*)&gTPDOData[0][2], 0xEE, 1}
};

/*0x607A Target position*/
CO_OD_entryRecord_t OD_record607A[2] = {
		{(void*)&k1Byte, 0x06, 1},
		{(void*)&gTPDOData[1][0], 0xEE, 4}
};

/*0xC101 Command serial number*/
CO_OD_entryRecord_t OD_recordC101[2] = {
		{(void*)&k1Byte, 0x06, 1},
		{(void*)&gTPDOData[1][4], 0xEE, 4}
};

/*0x6081 Profile velocity*/
CO_OD_entryRecord_t OD_record6081[2] = {
		{(void*)&k1Byte, 0x06, 1},
		{(void*)&gTPDOData[2][0], 0xEE, 4}
};

/*0x6083 Accelerate*/
CO_OD_entryRecord_t OD_record6083[2] = {
		{(void*)&k1Byte, 0x06, 1},
		{(void*)&gTPDOData[2][4], 0xEE, 4}
};

/*0x6084 Decelerate*/
CO_OD_entryRecord_t OD_record6084[2] = {
		{(void*)&k1Byte, 0x06, 1},
		{(void*)&gTPDOData[3][0], 0xEE, 4}
};

typedef struct PDO_DataCache_{
	int32_t serialNum;
	float   position;
	float   velocity;
	float   acceleration;
	uint8_t reset;
}PDO_DataCache_t;

PDO_DataCache_t g_PDODataCache[NUM_NODES] = {{0}};

void PDOM_resetCache(int8_t nodeIdx)
{
	if (nodeIdx < 0 || nodeIdx >= NUM_NODES) return;
	g_PDODataCache[nodeIdx].reset = 1;
}

void PDOM_MapTPDO(uint8_t TPDO_idx, uint8_t nodeId, uint8_t* data)
{
	if (TPDO_idx >= 4) return;

	int8_t nodeIdx = CO_HBconsumer_getIdxByNodeId(CO->HBcons, nodeId);
	if (nodeIdx < 0) return;

	/*
	uint32_t lastd0 = *(uint32_t*)&g_TPDOCommParams[nodeIdx][TPDO_idx].data[0];
	uint32_t lastd1 = *(uint32_t*)&g_TPDOCommParams[nodeIdx][TPDO_idx].data[4];

	uint32_t d0 = *(uint32_t*)data;
	uint32_t d1 = *(uint32_t*)(data + 4);

	uint8_t sendRequest = (d0 ^ lastd0) | (d1 ^ lastd1);
	*/

	CO->TPDO[TPDO_idx]->sendRequest = 1; //sendRequest;

	/*Set the TPDO comm parameters*/
	CO->TPDO[TPDO_idx]->TPDOCommPar->transmissionType = g_TPDOCommParams[nodeIdx][TPDO_idx].transmissionType;
	CO->TPDO[TPDO_idx]->TPDOCommPar->inhibitTime = g_TPDOCommParams[nodeIdx][TPDO_idx].inhibitTime;
	CO->TPDO[TPDO_idx]->TPDOCommPar->eventTimer = g_TPDOCommParams[nodeIdx][TPDO_idx].eventTimer;
	CO->TPDO[TPDO_idx]->TPDOCommPar->SYNCStartValue = g_TPDOCommParams[nodeIdx][TPDO_idx].SYNCStartValue;

	/*Set the TPDO counter*/
	CO->TPDO[TPDO_idx]->inhibitTimer = g_TPDOCommParams[nodeIdx][TPDO_idx].inhibitTimeCounter;
	CO->TPDO[TPDO_idx]->eventTimer   = g_TPDOCommParams[nodeIdx][TPDO_idx].eventTimerCounter;
	CO->TPDO[TPDO_idx]->syncCounter  = g_TPDOCommParams[nodeIdx][TPDO_idx].syncCounter;

	/*copy data into buffer*/
	memcpy(gTPDOData[TPDO_idx], data, 8);

	/*Config the target node for TPDO tx buffer*/
	uint32_t cobId = CO->TPDO[TPDO_idx]->CANtxBuff->ident;
	cobId &= 0x1E03;
	cobId |= nodeId << 2;
	CO->TPDO[TPDO_idx]->CANtxBuff->ident = cobId;

	uint32_t timerNext;
	bool     syncWas = true;
	CO_TPDO_process(CO->TPDO[TPDO_idx], syncWas, 100, &timerNext);

	/* Update the last node data*/
	memcpy(&g_TPDOCommParams[nodeIdx][TPDO_idx].data[0], data, 8);

	/*Save the TPDO comm parameters*/
	g_TPDOCommParams[nodeIdx][TPDO_idx].inhibitTime = CO->TPDO[TPDO_idx]->TPDOCommPar->inhibitTime;
	g_TPDOCommParams[nodeIdx][TPDO_idx].eventTimer = CO->TPDO[TPDO_idx]->TPDOCommPar->eventTimer;
	g_TPDOCommParams[nodeIdx][TPDO_idx].SYNCStartValue = CO->TPDO[TPDO_idx]->TPDOCommPar->SYNCStartValue;

	/*Set the TPDO counter*/
	g_TPDOCommParams[nodeIdx][TPDO_idx].inhibitTimeCounter = CO->TPDO[TPDO_idx]->inhibitTimer;
	g_TPDOCommParams[nodeIdx][TPDO_idx].eventTimerCounter = CO->TPDO[TPDO_idx]->eventTimer;
	g_TPDOCommParams[nodeIdx][TPDO_idx].syncCounter = CO->TPDO[TPDO_idx]->syncCounter;
}

void PDOM_Init()
{
	/*Clean the data*/
	memset(gTPDOData, 0, 4 * 8 * sizeof(uint8_t));

	/*Create TPDO message queue*/
	gPDOQueue = xQueueCreate(120, sizeof(MTPDO_map_item_t));
}

/*Called after TPDO init finished*/
void PDOM_setupTPDOParams()
{
	uint8_t idx = 0;

	for (; idx < CO->HBcons->numberOfMonitoredNodes; idx++) {
		uint8_t nodeId = CO->HBcons->monitoredNodes->nodeId;

		uint8_t ch = 0;
		for (; ch < CO_NO_TPDO; ch++){
			uint32_t cobId = (CO->TPDO[ch]->defaultCOB_ID & 0x07F) | nodeId;
			g_TPDOCommParams[idx][ch].COB_IDUsedByTPDO = cobId;

			g_TPDOCommParams[idx][ch].eventTimerCounter = CO->TPDO[ch]->eventTimer;
			g_TPDOCommParams[idx][ch].inhibitTimeCounter = CO->TPDO[ch]->inhibitTimer;
			g_TPDOCommParams[idx][ch].syncCounter = CO->TPDO[ch]->syncCounter;

			g_TPDOCommParams[idx][ch].eventTimer = CO->TPDO[ch]->TPDOCommPar->eventTimer;
			g_TPDOCommParams[idx][ch].inhibitTime = CO->TPDO[ch]->TPDOCommPar->inhibitTime;
			g_TPDOCommParams[idx][ch].SYNCStartValue = CO->TPDO[ch]->TPDOCommPar->SYNCStartValue;
		}
	}
}

void PDOM_process()
{
	/*Receive the message from Queue */
	MTPDO_map_item_t pdoItem;

	BaseType_t ret = xQueueReceive(gPDOQueue, (void *)&pdoItem, 0);
	if (ret == pdPASS) {
		/*Send the item*/
		PDOM_MapTPDO(pdoItem.tpdoIdx, pdoItem.nodeId, pdoItem.data);
	}
}

bool_t PDOM_sendItem(MTPDO_map_item_t* item)
{
	BaseType_t ret = xQueueSend(gPDOQueue, item, 10);
	if (ret != pdPASS) {
		printf("Push item failed.\n");
		return false;
	}

	return true;
}

bool_t PDOM_sendTarget(int8_t nodeId, int8_t ch, int32_t target)
{
	/*Heavy check the slave driver of target position transmit permission*/
	int8_t idx = 0;
	idx = CO_HBconsumer_getIdxByNodeId(CO->HBcons, nodeId);

	if (!g_MotorDriverData[idx].posDriveEnable)
		return false;

    MTPDO_map_item_t item;

    item.nodeId = nodeId;
    item.tpdoIdx = ch;
    memcpy(&item.data, (void*)&target, sizeof(int32_t));

    return PDOM_sendItem(&item);
}

bool_t PDOM_sendPDO(int8_t nodeIdx, int8_t ch, float fdata)
{
	/*Heavy check the slave driver of target position transmit permission*/
	if (nodeIdx < 0 || nodeIdx >= NUM_NODES) return false;

	if (!g_MotorDriverData[nodeIdx].posDriveEnable)
		return false;

    MTPDO_map_item_t item;

    item.nodeId = g_MotorDriverData[nodeIdx].nodeId;
    item.tpdoIdx = ch;
    memcpy(&item.data, (void*)&fdata, sizeof(float));

    return PDOM_sendItem(&item);
}

bool_t PDOM_send8BytesPDO(int8_t nodeIdx, int8_t ch, void* data1, size_t sz1, void* data2, size_t sz2)
{
	/*Heavy check the slave driver of target position transmit permission*/
	if (nodeIdx < 0 || nodeIdx >= NUM_NODES) return false;

	if (!g_MotorDriverData[nodeIdx].posDriveEnable)
		return false;

    MTPDO_map_item_t item;

    item.nodeId = g_MotorDriverData[nodeIdx].nodeId;
    item.tpdoIdx = ch;
    memcpy(&item.data, data1, sz1);
    memcpy(&(item.data[4]), data2, sz2);

    return PDOM_sendItem(&item);
}

bool_t PDOM_sendTPDOs(int8_t nodeIdx, int32_t serialNum, float pos, float vel, float acce)
{
	if (nodeIdx < 0 || nodeIdx >= NUM_NODES) return false;

	if (g_PDODataCache[nodeIdx].reset == 1 ||
	    pos != g_PDODataCache[nodeIdx].position ||
	    vel != g_PDODataCache[nodeIdx].velocity ||
		acce != g_PDODataCache[nodeIdx].acceleration){

	    bool_t sent = PDOM_send8BytesPDO(nodeIdx, 1, &pos, sizeof(float), &serialNum, sizeof(int32_t));
	    if (sent){
	    	sent = PDOM_send8BytesPDO(nodeIdx, 2, &vel, sizeof(float), &acce, sizeof(float));
	    }

	    if (sent){
	    	// save the latest PP data
	    	g_PDODataCache[nodeIdx].position = pos;
	    	g_PDODataCache[nodeIdx].velocity = vel;
	    	g_PDODataCache[nodeIdx].acceleration = acce;
	    	g_PDODataCache[nodeIdx].serialNum = serialNum;

	    	g_PDODataCache[nodeIdx].reset = 0;
	    }

        return sent;
	}

	return false;
}
