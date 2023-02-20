/*
 * bm.h
 *
 *  Created on: 2020年9月13日
 *      Author: Administrator
 */

#ifndef NM_BM_H_
#define NM_BM_H_

#ifdef __cplusplus
extern "C" {
#endif

#define NUM_NODES   6

typedef struct BM_NodeMessage_ {
	uint8_t  dstNodeId;
	uint8_t  bmIdx;
	uint8_t  msgCode;
	uint16_t msgSize;
	uint8_t  *data;
}BM_NodeMessage;

void BM_Init();
void BM_SetupNodeProfiles();
void BM_OnNodeNMTStateChanged(uint16_t nodeId, uint8_t monitoredIdx, CO_NMT_internalState_t state, CO_HBconsumer_state_t hbState);

uint8_t BM_Process();

#ifdef __cplusplus
}
#endif

#endif /* NM_BM_H_ */
