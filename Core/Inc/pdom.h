/*
 * pdom.h
 *
 *  Created on: 2020年9月26日
 *      Author: Administrator
 */

#ifndef SRC_NM_PDOM_H_
#define SRC_NM_PDOM_H_

#include "CO_OD.h"

typedef enum MTPDO_map_idx_ {
	MP_Control = 3,		// 0x6040
	MP_TargetPos,		// 0x607A
	MP_Velocity,		// 0x6081
	MP_Accelerate,		// 0x6083
	MP_TimeStamp,		// 0x0000
}MTPDO_map_idx;

typedef struct MTPDO_map_item_ {
	uint8_t  nodeId;		// 1 - 127
	uint8_t  tpdoIdx;       // 0-3
	uint8_t  data[8];		// mapped data
}MTPDO_map_item_t;

#ifdef __cplusplus
extern "C" {
#endif

void   PDOM_Init();
void   PDOM_setupTPDOParams();
void   PDOM_process();
bool_t PDOM_sendItem(MTPDO_map_item_t* item);
bool_t PDOM_sendTarget(int8_t nodeId, int8_t ch, int32_t target);
bool_t PDOM_sendPDO(int8_t nodeIdx, int8_t ch, float fdata);
bool_t PDOM_send8BytesPDO(int8_t nodeIdx, int8_t ch, void* data1, size_t sz1, void* data2, size_t sz2);
bool_t PDOM_sendTPDOs(int8_t nodeIdx, int32_t serialNum, float pos, float vel, float acce);
void   PDOM_resetCache(int8_t nodeIdx);
#ifdef __cplusplus
}
#endif


#endif /* SRC_NM_PDOM_H_ */
