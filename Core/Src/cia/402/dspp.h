/*
 * dspp.h
 *
 *  Created on: 2020Äê5ÔÂ27ÈÕ
 *      Author: Administrator
 */

#ifndef DSPP_H_
#define DSPP_H_

#include "dsposc.h"


typedef struct DS402_ProfiledPosition{
	int32_t  targetPos;             //
	int32_t  newPos;                // 0x607A

	int32_t  posLimit[2];           // 0x607B
	int32_t  softwarePosLimit[2];   // 0x607D

	uint32_t maxProfileVel;         // 0x607F

	VelocityRefs_t velRefs;
}ProfiledPosition_t;

void DS402_ProfiledPosition_Init(void);
void DS402_ProfiledPosition_Process(void);

void DS402_ProfiledPosition_UpdateTargetPos(int32_t target);
void DS402_ProfiledPosition_UpdateVelRefs(VelocityRefs_t *velRefs);

void DS402_ProfiledPosition_SetHomeOffset(int32_t offset);

#endif /* DSPP_H_ */
