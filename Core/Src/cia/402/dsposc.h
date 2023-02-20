/*
 * dsposc.h
 *
 *  Created on: 2020年5月27日
 *      Author: Administrator
 */

#ifndef DSPOSC_H_
#define DSPOSC_H_

typedef void (*DS402_UpdateTargetPosFunc_t)(int32_t target);
typedef void (*DS402_ModeProcessFunc_t)(void);

typedef struct DS402_VelocityRefs{
	uint32_t  profileVel;        // 0x6081
	uint32_t  profileAccel;      // 0x6083
	uint32_t  profileDecel;      // 0x6084
	uint32_t  quickStopDecel;    // 0x6085
	uint8_t   effected;          // 是否生效
}VelocityRefs_t;

typedef struct DS402_PositionControl{
	int32_t   posDemand;    // 0x6062
	int32_t   posActual;    // 0x6064

	int32_t   posDemandByInc;   // 0x60FC
	int32_t   posActualByInc;   // 0x6063

	uint32_t  followingErrWindow;  //0x6065
	uint16_t  followingErrTimeout; //0x6066

	uint32_t  posWindow;           //0x6067
	uint16_t  posWindowTimeout;    //0x6068

	int32_t         targetPos;
	VelocityRefs_t  velRefs;
}PositionControl_t;

void DS402_PositionControl_Init(void);
void DS402_PositionControl_Process(void);

void DS402_PositionControl_SetTargetPos(int32_t target);
int32_t DS402_PositionControl_GetActualPos();

bool_t DS402_IsVelEffected();
void DS402_PositionControl_SetVelRefs(VelocityRefs_t *refs);

#endif /* DSPOSC_H_ */
