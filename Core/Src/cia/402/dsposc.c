/*
 * dsposc.c
 *
 *  Created on: 2020��5��27��
 *      Author: Administrator
 */

#include <string.h>

#include "c402def.h"
#include "dsposc.h"

PositionControl_t  g_PSCData;

void DS402_PositionControl_Init(void)
{
	memset(&g_PSCData.velRefs, 0, sizeof(VelocityRefs_t));
	g_PSCData.velRefs.effected = false;

	/*Motor init*/
}

void DS402_PositionControl_Process(void)
{
	int32_t dist = g_PSCData.posActual - g_PSCData.posDemand;

	if (dist < g_PSCData.posWindow){
		/*Ŀ��λ���Ѿ�reach�� ����reach point*/
	}

	/*����Following error window �� timeout*/
}

void DS402_PositionControl_SetTargetPos(int32_t target)
{
	g_PSCData.posDemand = target;
	g_PSCData.posDemandByInc = target;

	g_PSCData.targetPos = target;
	//DS402_Motor_SetTargetPos(target);
}

int32_t DS402_PositionControl_GetActualPos()
{
	return g_PSCData.posActual;
}

bool_t DS402_IsVelEffected()
{
	return g_PSCData.velRefs.effected;
}

void DS402_PositionControl_SetVelRefs(VelocityRefs_t *refs)
{
	if (g_PSCData.velRefs.profileVel != refs->profileVel ||
	    g_PSCData.velRefs.profileAccel != refs->profileAccel ||
		g_PSCData.velRefs.profileDecel != refs->profileDecel ||
		g_PSCData.velRefs.quickStopDecel != refs->quickStopDecel){
		g_PSCData.velRefs.effected = false;
	}

	/*
	if (Is_Motor_Stop() && !g_PSCData.velRefs.effected){
        memcpy(&g_PSCData.velRefs, refs, sizeof(VelocityRefs_t));
		g_PSCData.velRefs.effected = true;

		DS402_Motor_CalcIntervalParams(600, 4800, 96000);
 	}else{
 		g_PSCData.velRefs.effected = false;
 	}
 	*/
}
