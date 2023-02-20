/*
 * dspp.c
 *
 *  Created on: 2020��5��27��
 *      Author: Administrator
 */

#include "c402def.h"
#include "dspp.h"

int32_t            g_PPHomeOffset;
ProfiledPosition_t g_PPData;

void DS402_ProfiledPosition_Init(void)
{
	g_PPData.targetPos = 0;

	g_PPData.maxProfileVel = 16000;
	g_PPData.velRefs.profileVel = 0;
	g_PPData.velRefs.profileAccel = 0;
	g_PPData.velRefs.profileDecel = 0;
	g_PPData.velRefs.quickStopDecel = 0;

	g_PPData.posLimit[0] = -0xffff;
	g_PPData.posLimit[1] = 0xffff;

	g_PPData.softwarePosLimit[0] = -0xffff;
	g_PPData.softwarePosLimit[1] = 0xffff;
}

void DS402_ProfiledPosition_SetHomeOffset(int32_t offset)
{
	g_PPData.softwarePosLimit[0] = -offset;
	g_PPData.softwarePosLimit[1] = offset;
}

void DS402_ProfiledPosition_UpdateTargetPos(int32_t target)
{
	int32_t tpos = target;
	/*filter by limit*/


	g_PPData.targetPos = tpos;
	//DS402_PosC_SetTarget(tpos);
}

void DS402_ProfiledPosition_UpdateVelRefs(VelocityRefs_t *velRefs)
{
	if (velRefs->profileVel > g_PPData.maxProfileVel){
		return;
	}

	//DS402_PosC_SetVelRefs(velRefs);
}

void DS402_ProfiledPosition_Process(void)
{
	if (!DS402_IsVelEffected()){
		DS402_ProfiledPosition_UpdateVelRefs(&g_PPData.velRefs);
	}

	//DS402_PosC_Process();
}

void DS402_setTarget(int32_t target)
{

}
