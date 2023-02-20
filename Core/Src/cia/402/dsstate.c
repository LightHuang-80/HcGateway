/*
 * dsstate.c
 *
 *  Created on: 2020��5��26��
 *      Author: Administrator
 */

#include "c402def.h"

uint16_t DS402_ChangeStatusWord(uint16_t curStatusWord, DS402_Status_t state)
{
	uint16_t newStatusWord = curStatusWord;

	switch(state){
	case DS402_Status_NotReadyToSwitchON:
		newStatusWord &= 0xFFB0;
		break;
	case DS402_Status_SwitchONDisabled:
		newStatusWord &= 0xFFB0;
		newStatusWord |= 0x0040;
		break;
	case DS402_Status_ReadyToSwitchON:
		newStatusWord &= 0xFFB0;
		newStatusWord |= 0x0021;
		break;
	case DS402_Status_SwitchedON:
		newStatusWord &= 0xFFB0;
		newStatusWord |= 0x0023;
		break;
	case DS402_Status_OperationEnable:
		newStatusWord &= 0xFFB0;
		newStatusWord |= 0x0027;
		break;
	case DS402_Status_QuickStopActive:
		newStatusWord &= 0xFF90;
		newStatusWord |= 0x0007;
		break;
	case DS402_Status_FaultReactionActive:
		newStatusWord &= 0xFFB0;
		newStatusWord |= 0x000F;
		break;
	case DS402_Status_Fault:
		newStatusWord &= 0xFFB0;
		newStatusWord |= 0x0008;
		break;
	default:
		break;
	}

	return newStatusWord;
}

void DS402_SetStatusWordBit(uint16_t* statusWord, StatusWordBit_t bit, uint8_t enable){
	if (bit >= 16) return;

	uint16_t bitset = 1 << bit;
	if (enable == 1)
		*statusWord |= bitset;
	else
		*statusWord &= ~bitset;
}

uint16_t DS402_buildControlWordByCommand(uint16_t controlWord, DS402_Command_t command)
{
	if (command == DS402_Command_Shutdown) {
		controlWord &= 0xFF7E;
		controlWord |= 0x0006;
	}else if (command == DS402_Command_SwitchON) {
		controlWord &= 0xFF70;
		controlWord |= 0x0007;
	}else if (command == DS402_Command_EnableOperation) {
		controlWord &= 0xFF70;
		controlWord |= 0x000F;
	}

	return controlWord;
}

DS402_Status_t DS402_getStatus(uint16_t statusWord)
{
	DS402_Status_t status = DS402_Status_Fault;
	uint16_t sw = statusWord & 0x6F;

	if (sw == 0x21) {
		status = DS402_Status_ReadyToSwitchON;
	}else if (sw == 0x23) {
		status = DS402_Status_SwitchedON;
	}else if (sw == 0x27) {
		status = DS402_Status_OperationEnable;
	}else if (sw == 0x07) {
		status = DS402_Status_QuickStopActive;
	}else {
		sw &= 0x4F;
		if (sw == 0) {
			status = DS402_Status_NotReadyToSwitchON;
		}else if (sw == 0x40) {
			status = DS402_Status_SwitchONDisabled;
		}else if (sw == 0x0F) {
			status = DS402_Status_FaultReactionActive;
		}
	}

	return status;
}

/*
static DS402_OperateResult_t DS402_ChangeState(DS402_StateData_t *data)
{
	DS402_OperateResult_t result = DS402_OPR_StateNotChanged;
	DS402_Status_t  state = data->state;

    uint32_t cw = data->newControlWord & 0x0F;
    if ((cw & 0x02) == 0){
    	if (state == DS402_Status_ReadyToSwitchON ||
    		state == DS402_Status_OperationEnable ||
			state == DS402_Status_SwitchedON ||
			state == DS402_Status_QuickStopActive){
    		state = DS402_Status_SwitchONDisabled;
    	}
    }

    if (cw == 0x06 || cw == 0x0E){
    	if (state == DS402_Status_SwitchONDisabled ||
    		state == DS402_Status_SwitchedON ||
			state == DS402_Status_OperationEnable){
    		state = DS402_Status_ReadyToSwitchON;
    	}
    }else if (cw == 0x07){
    	if (state == DS402_Status_ReadyToSwitchON ||
    		state == DS402_Status_SwitchONDisabled){
    		state = DS402_Status_SwitchedON;
    	}else if (state == DS402_Status_OperationEnable){
    		state = DS402_Status_SwitchedON;
    	}
    }else if (cw == 0x0F){
    	if (state == DS402_Status_ReadyToSwitchON){
    		state = DS402_Status_SwitchedON;
    	}else if (state == DS402_Status_SwitchedON ||
    			state == DS402_Status_QuickStopActive){
    		state = DS402_Status_OperationEnable;
    	}
    }else if (cw == 0x02 || cw == 0x03){
    	if (state == DS402_Status_ReadyToSwitchON ||
    		state == DS402_Status_SwitchedON){
    		state = DS402_Status_SwitchONDisabled;
    	}else if (state == DS402_Status_OperationEnable){
    		state = DS402_Status_QuickStopActive;
    	}
    }
    return result;
}
*/
