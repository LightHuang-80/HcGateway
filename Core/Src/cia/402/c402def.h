/*
 * c402def.h
 *
 *  Created on: 2020��5��21��
 *      Author: Administrator
 */

#ifndef C402DEF_H_
#define C402DEF_H_

#include <stdint.h>
#include "CO_driver_target.h"
#include "CO_OD.h"

typedef enum DS402_STATUS{
  DS402_Status_Start = 0x0,
  DS402_Status_NotReadyToSwitchON = 0x1,
  DS402_Status_SwitchONDisabled,
  DS402_Status_ReadyToSwitchON,
  DS402_Status_SwitchedON,
  DS402_Status_OperationEnable,
  DS402_Status_QuickStopActive,
  DS402_Status_FaultReactionActive = 0x20,
  DS402_Status_Fault,
}DS402_Status_t;

typedef enum DS402_COMMAND{
  DS402_Command_Shutdown = 0x0,
  DS402_Command_SwitchON,
  DS402_Command_DisableVoltage,
  DS402_Command_QuickStop,
  DS402_Command_DisableOperation,
  DS402_Command_EnableOperation,
  DS402_Command_FaultReset,
  DS402_Command_None = 0xFF,
}DS402_Command_t;

typedef enum DS02_ControlWordBit{
  DS402_CWB_SwitchON = 0x0,
  DS402_CWB_DisableVoltage,
  DS402_CWB_QuickStop,
  DS402_CWB_EnableOperation,
  DS402_CWB_OperationModeSpecific0,
  DS402_CWB_OperationModeSpecific1,
  DS402_CWB_OperationModeSpecific2,
  DS402_CWB_ResetFault,
  DS402_CWB_PauseOrHalt,
  DS402_CWB_Reserved0,
  DS402_CWB_Reserved1,
  DS402_CWB_ResponseMonitoring,
  DS402_CWB_ResetPosition,
  DS402_CWB_ManufactureSpecific0,
  DS402_CWB_ManufactureSpecific1,
  DS402_CWB_ManufactureSpecific2 = 15,
}ControlWordBit_t;

typedef enum DS402_ProfiledPositionControlWordBit{
  DS402_PPCWB_SetNewPoint = 4,
  DS402_PPCWB_ChangeSetImmediately = 5,
  DS402_PPCWB_AbsoluteOrRelativeMovement = 6,
  DS402_PPCWB_NewPointBuffered = 13,
}PPControlWordBit_t;

typedef enum DS402_InterpolatedPControlWordBit{
	DS402_IPCWB_IpModeActive = 12,
}IPControlWordBit_t;

typedef enum DS402_StatusWordBit{
	DS402_SWB_ReadyToSwitchON = 0x0,
	DS402_SWB_SwitchedON,
	DS402_SWB_OperationEnabled,
	DS402_SWB_Fault,
	DS402_SWB_VoltageEnabled,
	DS402_SWB_QuickStop,
	DS402_SWB_SwitchONDisabled,
	DS402_SWB_Warning,
	DS402_SWB_ManufacturerSpecific,
	DS402_SWB_Remote,
	DS402_SWB_TargetReached,
	DS402_SWB_InternalLimitActive,
	DS402_SWB_OperationModeSpecific0 = 12,
	DS402_SWB_OperationModeSpecific1 = 13,
	DS402_SWB_ManufacturerSpecific0 = 14,
	DS402_SWB_ManufacturerSpecific1 = 15,
}StatusWordBit_t;

typedef enum DS402_PPStatusWordBit{
	DS402_PPSWB_SetNewPointAcknowledge = 12,
	DS402_PPSWB_FollowingError = 13,
}PPStatusWordBit_t;

#define DS402_OperMode_NoMode          (int8_t)(-1)
#define DS402_OperMode_Reserved        (int8_t)(0)
#define DS402_OperMode_ProfilePosition (int8_t)(1)
#define DS402_OperMode_Velocity        (int8_t)(2)
#define DS402_OperMode_ProfileVelocity (int8_t)(3)
#define DS402_OperMode_TorqueProfiled  (int8_t)(4)
#define DS402_OperMode_Reserved1       (int8_t)(5)
#define DS402_OperMode_Homing          (int8_t)(6)
#define DS402_OperMode_InterpolatedPosition  (int8_t)(7)
#define DS402_OperMode_NotUsed         (int8_t)(8)

#define DS402_MT_NonStandardMotor      (uint16_t)(0)
#define DS402_MT_DC                    (uint16_t)(1)
#define DS402_MT_StpperMotor           (uint16_t)(9)
#define DS402_MT_SinPMBLDC             (uint16_t)(10)  // Sinusoidal PM bldc
#define DS402_MT_TrapPMBLDC            (uint16_t)(11)  // Trapezoidal PM bldc

typedef enum DS402_Statemachine_OperateResult{
	DS402_OPR_ErrorHappen = -3,
	DS402_OPR_InvalidCommand = -2,
	DS402_OPR_InvalidStatus = -1,
	DS402_OPR_OK = 0,
	DS402_OPR_StateNotChanged,
	DS402_OPR_StateChanged,
	DS402_OPR_StatusWordNotChanged,
	DS402_OPR_StatusWordChanged,
}DS402_OperateResult_t;

typedef enum DS402_Operate_Type{
	DS402_OP_Donothing,
	DS402_OP_NewControlWord,
	DS402_OP_QueryStatusWord,
	DS402_OP_ErrorHandle,
}DS402_OperateType_t;

typedef void (*DS402_OnStateChanged_Func_t)();
typedef void (*DS402_OnStatusUpdated_Func_t)();
typedef void (*DS402_OnStateError_Func_t)();

typedef struct DS402_StateData{
	DS402_OperateType_t operate;

	uint16_t   controlWord;
	uint16_t   newControlWord;

	DS402_Command_t  triggerdCommand;

	uint16_t  lastStatusWord;
	uint16_t  statusWord;

	DS402_Status_t   lastState;
	DS402_Status_t   state;

	DS402_OperateResult_t result;

	DS402_OnStateChanged_Func_t   pStateChangedFunc;
	DS402_OnStatusUpdated_Func_t  pStatusUpdatedFunc;
	DS402_OnStateError_Func_t     pStateErrorFunc;
}DS402_StateData_t;


typedef struct DS402_MotorData{
	uint16_t  type;
	char catalogNumber[16];
	char manufacturer[16];
}DS402_MotorData_t;

typedef struct DS402_SupportedModes{
	uint32_t supportedModes;
	int8_t   newMode;
	int8_t   curMode;
}DS402_MotorModes_t;

typedef struct DS402_Homing{
	int32_t  homeOffset;
	uint32_t homingSpeeds[2];
	uint32_t homingAcce;
	int8_t   homingMethod;
}DS402_Homing_t;

typedef struct DS402_InterpolatedPosition{
	int32_t*   dataRecord;
	void*      timePeriod;
	uint8_t*   syncDefinition;
	void*      dataConfig;
	int16_t    ipSubModeSelected;
}DS402_InterpolatedPosition_t;

typedef struct DS402_FactorParam{
	uint32_t  numerator;
	uint32_t  divisor;
}DS402_FactorParam_t;

typedef struct DS402_DriverFactors{
	uint32_t   posEncoderRatio[2];  // 0x068F
	uint32_t   velEncoderRatio[2];  // 0x6090

	uint32_t   posFactor[2];        // 0x6093
	uint32_t   velEncoderFactor[2]; // 0x6094

	uint32_t   velFactor[2];        // 0x6095
	uint32_t   accelFactor[2];      // 0x6097

	uint8_t    polarity;            // 0x707E
}Ds402_DriveFactors_t;

typedef void (*DS402_PDOSend_Func_t)(uint8_t pdoIdx, bool sync, uint32_t timeDifference_us);

#endif /* C402DEF_H_ */
