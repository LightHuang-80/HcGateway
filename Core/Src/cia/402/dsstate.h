/*
 * dsstate.h
 *
 *  Created on: 2020��5��27��
 *      Author: Administrator
 */

#ifndef DSSTATE_H_
#define DSSTATE_H_

void DS402_StateManage_Init(void);
void DS402_StateManage_Process(void);

DS402_Status_t DS402_getStatus(uint16_t statusWord);
uint16_t DS402_buildControlWordByCommand(uint16_t controlWord, DS402_Command_t command);
#endif /* DSSTATE_H_ */
