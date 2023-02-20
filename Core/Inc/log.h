/*
 * log.h
 *
 *  Created on: 2021年4月4日
 *      Author: Administrator
 */

#ifndef INC_LOG_H_
#define INC_LOG_H_

#define LOG_ErrorLevel    0u
#define LOG_SysLevel      1u
#define LOG_WarningLevel  2u
#define LOG_InfoLevel     3u
#define LOG_DbgLevel      4u

#define LOG_MonOut        0u
#define LOG_USARTOut      1u
#define LOG_SWDOut        2u

#ifdef __cplusplus
extern "C" {
#endif

void LOG_Init(uint8_t level, uint8_t on, uint8_t output);
void LOG_Print(uint8_t level, const char* fmt, ...);
void LOG_SetLevel(uint8_t level);
void LOG_OnOff(uint8_t on);

#ifdef __cplusplus
}
#endif




#endif /* INC_LOG_H_ */
