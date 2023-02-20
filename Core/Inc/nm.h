/*
 * nm.h
 *
 *  Created on: 2020年9月10日
 *      Author: Administrator
 */

#ifndef NM_NM_H_
#define NM_NM_H_

#ifdef __cplusplus
extern "C" {
#endif

uint8_t NM_New();
uint8_t NM_Init(void *CANdevice0Index, uint8_t nodeId);

#ifdef __cplusplus
}
#endif

#endif /* NM_NM_H_ */
