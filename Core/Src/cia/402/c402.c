/*
 * c402.c
 *
 *  Created on: 2020年8月3日
 *      Author: Administrator
 */

#include "main.h"
#include "c402def.h"
#include "dspp.h"

/* Service:
 * a) Maintain the global nodes status
 * b) report status to upper device
 *
 * 1. Send control command
 * 2. Monitor node status
 * 3. Send node position
 * 4. Increase steps
 * 5. Setting velocity parameters, accelerate and decelerate
 * 6. plug and play
 * */


void DS402_Start_Node(uint8_t idx, ProfiledPosition_t * pp)
{
	/*Send Switch on command
	 * control word = 0x0F*/


}

void DS402_Stop_Node(uint8_t idx)
{
	/*Send Shutdown command
	 * control word = 0x06*/

}

