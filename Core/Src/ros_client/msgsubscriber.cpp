/*
 * msgsubscriber.cpp
 *
 *  Created on: 2020年9月23日
 *      Author: Administrator
 */
#include <stdlib.h>
#include <string.h>
#include "ros/subscriber.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "msgsubscriber.h"
#include "pdom.h"

void MsgSubscriberCallback(const std_msgs::String &msg)
{
	/*Receive the 0x607A target position message from moveit,
	 *String format:  Hnn:xxxxxxx,yyyyy,zzzzz for example,
	 *100|H04:2.5678,0.234,0.111,H05:2.22,0.12,0.33
	 * serial number 100,
	 * send nodename(H04),
	 * target position: 2.5678(R), velocity: 0.234, accelerate: 0.111
 	 * send nodename(H05),
	 * target position: 2.22(R), velocity: 0.12, accelerate: 0.33
	 * */

	int8_t nodeIdx = 0;

	float pos  = 0.0f;
    float vel  = 0.0f;
    float acce = 0.0f;

	uint32_t length_data = strlen(msg.data);

	uint32_t idx = 0;
    int8_t   key = 0;
    uint8_t  ilen = 0;

    bool found = false;

    int32_t  serialNum = 0;
    uint32_t i = 0;

    char ele[32];
    for (i = 0; i < length_data; i++) {
    	if (msg.data[i] == '|' && idx > 0) {
    		ele[idx] = 0;
    		found = true;
    	}

    	if (found) {
    		serialNum = atoi(ele);
    		break;
    	}

    	ele[idx++] = msg.data[i];
    }

    if (!found) {
    	/*Cannot find the msg serial number frame*/
    	return;
    }

    i++; /*skip the '|' char*/
    ilen = 0;
    idx = 0;

    for (; i < length_data; i++) {
        ele[idx++] = msg.data[i];

        found = false;
        if (msg.data[i] == ':' || msg.data[i] == ',' || msg.data[i] == ' ' ||
			i == (length_data - 1)){
          ele[idx] = 0;
          idx = 0;
          found = true;
    	}else{
          ilen ++;
    	}

        if (found) {
        	if (ilen == 1 && ele[0] == '-'){
        		ele[0] = '0';
        	}
        	ilen = 0;

        	switch(key) {
        	case 0:
        		nodeIdx = atoi((ele+1));
        		break;
        	case 1:
                pos = atoff(ele);
        		break;
        	case 2:
                vel = atoff(ele);
        		break;
        	case 3:
                acce = atoff(ele);
        		break;

        	default:
        		break;
        	}

        	key++;
        }

        if (key == 4) {
        	// 4 elements per node
        	key = 0;
        	PDOM_sendTPDOs(nodeIdx - 1, serialNum, pos, vel, acce);
        }
    }
}
