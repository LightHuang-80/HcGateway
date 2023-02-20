/*
 * msgsubscriber.h
 *
 *  Created on: 2020年9月23日
 *      Author: Administrator
 */

#ifndef SRC_ROS_CLIENT_MSGSUBSCRIBER_H_
#define SRC_ROS_CLIENT_MSGSUBSCRIBER_H_

#include "ros.h"
#include "rosserial_msgs/TopicInfo.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "ros/subscriber.h"

#ifdef __cplusplus
extern "C" {
#endif

void MsgSubscriberCallback(const std_msgs::String &msg);

#ifdef __cplusplus
}
#endif

#endif /* SRC_ROS_CLIENT_MSGSUBSCRIBER_H_ */
