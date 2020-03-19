/*
 * msgs.h
 *
 *  Created on: Jan 6, 2011
 *      Author: erublee
 */

#ifndef opencv_candidate_ROS_MSGS_H_
#define opencv_candidate_ROS_MSGS_H_
#include <sensor_msgs/CameraInfo.h>
#include <opencv_candidate/Camera.h>

namespace opencv_candidate{
  Camera fromRosMsg(sensor_msgs::CameraInfoConstPtr camera_info);
}
#endif /* opencv_candidate_ROS_MSGS_H_ */
