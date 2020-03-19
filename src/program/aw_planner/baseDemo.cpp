#include <time.h>
#include <stdio.h>

#include <stdint.h>
#include <iostream>
#include <vector>

#include <vlrException.h>
#include <lltransform.h>

#include "baseDemo.h"

using namespace dgc;
using namespace std;

namespace vlr {

BaseDemo::BaseDemo(const std::string& rndf_name, const std::string& mdf_name, const double start_lat, const double start_lon, const double start_yaw) :
                       rndf_name_(rndf_name), mdf_name_(mdf_name), start_lat_(start_lat), start_lon_(start_lon), start_yaw_(start_yaw),
                       smooth_x_start_(0), smooth_y_start_(0), current_offset_x_(0), current_offset_y_(0) {

  current_timestamp_ = Time::current();

  latLongToUtm(start_lat_, start_lon_, &start_x_,  &start_y_, utm_zone_);

//  fake_tracker_ = new FakeObstacleTracker(car_states_);
  memset(&obstacle_msg_, 0, sizeof(obstacle_msg_));
//  obstacle_msg_.num_dynamic_obstacles = num_fast_cars_ + num_slow_cars_ + num_trucks_;
//  obstacle_msg_.dynamic_obstacle = new PerceptionDynamicObstacle[obstacle_msg_.num_dynamic_obstacles];
  strcpy(obstacle_msg_.host, dgc_hostname());
}


BaseDemo::~BaseDemo() {

}

void BaseDemo::updatePoses(bool init, const TrajectoryPoint2D& current_trajectory_point,
                             dgc::ApplanixPose& applanix_pose, dgc::LocalizePose& localize_pose) {

  memset(&applanix_pose, 0, sizeof(applanix_pose));
  memset(&localize_pose, 0, sizeof(localize_pose));

  strcpy(localize_pose.utmzone, utm_zone_.c_str());

  if(init) {
    applanix_pose.ar_yaw = 0;
    applanix_pose.speed = 0;
    applanix_pose.altitude = 0;
    applanix_pose.yaw = start_yaw_;
    applanix_pose.pitch = 0;
    applanix_pose.roll = 0;
    applanix_pose.a_x = 0;
    applanix_pose.a_y = 0;
    applanix_pose.a_z = 0;
    smooth_x_start_ = current_trajectory_point.x;
    smooth_y_start_ = current_trajectory_point.y;
  }
  else {
    applanix_pose.ar_yaw  = current_trajectory_point.kappa * current_trajectory_point.v;
    applanix_pose.speed = current_trajectory_point.v;
    applanix_pose.altitude = 0;
    applanix_pose.yaw = current_trajectory_point.theta;
    applanix_pose.pitch = 0;
    applanix_pose.roll = 0;
    applanix_pose.a_x = current_trajectory_point.a*cos(current_trajectory_point.theta) - current_trajectory_point.a_lat*sin(current_trajectory_point.theta);
    applanix_pose.a_y = current_trajectory_point.a*sin(current_trajectory_point.theta) + current_trajectory_point.a_lat*cos(current_trajectory_point.theta);
    applanix_pose.a_z = 0;
  }

  applanix_pose.smooth_x = current_trajectory_point.x - smooth_x_start_;
  applanix_pose.smooth_y = current_trajectory_point.y - smooth_y_start_;

  localize_pose.x_offset = current_trajectory_point.x - applanix_pose.smooth_x;
  localize_pose.y_offset = current_trajectory_point.y - applanix_pose.smooth_y;

  current_offset_x_ = localize_pose.x_offset;
  current_offset_y_ = localize_pose.y_offset;
  current_timestamp_ = current_trajectory_point.t;

  utmToLatLong(applanix_pose.smooth_x+localize_pose.x_offset, applanix_pose.smooth_y+localize_pose.y_offset, utm_zone_, &applanix_pose.latitude, &applanix_pose.longitude);

  applanix_pose.timestamp = current_timestamp_;
  localize_pose.timestamp = current_timestamp_;
//  printf("smx: %f, smy: %f, offx: %f, offy: %f, lat: %f, long: %f\n",applanix_pose.smooth_x, applanix_pose.smooth_y,
//         localize_pose.x_offset, localize_pose.y_offset,
//         applanix_pose.latitude, applanix_pose.longitude);
//    return;
}

} // namespace vlr
