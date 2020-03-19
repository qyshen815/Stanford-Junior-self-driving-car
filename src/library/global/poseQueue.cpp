#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>

#include "vlrException.h"
#include "poseQueue.h"

namespace vlr {
PoseQueue::PoseQueue(uint32_t queue_size) {

  if (queue_size == 0) {
    throw Exception("Invalid queue size (zero) requested.");
  }
  queue_size_ = queue_size;
}

PoseQueue::~PoseQueue() {
}

void PoseQueue::push(const Pose& pose, double timestamp) {
  if (queue_.size() > queue_size_) {
    queue_.erase(queue_.begin());
  }
  queue_.insert(std::make_pair(timestamp, pose));
}

int PoseQueue::bracketPose(double timestamp, std::map<double, Pose>::const_iterator* before,
    std::map<double, Pose>::const_iterator* after) {

  std::map<double, Pose>::const_iterator pit = queue_.lower_bound(timestamp);
  if (pit == queue_.end()) {
    std::stringstream s;
    if (queue_.empty()) {
      s << "Pose queue is empty.";
      throw Exception(s.str());
    }
    //    else {
    //        s << std::fixed << std::setprecision(3) << "Requested timestamp (" << timestamp << ") is newer than newest queue entry (" << (--queue_.end())->first << ").";
    //    }
    *before = --queue_.end(); // TODO: implement proper extrapolation
    *after = *before;
    (*after)--;
    return 1;
  }

  *after = pit; // btw...who named lower_bound ?!?

  if (pit == queue_.begin()) {
    if (pit->first == timestamp) {
      *before = pit;
      return 0;
    }

    //      std::stringstream s;
    //      s << std::fixed << std::setprecision(3) << "Requested timestamp (" << timestamp << ") is older than oldest queue entry (" << queue_.begin()->first << ").";
    //      throw Exception(s.str());
    *before = queue_.begin();
    *after = queue_.begin();
    (*after)++;
    return -1;
  }

  *before = --pit;
  return 0;
}

void PoseQueue::xy(double timestamp, double& x, double& y) {
  const Pose& tpose = (--queue_.end())->second;
  x = tpose.x();
  y = tpose.y();
  return;

  // empty case is handle in bracketPose
  if (queue_.size() == 1) {
    const Pose& pose = queue_.begin()->second;
    x = pose.x();
    y = pose.y();
    return;
  }

  std::map<double, Pose>::const_iterator before, after;
  int polation_mode = bracketPose(timestamp, &before, &after);
  const Pose& pose1 = before->second;
  const Pose& pose2 = after->second;

  if (polation_mode != 0) {
    double ts1 = before->first;
    double ts2 = after->first;
    double dt = ts1 - ts2;

    x = pose1.x() + (pose1.x() - pose2.x()) / dt * (timestamp - ts1);
    y = pose1.y() + (pose1.y() - pose2.y()) / dt * (timestamp - ts1);
    return;
  }

  double frac = (timestamp - before->first) / (after->first - before->first);

  x = pose1.x() + frac * (pose2.x() - pose1.x());
  y = pose1.y() + frac * (pose2.y() - pose1.y());

  //pose->x = pose1->x + frac * (pose2->x - pose1->x);
  //pose->y = pose1->y + frac * (pose2->y - pose1->y);
  //pose->z = pose1->z + frac * (pose2->z - pose1->z);
  //pose->roll = pose1->roll + frac * (pose2->roll - pose1->roll);
  //pose->pitch = pose1->pitch + frac * (pose2->pitch - pose1->pitch);
  //pose->yaw = interpolate_yaw(pose1->yaw, pose2->yaw, frac);
}

void PoseQueue::offsets(double timestamp, double& offset_x, double& offset_y) {
  const Pose& tpose = (--queue_.end())->second;
  offset_x = tpose.offsetX();
  offset_y = tpose.offsetY();
  return;
  // empty case is handle in bracketPose
  if (queue_.size() == 1) {
    const Pose& pose = queue_.begin()->second;
    offset_x = pose.offsetX();
    offset_y = pose.offsetY();
    return;
  }

  std::map<double, Pose>::const_iterator before, after;
  int polation_mode = bracketPose(timestamp, &before, &after);
  const Pose& pose1 = before->second;
  const Pose& pose2 = after->second;

  if (polation_mode != 0) {
    double ts1 = before->first;
    double ts2 = after->first;
    double dt = ts1 - ts2;

    offset_x = pose1.offsetX() + (pose1.offsetX() - pose2.offsetX()) / dt * (timestamp - ts1);
    offset_y = pose1.offsetY() + (pose1.offsetY() - pose2.offsetY()) / dt * (timestamp - ts1);
    return;
  }

  double frac = (timestamp - before->first) / (after->first - before->first);

  offset_x = pose1.offsetX() + frac * (pose2.offsetX() - pose1.offsetX());
  offset_y = pose1.offsetY() + frac * (pose2.offsetY() - pose1.offsetY());
}

void PoseQueue::xyAndOffsets(double timestamp, double& x, double& y, double& offset_x, double& offset_y) {
  const Pose& tpose = (--queue_.end())->second;
  x = tpose.x();
  y = tpose.y();
  offset_x = tpose.offsetX();
  offset_y = tpose.offsetY();
  return;
  // empty case is handle in bracketPose
  if (queue_.size() == 1) {
    const Pose& pose = queue_.begin()->second;
    x = pose.x();
    y = pose.y();
    offset_x = pose.offsetX();
    offset_y = pose.offsetY();
    return;
  }

  std::map<double, Pose>::const_iterator before, after;
  int polation_mode = bracketPose(timestamp, &before, &after);
  const Pose& pose1 = before->second;
  const Pose& pose2 = after->second;

  if (polation_mode != 0) {
    double ts1 = before->first;
    double ts2 = after->first;
    double dt = ts1 - ts2;

    x = pose1.x() + (pose1.x() - pose2.x()) / dt * (timestamp - ts1);
    y = pose1.y() + (pose1.y() - pose2.y()) / dt * (timestamp - ts1);
    offset_x = pose1.offsetX() + (pose1.offsetX() - pose2.offsetX()) / dt * (timestamp - ts1);
    offset_y = pose1.offsetY() + (pose1.offsetY() - pose2.offsetY()) / dt * (timestamp - ts1);
    return;
  }

  double frac = (timestamp - before->first) / (after->first - before->first);

  x = pose1.x() + frac * (pose2.x() - pose1.x());
  y = pose1.y() + frac * (pose2.y() - pose1.y());
  offset_x = pose1.offsetX() + frac * (pose2.offsetX() - pose1.offsetX());
  offset_y = pose1.offsetY() + frac * (pose2.offsetY() - pose1.offsetY());
}

void PoseQueue::utmXY(double timestamp, double& utm_x, double& utm_y) {
  double x, y, offset_x, offset_y;
  xyAndOffsets(timestamp, x, y, offset_x, offset_y);
  utm_x = x + offset_x;
  utm_y = y + offset_y;
}

void PoseQueue::xyAndUtmXY(double timestamp, double& x, double& y, double& utm_x, double& utm_y) {
  // utm_x and utm_y are also used to temporarily store offsets
  xyAndOffsets(timestamp, x, y, utm_x, utm_y);
  utm_x += x;
  utm_y += y;
}

Pose PoseQueue::pose(double timestamp) {
  const Pose& tpose = (--queue_.end())->second;
  return tpose;
  Pose res_pose;

    // empty case is handle in bracketPose
  if (queue_.size() == 1) {
    return queue_.begin()->second;
  }

  std::map<double, Pose>::const_iterator before, after;
  int polation_mode = bracketPose(timestamp, &before, &after);
  const Pose& pose1 = before->second;
  const Pose& pose2 = after->second;

  if (polation_mode != 0) {
    double ts1 = before->first;
    double ts2 = after->first;
    double dt = ts1 - ts2;

    res_pose.x() = pose1.x() + (pose1.x() - pose2.x()) / dt * (timestamp - ts1);
    res_pose.y() = pose1.y() + (pose1.y() - pose2.y()) / dt * (timestamp - ts1);
    res_pose.z() = pose1.z() + (pose1.z() - pose2.z()) / dt * (timestamp - ts1);
    res_pose.offsetX() = pose1.offsetX() + (pose1.offsetX() - pose2.offsetX()) / dt * (timestamp - ts1);
    res_pose.offsetY() = pose1.offsetY() + (pose1.offsetY() - pose2.offsetY()) / dt * (timestamp - ts1);
    res_pose.offsetZ() = pose1.offsetZ() + (pose1.offsetZ() - pose2.offsetZ()) / dt * (timestamp - ts1);

      // TODO: implement proper angle extrapolation
    res_pose.yaw() = pose1.yaw();// + (pose1.yaw() - pose2.yaw()) / dt * (timestamp - ts1);
    res_pose.pitch() = pose1.pitch() + (pose1.pitch() - pose2.pitch()) / dt * (timestamp - ts1);
    res_pose.roll() = pose1.roll() + (pose1.roll() - pose2.roll()) / dt * (timestamp - ts1);
    return res_pose;
  }

  double frac = (timestamp - before->first) / (after->first - before->first);

  res_pose.x() = pose1.x() + frac * (pose2.x() - pose1.x());
  res_pose.y() = pose1.y() + frac * (pose2.y() - pose1.y());
  res_pose.z() = pose1.z() + frac * (pose2.z() - pose1.z());
  res_pose.offsetX() = pose1.offsetX() + frac * (pose2.offsetX() - pose1.offsetX());
  res_pose.offsetY() = pose1.offsetY() + frac * (pose2.offsetY() - pose1.offsetY());
  res_pose.offsetZ() = pose1.offsetZ() + frac * (pose2.offsetZ() - pose1.offsetZ());

  res_pose.yaw() = interpolateYaw(pose1.yaw(), pose2.yaw(), frac);
  res_pose.pitch() = pose1.pitch() + frac * (pose2.pitch() - pose1.pitch());
  res_pose.roll() = pose1.roll() + frac * (pose2.roll() - pose1.roll());

  res_pose.v() = pose1.v() + frac * (pose2.v() - pose1.v());
  res_pose.vLateral() = pose1.vLateral() + frac * (pose2.vLateral() - pose1.vLateral());
  res_pose.vZ() = pose1.vZ() + frac * (pose2.vZ() - pose1.vZ());

  res_pose.a() = pose1.a() + frac * (pose2.a() - pose1.a());
  res_pose.aLateral() = pose1.aLateral() + frac * (pose2.aLateral() - pose1.aLateral());
  res_pose.aZ() = pose1.aZ() + frac * (pose2.aZ() - pose1.aZ());
  return res_pose;
}

  // TODO: verify this works...
double PoseQueue::interpolateYaw(double head1, double head2, double fraction) {
  double result;

  if(head1 > 0 && head2 < 0 && head1 - head2 > M_PI) {
    head2 += 2 * M_PI;
    result = head1 + fraction * (head2 - head1);
    if(result > M_PI) {result -= 2 * M_PI;}
    return result;
  }
  else if(head1 < 0 && head2 > 0 && head2 - head1 > M_PI) {
    head1 += 2 * M_PI;
    result = head1 + fraction * (head2 - head1);
    if(result > M_PI) {result -= 2 * M_PI;}
    return result;
  }

return head1 + fraction * (head2 - head1);
}

} // namespace vlr
