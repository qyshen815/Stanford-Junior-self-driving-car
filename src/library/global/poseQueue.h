#ifndef GLOBAL_POSEQUEUE_H_
#define GLOBAL_POSEQUEUE_H_

#include <map>
#include <stdint.h>

#include "pose.h"

namespace vlr {

class PoseQueue {
 public:
  PoseQueue(uint32_t queue_size);
  virtual ~PoseQueue();

  inline bool empty() const {return queue_.empty();}

  void push(const Pose& pose, double timestamp);

  void xy(double timestamp, double& x, double& y);
  void xyAndOffsets(double timestamp, double& x, double& y, double& offset_x, double& offset_y);
  void offsets(double timestamp, double& offset_x, double& offset_y);
  void utmXY(double timestamp, double& utm_x, double& utm_y);
  void xyAndUtmXY(double timestamp, double& x, double& y, double& utm_x, double& utm_y);
  Pose pose(double timestamp);

 private:
  int bracketPose(double timestamp,
        std::map<double, Pose>::const_iterator* before, std::map<double, Pose>::const_iterator* after);
  double interpolateYaw(double head1, double head2, double fraction);

    std::map <double, Pose> queue_; // ok, it's a map, so what? ...
  uint32_t queue_size_;
};

} // namespace vlr
#endif
