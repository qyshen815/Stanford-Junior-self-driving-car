#ifndef TRACK_MANAGER_ROS_H
#define TRACK_MANAGER_ROS_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <float.h>

#include <sensor_msgs/PointCloud.h>

#define TRACK_SERIALIZATION_VERSION 2
#define TRACK_MANAGER_SERIALIZATION_VERSION 2
#define SEGMENT_SERIALIZATION_VERSION 0

typedef struct {
  double x; // meters
  double y;
  double z;
  double roll; // radians
  double pitch;
  double yaw;
} pose_t;

class Segment {
public:
  //! Points are stored in global coordinates in meters.
  //! Intensity values are stored in the first channel.
  boost::shared_ptr<sensor_msgs::PointCloud> cloud_;
  //! In seconds.
  double timestamp_;
  pose_t robot_pose_;

  void serialize(std::ostream& out) const;
  bool deserialize(std::istream& istrm);
};
 
class Track {
public:
  //! The object class, e.g. "car", "background", etc.
  std::string label_;
  std::vector< boost::shared_ptr<Segment> > segments_;

  //! Initialized as "unlabeled" with no segments.
  Track();
  void serialize(std::ostream& out) const;
  bool deserialize(std::istream& istrm);
};

class TrackManager {
public:
  std::vector< boost::shared_ptr<Track> > tracks_;

  TrackManager();
  //! Load from file.
  TrackManager(const std::string& filename);
  //! Save to file.
  void save(const std::string& filename) const;
  void serialize(std::ostream& out) const;
  bool deserialize(std::istream& istrm);
};

// -- Helper functions for saving and loading.
bool deserializePointCloudROS(std::istream& istrm, sensor_msgs::PointCloud* cloud);
void serializePointCloudROS(const sensor_msgs::PointCloud& cloud, std::ostream& out);
   
#endif //TRACK_MANAGER_H
