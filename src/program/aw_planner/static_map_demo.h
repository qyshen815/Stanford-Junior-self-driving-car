#ifndef STATIC_MAP_DEMO_H_
#define STATIC_MAP_DEMO_H_

#include <string>
#include <applanix_interface.h>
#include <localize_interface.h>
#include <perception_interface.h>

#include "baseDemo.h"
//#include "fakeObstacleTracker.h"

namespace vlr {
class StaticMapDemo : public BaseDemo{
public:
  StaticMapDemo(const std::string& rndf_name, const std::string& mdf_name, const std::string& map_name,
             const double start_lat, const double start_lon, const double start_yaw);
  virtual ~StaticMapDemo();

  void updateObstaclePredictions(double t, std::vector<ObstaclePrediction>& obstacle_predictions);
  void readObstacleMap(const std::string& map_name);

private:
  uint32_t img_id_;
  int32_t static_obstacle_map_size_x_;
  int32_t static_obstacle_map_size_y_;
  uint8_t* map_data_;
  double static_obstacle_map_resolution_;
  bool new_map_read_;
};

} // namespace vlr

#endif // STATIC_MAP_DEMO_H_
