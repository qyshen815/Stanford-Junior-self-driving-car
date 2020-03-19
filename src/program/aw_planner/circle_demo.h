/*
 * circle_demo.h
 *
 *  Created on: Dec 15, 2009
 *      Author: soeren
 */

#ifndef CIRCLE_DEMO_H_
#define CIRCLE_DEMO_H_

#include <string>
#include <applanix_interface.h>
#include <localize_interface.h>
#include <perception_interface.h>

#include "baseDemo.h"
#include "fakeObstacleTracker.h"

namespace vlr {

class CircleDemo : public BaseDemo {
public:
  CircleDemo(const std::string& rndf_name, const std::string& mdf_name, const double start_lat, const double start_lon, const double base_r);
  virtual ~CircleDemo();

  inline void updateObstaclePredictions(double t, std::vector<ObstaclePrediction>& obstacle_predictions) {
    obstacle_msg_.num_dynamic_obstacles = 0;//num_fast_cars_ + num_slow_cars_ + num_trucks_;
 //   fake_tracker_->predictObstacles(t, current_offset_x_, current_offset_y_, obstacle_predictions, obstacle_msg_);
  }

  inline void setFakeTrackerParams(double checked_horizon, double deltaT_sampling) {
    fake_tracker_->setParams(checked_horizon, deltaT_sampling);
  }

  inline std::vector<CircleDemoCarState>& getCarStates() {return car_states_;}

private:
  bool createMultiCircleRNDF();
  bool createMultiCircleMDF(std::vector<uint32_t>& checkpoints);
  void generateTraffic(uint32_t num_fast_cars, uint32_t num_slow_cars, uint32_t num_trucks);

private:
  FakeObstacleTracker* fake_tracker_;
  std::vector<CircleDemoCarState> car_states_;
  double base_r_;
  double c_x_, c_y_;
  uint32_t num_lanes_;
  double lane_width_;
  uint32_t num_fast_cars_;
  uint32_t num_slow_cars_;
  uint32_t num_trucks_;

};

} // namespace vlr

#endif // CIRCLE_DEMO_H_
