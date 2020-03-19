#ifndef BASE_DEMO_H_
#define BASE_DEMO_H_

#include <string>
#include <vlrException.h>
#include <applanix_interface.h>
#include <localize_interface.h>
#include <perception_interface.h>
#include "fakeObstacleTracker.h"

namespace vlr {

class BaseDemo {
public:
  BaseDemo(const std::string& rndf_name, const std::string& mdf_name, const double start_lat, const double start_lon, const double start_yaw);
  virtual ~BaseDemo();

  void updatePoses(bool init, const TrajectoryPoint2D& current_trajectory_point,
                   dgc::ApplanixPose& applanix_pose, dgc::LocalizePose& localize_pose);

  virtual void updateObstaclePredictions(double t, std::vector<ObstaclePrediction>& obstacle_predictions) = 0;

  //virtual void setFakeTrackerParams(double checked_horizon, double deltaT_sampling) = 0;

//  inline std::vector<BaseDemoCarState>& getCarStates() {return car_states_;}
  inline const PerceptionObstacles& getObstacleMessage() {return obstacle_msg_;}

protected:
    static const double meters2feet_ = 3.28083989501312;
//    FakeObstacleTracker* fake_tracker_;
    PerceptionObstacles obstacle_msg_;
    std::string rndf_name_;
    std::string mdf_name_;
    double start_lat_, start_lon_;
    double start_x_, start_y_, start_yaw_;
    std::string utm_zone_;
    double smooth_x_start_, smooth_y_start_;
    double current_offset_x_, current_offset_y_;
    double current_timestamp_;
};

} // namespace vlr

#endif // BASE_DEMO_H_
