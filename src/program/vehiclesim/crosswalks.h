#ifndef CROSSWALK_SIM_H_
#define CROSSWALK_SIM_H_

#include <roadrunner.h>
#include <perception_interface.h>
#include <aw_roadNetwork.h>
#include <vector>
#include <obstacle_types.h>

namespace vlr {

struct SimCrosswalkPedestrian
{
    bool active;
    double speed;
    double utm_x, utm_y;
    double theta;
    double costheta;
    double sintheta;

    double current_x;
    double max_x;
    double next_time;
    char id[20];
};

class CrosswalkSimulator {

  struct CrosswalkParams {
      int generate_pedestrians;
      double max_wait_time;
      double min_speed;
      double max_speed;
    };

public:
  CrosswalkSimulator(rndf::RoadNetwork& rn, dgc::IpcInterface*& ipc);
  ~CrosswalkSimulator();

  void update (double time);

  inline int maxObstacles() {
    return pedestrians_.size();
  };

  inline int maxObstaclePoints() {
    return pedestrians_.size() * 16;
  };


  inline const PerceptionObstacles& getObstacles() {
    return obstacles_;
  };

private:

  void readParameters();

  dgc::IpcInterface * ipc_;

  CrosswalkParams params_;

  std::vector <SimCrosswalkPedestrian> pedestrians_;
//  vlr::rndf::RoadNetwork& rn_;

  double lastTime_;
  PerceptionObstacles obstacles_;

};

} // namespace vlr

#endif
