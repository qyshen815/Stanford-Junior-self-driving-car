#ifndef DGC_TRAFFICLIGHS_SIM_H
#define DGC_TRAFFICLIGHS_SIM_H

#include <roadrunner.h>
#include <perception_interface.h>
#include <traffic_light_messages.h>
#include <aw_roadNetwork.h>
#include <vector>

namespace vlr {
static const int NUM_LIGHT_GROUPS = 8;

struct TrafficLight
{
  TrafficLightState state;
  int group;
};

class TrafficLightSimulator {

  struct TrafficLightParams {
    double state_duration;
    double yellow_duration;
    int switch_light_states_;
  };

public:
  TrafficLightSimulator(rndf::RoadNetwork& rn, dgc::IpcInterface*& ipc);
  ~TrafficLightSimulator();

  void update (double time);

private:

  void readParameters();

  TrafficLightParams params_;

  TrafficLightList ipc_light_list;

  std::vector <TrafficLight> lights_;
//  vlr::rndf::RoadNetwork& rn_;
  dgc::IpcInterface* ipc_;

  int state_;
  double lastTime_;

  bool switching_;

};
} // namespace vlr
#endif
