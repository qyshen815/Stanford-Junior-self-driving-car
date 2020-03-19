#include <sys/types.h>
#include <algorithm>

#include <aw_Topology.hpp>

#include "aw_ChsmPlanner.hpp"
#include "aw_TrafficLightManager.hpp"

using namespace std;

namespace vlr {

#undef TRACE
//#define TRACE(str) cout << "[TrafficLightManager] " << str << endl;
#define TRACE(str)

TrafficLightManager::TrafficLightManager(Topology* top) : top_(top), graph_(NULL) {
	if(!top_) {throw vlr::Exception("zero pointer to topology");}
	graph_ = top_->complete_graph;
  if(!graph_) {throw vlr::Exception("zero pointer to complete graph");}

  std::vector<std::string> tl_names;
  top->dist_to_next_traffic_light(&tl_names, NULL);
  if(tl_names.empty()) {throw vlr::Exception("could not determine associated traffic light names");}

  std::vector<std::string>::const_iterator tlnit=tl_names.begin(), tlnit_end=tl_names.end();
  for(; tlnit != tlnit_end; tlnit++) {
//    rndf::TrafficLight* tl = const_cast<rndf::RoadNetwork*>(&top->roadNetwork())->trafficLight(*tlnit);
//    if(tl) {traffic_light_names_.push_back(tl);}
    traffic_light_names_.push_back(*tlnit);
  }
  if(traffic_light_names_.empty()) {throw vlr::Exception("could not determine associated traffic lights");}
}

TrafficLightManager::~TrafficLightManager() {
}

bool TrafficLightManager::hasToStop(std::map<std::string, TrafficLightState*>& traffic_light_states) {
  std::map<std::string, TrafficLightState*>::const_iterator tlsit = traffic_light_states.find(traffic_light_names_[0]);
  if (tlsit != traffic_light_states.end()) {
    printf("Current TLID: %s (%c)\n", traffic_light_names_[0].c_str(), (*tlsit).second->state);
    return (*tlsit).second->state != 'g';
  }

  std::cout << "Warning: state of traffic light " << traffic_light_names_[0] << " was requested but is unavailable...assuming red...\n";
  return true;
}

} // namespace vlr
