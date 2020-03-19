#ifndef AW_TRAFFIC_LIGHT_MANAGER_H_
#define AW_TRAFFIC_LIGHT_MANAGER_H_

#include <string>
#include <set>
#include <map>

#include <aw_roadNetwork.h>

namespace vlr {

class Topology;

class TrafficLightManager {
public:
	TrafficLightManager(Topology* top);
	~TrafficLightManager();

	bool hasToStop(std::map<std::string, TrafficLightState*>& traffic_light_states);

 private:
	Topology* top_;
	RndfGraph* graph_;
	std::vector<std::string> traffic_light_names_;
};

} // namespace vlr

#endif // AW_TRAFFIC_LIGHT_MANAGER_H_
