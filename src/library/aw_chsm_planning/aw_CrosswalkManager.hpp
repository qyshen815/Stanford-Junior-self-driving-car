#ifndef AW_CROSSWALKMANAGER_H
#define AW_CROSSWALKMANAGER_H

#include <string>
#include <set>
#include <map>

//#include <aw_RndfGraph.h>
//#include <aw_RndfGraphSearch.h>
#include <aw_roadNetwork.h>
#include <obstaclePrediction.h>

namespace vlr {

#define CROSSWALK_MIN_FREE_TIME 1.5

class Topology;

class CrosswalkManager {
public:
	CrosswalkManager(Topology* top);
	~CrosswalkManager();

	bool isOccupied(std::vector<ObstaclePrediction>& pedestrians);
  inline static void minFree(double t) {min_free_time_ = t;}
  inline static double minFree() {return min_free_time_;}

 private:
  bool pedestriansOnCrosswalk(rndf::Crosswalk* crosswalk, std::vector<ObstaclePrediction>& pedestrians);

 private:
	Topology* top_;
	RndfGraph* graph_;
	std::vector<rndf::Crosswalk*> crosswalks_;
  double last_occupied_;
  static double min_free_time_;
};

} // namespace vlr

#endif // AW_CROSSWALKMANAGER_H
