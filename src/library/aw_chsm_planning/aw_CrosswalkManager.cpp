#include <sys/types.h>
#include <algorithm>

#include <aw_Topology.hpp>

#include "aw_ChsmPlanner.hpp"
#include "aw_CrosswalkManager.hpp"

using namespace std;

namespace vlr {

#undef TRACE
//#define TRACE(str) cout << "[CrosswalkManager] " << str << endl;
#define TRACE(str)

CrosswalkManager::CrosswalkManager(Topology* top) : top_(top), graph_(NULL), last_occupied_(0) {
	if(!top_) {throw vlr::Exception("zero pointer to topology");}
	graph_ = top_->complete_graph;
  if(!graph_) {throw vlr::Exception("zero pointer to complete graph");}

  std::vector<std::string> cw_names;
  top->dist_to_next_crosswalk(&cw_names, NULL);
  if(cw_names.empty()) {throw vlr::Exception("could not determine associated crosswalk names");}

  std::vector<std::string>::const_iterator cwnit=cw_names.begin(), cwnit_end=cw_names.end();
  for(; cwnit != cwnit_end; cwnit++) {
    rndf::Crosswalk* cw = const_cast<rndf::RoadNetwork*>(&top->roadNetwork())->crosswalk(*cwnit);
    if(cw) {crosswalks_.push_back(cw);}
  }
  if(crosswalks_.empty()) {throw vlr::Exception("could not determine associated crosswalks");}
}

CrosswalkManager::~CrosswalkManager() {
}

bool CrosswalkManager::isOccupied(std::vector<ObstaclePrediction>& pedestrians) {

  std::vector<rndf::Crosswalk*>::const_iterator cwit=crosswalks_.begin(), cwit_end=crosswalks_.end();
  for(; cwit != cwit_end; cwit++) {
    if(pedestriansOnCrosswalk(*cwit, pedestrians)) {
      printf("CROSSWALK OCCUPIED!\n");
      last_occupied_ = Time::current();
      return true;
    }
  }


  if ((Time::current() - last_occupied_) > min_free_time_) {
    printf("CROSSWALK FREE!\n");
//    printf("CROSSWALK FREE FOR %f sec!\n", min_free_time_);
    return false;
  }

  return true;
}

bool CrosswalkManager::pedestriansOnCrosswalk(rndf::Crosswalk* crosswalk, std::vector<ObstaclePrediction>& pedestrians) {
  std::vector<ObstaclePrediction>::const_iterator pit = pedestrians.begin(), pit_end = pedestrians.end();
  for (; pit != pit_end; pit++) {
//    printf("testing pedestrian\n");
    std::vector<MovingBox>::const_iterator predit = (*pit).predicted_traj_.begin(), predit_end = (*pit).predicted_traj_.end();

    for (; predit != predit_end; predit++) {

      double px = (*predit).x;// + localize_pose_x_offset;
      double py = (*predit).y;// + localize_pose_y_offset;
      double r;

      if (crosswalk->width() == 0) {r = 2.5;} else {r = crosswalk->width() / 2.0;}

      double theta = atan2(crosswalk->utm_y2() - crosswalk->utm_y1(), crosswalk->utm_x2() - crosswalk->utm_x1());
      double dx = r * cos(theta + M_PI_2);
      double dy = r * sin(theta + M_PI_2);

      double lx = crosswalk->utm_x2() + dx - (crosswalk->utm_x1() + dx);
      double ly = crosswalk->utm_y2() + dy - (crosswalk->utm_y1() + dy);

      double lpx = px - (crosswalk->utm_x1() + dx);
      double lpy = py - (crosswalk->utm_y1() + dy);

//      printf("%f, %f, %f, %f\n", px, py, crosswalk->utm_x1(), crosswalk->utm_y1());
      //    printf("%f, %f, %f, %f, %f, %f, %f\n", r, px, py, lx, ly, lpx, lpy);

      if (lx * lpx + ly * lpy < 0) {
        continue;
      }

      lx = crosswalk->utm_x1() - dx - (crosswalk->utm_x1() + dx);
      ly = crosswalk->utm_y1() - dy - (crosswalk->utm_y1() + dy);

      if (lx * lpx + ly * lpy < 0) {
        continue;
      }

      lx = crosswalk->utm_x2() + dx - (crosswalk->utm_x2() - dx);
      ly = crosswalk->utm_y2() + dy - (crosswalk->utm_y2() - dy);

      lpx = px - (crosswalk->utm_x2() - dx);
      lpy = py - (crosswalk->utm_y2() - dy);

      if (lx * lpx + ly * lpy < 0) {
        continue;
      }

      lx = crosswalk->utm_x1() - dx - (crosswalk->utm_x2() - dx);
      ly = crosswalk->utm_y1() - dy - (crosswalk->utm_y2() - dy);

      if (lx * lpx + ly * lpy < 0) {
        continue;
      }

//      printf("ped inside\n");

      return true;
    }
  }

//  printf("no ped inside\n");
  return false;
}

double CrosswalkManager::min_free_time_ = CROSSWALK_MIN_FREE_TIME;

} // namespace vlr
