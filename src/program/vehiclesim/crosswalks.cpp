#include "crosswalks.h"

#include <param_interface.h>
#include <cstdio>
#include <iostream>

static const double SIM_PEDESTRIAN_WIDTH = .5;
static const double SIM_PEDESTRIAN_LENGTH = .3;

using namespace std;
using namespace dgc;
using namespace vlr::rndf;

namespace vlr {
void fill_dirk_points(PerceptionObstaclePoint *point, int n, double x1, double y1, double x2, double y2, int id);

CrosswalkSimulator::CrosswalkSimulator(vlr::rndf::RoadNetwork& rn, IpcInterface*& ipc) :
  ipc_(ipc), lastTime_(0.) {

  readParameters();
  srand(1);

  SimCrosswalkPedestrian temp;
  temp.active = false;
  temp.current_x = 0.;

  double time = dgc_get_time();
  double r;
  double dx, dy;

  const TCrosswalkMap& crosswalks = rn.crosswalks();
  TCrosswalkMap::const_iterator cwit = crosswalks.begin(), cwit_end = crosswalks.end();
  for (; cwit != cwit_end; cwit++) {
    std::string name = (*cwit).first;
    vlr::rndf::Crosswalk* cw = (*cwit).second;

    temp.max_x = fabs(hypot(cw->utm_y2() - cw->utm_y1(), cw->utm_x2() - cw->utm_x1())) / 2. + 1;
    temp.theta = atan2(cw->utm_y2() - cw->utm_y1(), cw->utm_x2() - cw->utm_x1());
    temp.sintheta = sin(temp.theta);
    temp.costheta = cos(temp.theta);

    r = -cw->width() / 2.0 + .5 * SIM_PEDESTRIAN_WIDTH;
    int max_k = cw->width() / SIM_PEDESTRIAN_WIDTH - 1;
    if (max_k < 1) max_k = 1;
    for (int k = 0; k < max_k; ++k) {
      dx = r * cos(temp.theta + M_PI_2);
      dy = r * sin(temp.theta + M_PI_2);

      temp.utm_x = (cw->utm_x2() + cw->utm_x1()) / 2. + dx;
      temp.utm_y = (cw->utm_y2() + cw->utm_y1()) / 2. + dy;
      temp.next_time = time + ((double) (rand() % 10000)) / 10000. * params_.max_wait_time;
      sprintf(temp.id, "%s.%d", cw->name().c_str(), k + 1);
      pedestrians_.push_back(temp);

      r += cw->width() / (double) max_k;
    }
  }

  obstacles_.point = new PerceptionObstaclePoint[pedestrians_.size() * 16];
  obstacles_.num_points = 0;

  obstacles_.dynamic_obstacle = new PerceptionDynamicObstacle[pedestrians_.size()];
  obstacles_.num_dynamic_obstacles = 0;
}

CrosswalkSimulator::~CrosswalkSimulator() {
  delete[] obstacles_.point;
}

void CrosswalkSimulator::update(double time) {
  if (!params_.generate_pedestrians) {
    return;
  }

  static const int pedestrian_obstacle_id = 1024;
  int& num_obstacles = obstacles_.num_dynamic_obstacles;
  int& mark = obstacles_.num_points;

  mark = 0;
  num_obstacles = 0;

  int id = 1; //moving
  //int id = 0; //static
  std::vector<SimCrosswalkPedestrian>::iterator it;
  for (it = pedestrians_.begin(); it != pedestrians_.end(); ++it) {
    if (it->active) {

      if (it->current_x > it->max_x || it->current_x < -it->max_x) {
        it->active = false;
        it->next_time = time + ((double) (rand() % 10000)) / 10000. * params_.max_wait_time;
        std::cout << "Pedestrian " << it->id << " has finished crossing." << std::endl;
      }
      else {
        it->current_x += it->speed * (time - lastTime_);

        double& ctheta = it->costheta;
        double& stheta = it->sintheta;
        double utm_x = it->utm_x + it->current_x * ctheta;
        double utm_y = it->utm_y + it->current_x * stheta;

        obstacles_.dynamic_obstacle[num_obstacles].confidence = 255;
        obstacles_.dynamic_obstacle[num_obstacles].id = num_obstacles + pedestrian_obstacle_id;
        obstacles_.dynamic_obstacle[num_obstacles].x = utm_x;
        obstacles_.dynamic_obstacle[num_obstacles].y = utm_y;
        obstacles_.dynamic_obstacle[num_obstacles].direction = it->theta;
        obstacles_.dynamic_obstacle[num_obstacles].velocity = it->speed;
        obstacles_.dynamic_obstacle[num_obstacles].length = SIM_PEDESTRIAN_LENGTH;
        obstacles_.dynamic_obstacle[num_obstacles].width = SIM_PEDESTRIAN_WIDTH;
        obstacles_.dynamic_obstacle[num_obstacles].obstacleType = OBSTACLE_PEDESTRIAN;
        obstacles_.dynamic_obstacle[num_obstacles].xy_cov = 0.0;
        num_obstacles++;

        fill_dirk_points(obstacles_.point + mark, 4, utm_x + 0.5 * (SIM_PEDESTRIAN_LENGTH * ctheta - SIM_PEDESTRIAN_WIDTH * stheta), utm_y + 0.5
            * (SIM_PEDESTRIAN_LENGTH * stheta + SIM_PEDESTRIAN_WIDTH * ctheta), utm_x + 0.5 * (SIM_PEDESTRIAN_LENGTH * ctheta + SIM_PEDESTRIAN_WIDTH * stheta),
            utm_y + 0.5 * (SIM_PEDESTRIAN_LENGTH * stheta - SIM_PEDESTRIAN_WIDTH * ctheta), id);
        fill_dirk_points(obstacles_.point + mark + 4, 4, utm_x + 0.5 * (SIM_PEDESTRIAN_LENGTH * ctheta + SIM_PEDESTRIAN_WIDTH * stheta), utm_y + 0.5
            * (SIM_PEDESTRIAN_LENGTH * stheta - SIM_PEDESTRIAN_WIDTH * ctheta),
            utm_x + 0.5 * (-SIM_PEDESTRIAN_LENGTH * ctheta + SIM_PEDESTRIAN_WIDTH * stheta), utm_y + 0.5 * (-SIM_PEDESTRIAN_LENGTH * stheta
                - SIM_PEDESTRIAN_WIDTH * ctheta), id);
        fill_dirk_points(obstacles_.point + mark + 8, 4, utm_x + 0.5 * (-SIM_PEDESTRIAN_LENGTH * ctheta + SIM_PEDESTRIAN_WIDTH * stheta), utm_y + 0.5
            * (-SIM_PEDESTRIAN_LENGTH * stheta - SIM_PEDESTRIAN_WIDTH * ctheta), utm_x + 0.5
            * (-SIM_PEDESTRIAN_LENGTH * ctheta - SIM_PEDESTRIAN_WIDTH * stheta), utm_y + 0.5
            * (-SIM_PEDESTRIAN_LENGTH * stheta + SIM_PEDESTRIAN_WIDTH * ctheta), id);
        fill_dirk_points(obstacles_.point + mark + 12, 10, utm_x + 0.5 * (-SIM_PEDESTRIAN_LENGTH * ctheta - SIM_PEDESTRIAN_WIDTH * stheta), utm_y + 0.5
            * (-SIM_PEDESTRIAN_LENGTH * stheta + SIM_PEDESTRIAN_WIDTH * ctheta),
            utm_x + 0.5 * (SIM_PEDESTRIAN_LENGTH * ctheta - SIM_PEDESTRIAN_WIDTH * stheta), utm_y + 0.5 * (SIM_PEDESTRIAN_LENGTH * stheta
                + SIM_PEDESTRIAN_WIDTH * ctheta), id);
        mark += 16;
      }
    }
    if (!it->active) if (time > it->next_time) {
      std::cout << "Pedestrian " << it->id << " starts crossing now." << std::endl;
      it->active = true;
      if (rand() % 2) {
        it->current_x = -it->max_x;
        it->speed = ((double) (rand() % 1000)) / 1000. * (params_.max_speed - params_.min_speed) + params_.min_speed;
      }
      else {
        it->current_x = it->max_x;
        it->speed = -((double) (rand() % 1000)) / 1000. * (params_.max_speed - params_.min_speed) - params_.min_speed;
      }
    }
  }

  lastTime_ = time;
}

void CrosswalkSimulator::readParameters() {

  ParamInterface pint(ipc_);

  Param params[] = { { "sim", "crosswalk_generate_pedestrians", DGC_PARAM_ONOFF, &params_.generate_pedestrians, 1, NULL }, { "sim", "crosswalk_max_wait_time",
      DGC_PARAM_DOUBLE, &params_.max_wait_time, 1, NULL }, { "sim", "crosswalk_min_pedestrian_speed", DGC_PARAM_DOUBLE, &params_.min_speed, 1, NULL }, { "sim",
      "crosswalk_max_pedestrian_speed", DGC_PARAM_DOUBLE, &params_.max_speed, 1, NULL }, };

  char temp = 0;
  char* temp2 = &temp;
  char **argv = &temp2;
  pint.InstallParams(1, argv, params, sizeof(params) / sizeof(params[0]));
}

} // namespace vlr
