/*
 * fakeObstaclePerception.cpp
 *
 *  Created on: Sep 10, 2009
 *      Author: moritzwerling
 */

#include <cmath>
#include <vector>
#include <iostream>

#include "obstaclePrediction.h"
#include "fakeObstacleTracker.h"

namespace vlr {

FakeObstacleTracker::FakeObstacleTracker(std::vector<CircleDemoCarState>& init_states,
		double t_horizon, double deltaT_sampling) :
		car_states_(init_states), t_horizon_(t_horizon), deltaT_sampling_(deltaT_sampling) {
}

FakeObstacleTracker::~FakeObstacleTracker() {
}


void FakeObstacleTracker::predictObstacles(double t, double offset_x, double offset_y, std::vector<ObstaclePrediction>& obstacle_predictions,
                                           PerceptionObstacles& obstacle_msg) {
  obstacle_predictions.clear();
  std::vector<CircleDemoCarState>::iterator it;
  uint32_t id;
  double degree90 = 0.5*M_PI;
  static bool init = true;
  static double last_t=0;
  double delta_t=0;
  if(init) {
    init=false;
    delta_t=0;
  }
  else {
    delta_t = t - last_t;
  }

  last_t = t;

  for (it = car_states_.begin(), id=0; it != car_states_.end(); it++, id++ ) {
    ObstaclePrediction obst_pred;
    double& v     = it->v;
    double& r     = it->r;
    double x_r   = it->x - offset_x;
    double y_r   = it->y - offset_y;

    double phi_dot = v / r;

    it->phi += phi_dot*delta_t;

    double& phi_0 = it->phi;

    for (double t_temp = 0; t_temp <= t_horizon_; t_temp += deltaT_sampling_) {
      MovingBox traj_point;
      traj_point.t  = t+t_temp;
      double phi = phi_0 + phi_dot * (t_temp);
      traj_point.x  = r * cos(phi) + x_r;
      traj_point.y  = r * sin(phi) + y_r;
      traj_point.psi  = phi +  degree90;
      traj_point.length = it->length;
      traj_point.width = it->width;
      traj_point.ref_offset = it->ref_offset;

      obst_pred.predicted_traj_.push_back(traj_point);
    }
    obstacle_predictions.push_back(obst_pred);
    obstacle_msg.dynamic_obstacle[id].id = id+1;  // renumber from 1
    obstacle_msg.dynamic_obstacle[id].obstacleType = OBSTACLE_CAR;
    obstacle_msg.dynamic_obstacle[id].confidence = 255;
    obstacle_msg.dynamic_obstacle[id].x = obst_pred.predicted_traj_[0].x;
    obstacle_msg.dynamic_obstacle[id].y = obst_pred.predicted_traj_[0].y;
    obstacle_msg.dynamic_obstacle[id].direction = obst_pred.predicted_traj_[0].psi;

    obstacle_msg.dynamic_obstacle[id].width = obst_pred.predicted_traj_[0].width;
    obstacle_msg.dynamic_obstacle[id].length = obst_pred.predicted_traj_[0].length;
    obstacle_msg.dynamic_obstacle[id].velocity = v;
    obstacle_msg.dynamic_obstacle[id].x_var = 0;
    obstacle_msg.dynamic_obstacle[id].y_var = 0;
    obstacle_msg.dynamic_obstacle[id].xy_cov = 0;
    obstacle_msg.timestamp = t;
  }
}

} // namespace vlr
