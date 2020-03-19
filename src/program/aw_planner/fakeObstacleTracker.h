/*
 * FakeObstacleTracker.h
 *
 *  Created on: Sep 10, 2009
 *      Author: moritzwerling
 */
#include <vector>
#include <trajectory_points_interface.h>
#include <perception_interface.h>
#include "obstaclePrediction.h"

#ifndef FAKEOBSTACLEPERCEPTION_H_
#define FAKEOBSTACLEPERCEPTION_H_

namespace vlr {

struct CircleDemoCarState {
      double phi;
      double v;
      double r;
      double x;	// center of circle
      double y;	// center of circle
      double width;
      double length;
      double ref_offset;
};

class FakeObstacleTracker {

public:
	FakeObstacleTracker(std::vector<CircleDemoCarState>& init_states, double t_horizon=.1, double deltaT_sampling=3);
	virtual ~FakeObstacleTracker();

	inline void setParams(double checked_horizon, double deltaT_sampling) {
	  t_horizon_ = checked_horizon;
	  deltaT_sampling_ = deltaT_sampling;
	}

	void predictObstacles(double t, double offset_x, double offset_y, std::vector<ObstaclePrediction>& obstacle_predictions, PerceptionObstacles& obstacle_msg);

private:
 std::vector<CircleDemoCarState>& car_states_;
 double t_horizon_;
 double deltaT_sampling_;
};

} // namespace vlr

#endif // FAKEOBSTACLEPERCEPTION_H_
