/*
 * collisionCheck.h
 *
 *  Created on: Sep 11, 2009
 *      Author: moritzwerling
 */
#ifndef COLLISIONCHECK_H_
#define COLLISIONCHECK_H_

#include <stdint.h>
#include <vector>
#include "obstaclePrediction.h"
#include "polyTraj2D.h"

namespace vlr {
typedef enum {
  OSM_FREE=0, OSM_MAYBE_BLOCKED=128, OSM_BLOCKED=255
} ObstacleMapOccupancyState_t;

typedef enum {
  TRJ_FREE, TRJ_BLOCKED, TRJ_MAYBE_BLOCKED
} TrjOccupancyState_t;

bool checkCollisionOfTrajectories(const double& width_safety, const double& width_no_safety,
    const double& length_safety, const double& length_no_safety, const double& pull_away_time, const double& offset,
    const std::vector<TrajectoryPoint2D>& trajectory, const std::vector<ObstaclePrediction>& obstacle_predictions,
    double& collision_time);
// width/length_safety		.. includes desired distance to other obstacles
// width/length_no_safety 	.. describes car with a fairly small safety margin in order be robust to noise of obstacle position
// pull_away_time			.. time to pull away from a car that got too close

bool checkaCollisionCircles(const Vehicle::circle& c1, const Vehicle::circle& c2);
// returns true if circles coincide

bool checkCollisionVehicles(const double& x1, const double& y1, const double& psi1, const ObstaclePrediction& par1,
    const double& x2, const double& y2, const double& psi2, const ObstaclePrediction& par2);
// returns true if circular vehicle approximations coincide

} // namespace vlr

#endif
