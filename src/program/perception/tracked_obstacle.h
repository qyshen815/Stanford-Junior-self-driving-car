/*
 *  Created on: Jul 24, 2009
 *      Author: duhadway
 */

#ifndef TRACKED_OBSTACLE_H_
#define TRACKED_OBSTACLE_H_

#include <roadrunner.h>
#include <grid.h>
#include <queue>
#include <vector>
#include <tr1/memory>

#include "obstacle.h"
#include <linear_kalman_filter/linear_kalman_filter.h>

namespace dgc {

class TrackedObstacle : public Obstacle {
private:
  int pedestrian_label_count;

//  int observed_, occluded_;
  int num_observations_;

//  double max_speed_;

  double timestamp_first_;       // time of earliest observation
  double timestamp_prediction_;  // time of most recent prediction

  double x_velocity_;
  double y_velocity_;
  Eigen::VectorXf prior_log_odds_; //Prior log odds for each class. The indices here correspond to those in booster->class_map_.  
  Eigen::VectorXf log_odds_; //Log odds for each class, updated via incorporateBoostingResponse.
protected:
  virtual void populatePoints();

public:
  std::tr1::shared_ptr<LinearKalmanFilter> filter;
  std::tr1::shared_ptr<Obstacle> lastObservation_;
  TrackedObstacle(int id, std::tr1::shared_ptr<Obstacle> observation, double timestamp);
  TrackedObstacle(const TrackedObstacle& o);
  virtual ~TrackedObstacle();

  void update(std::tr1::shared_ptr<Obstacle>, double timestamp);
  void update(double timestamp);
//  double x_var;
//  double y_var;
//  double xy_cov;
  int getNumObservations() { return num_observations_; }
  double getXVel() const;
  double getYVel() const;
//  double getXVar() const;
//  double getYVar() const;
//  double getXYCov() const;
  double getVelocity() const;
//  double getMaxSpeed() const { return max_speed_; }
//  int    getObserved() const { return observed_; }
//  int    getOccluded() const { return occluded_; }
  Eigen::VectorXf getLogOdds() { return log_odds_; }
  std::tr1::shared_ptr<Obstacle> getLastObservation() { return lastObservation_; }

  virtual int  getSize() { return 0; }
  double timestamp_observation_; // time of most recent observation

  virtual void markDynamic(dgc_grid_p grid, dgc_perception_map_cells_p obstacles_s, unsigned short counter);
  void incorporateClassification(int type, const Eigen::VectorXf& response);
  void estimateModel();

  float maxHeight();
};

//bool compareTrackedObstaclesConfidence(const std::tr1::shared_ptr<TrackedObstacle> o1, const std::tr1::shared_ptr<TrackedObstacle> o2);

}

#endif /* PERCEPTION_OBSTACLE_H_ */
