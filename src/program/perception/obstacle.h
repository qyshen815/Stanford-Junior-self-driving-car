/*
 *  Created on: Jul 24, 2009
 *      Author: duhadway
 */

#ifndef PERCEPTION_OBSTACLE_H_
#define PERCEPTION_OBSTACLE_H_

#include <roadrunner.h>
#include <grid.h>
#include <vector>
#include <aw_roadNetwork.h>

#include "perception_types.h"
#include "multibooster_support.h"

extern MultiBooster* booster;

namespace dgc {

class Obstacle {
private:
  double x_center_;
  double y_center_;
  double z_center_;

protected:
  std::vector<point3d_t> points_;
  vlr::rndf::Lane* lane_;
  double rndfDist_;
  bool matched_;

  virtual void populatePoints();
public:
  int id;
//  double confidence;
  dgc_pose_t pose;
  double length;
  double width;
  dgc_obstacle_type type;
  dgc_obstacle_type type_this_frame_;
  bool classified_this_frame_;
  double time_;  // TODO: take better care of observation timestamps: use the position in the spin.  (currently this is just the spin's timestamp, and this may be off by 0.1sec.)
//  double timestamp_;            // TODO: need to set this
  dgc_pose_t robot_pose_when_observed_;

  Eigen::VectorXf response_;
  
  Obstacle(int id);
  Obstacle(const Obstacle& o);
  virtual ~Obstacle();
  std::vector<point3d_t>& getPoints();
  void merge(const Obstacle& o);
  bool getCenterOfPoints(double *x, double *y);
  bool getCenterOfPoints(double *x, double *y, double *z);
  vlr::rndf::Lane *getLane() const { return lane_; }
  void setLane(vlr::rndf::Lane *lane) { this->lane_ = lane; }
  double getRndfDist() const {return rndfDist_; }
  void setRndfDist(double dist) {this->rndfDist_ = dist; }

  virtual int  getSize() = 0;
  virtual void markDynamic(dgc_grid_p grid, dgc_perception_map_cells_p obstacles_s, unsigned short counter, double velocity);

  virtual float maxHeight() = 0;

  bool getMatched() const { return matched_; }
  void setMatched(bool matched_) { this->matched_ = matched_; }
};

class GridObstacle : public Obstacle {
private:
  dgc_grid_p grid_;

  
protected:
  virtual void populatePoints();

public:
  std::vector<dgc_perception_map_cell_p> cells_;
  
  GridObstacle(int id, dgc_grid_p grid);
  GridObstacle (const GridObstacle& o);
  virtual ~GridObstacle();
  
  void addCell(dgc_perception_map_cell_p);
  void clear();
  std::vector<dgc_perception_map_cell_p>& getCells();
  virtual int  getSize();
  void merge(const GridObstacle& o);
  virtual float maxHeight();
//  virtual void markDynamic(dgc_grid_p grid, dgc_perception_map_cells_p obstacles_s, unsigned short counter);
};

class LaserObstacle : public Obstacle {
protected:
  virtual void populatePoints();
  double min_z_, max_z_;

public:
  LaserObstacle(int id);
  LaserObstacle (const LaserObstacle& o);
  virtual ~LaserObstacle();

  void addPoint(laser_point_p point);
  void reserve(int count);
  void clear();
  virtual int  getSize();
  void merge(const LaserObstacle& o);

  virtual float maxHeight();

//  virtual void markDynamic(dgc_grid_p grid, dgc_perception_map_cells_p obstacles_s, unsigned short counter);
};

class Observation {

};

class LaserObservation : public Observation {

};

class RadarObservation : public Observation {
public:
  RadarObservation();
  RadarObservation (const RadarObservation& o);
  virtual ~RadarObservation();

  int id;

  double x;
  double y;
  double yaw;
  double velocity;
  double x_vel;
  double y_vel;
};

typedef std::vector<Obstacle*> TObstacleVec;

class Obstacles {
public:
  Obstacles() {};
  ~Obstacles() {};
  dgc_pose_t robot_pose;  // pose of the robot to calculate global obstacle pose from local map
  double timestamp;
  std::vector<Obstacle*> obstacles;
};

}

#endif /* PERCEPTION_OBSTACLE_H_ */
