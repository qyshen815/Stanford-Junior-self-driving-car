/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef KALMAN_TRACKER_H
#define KALMAN_TRACKER_H

#include <vector>
#include <ext/hash_map>

#include <boost/thread/mutex.hpp>

#include <kalman_filter.h>

#include "perception.h"
#include "tracked_obstacle.h"

typedef struct {
  double default_x_vel;
  double default_y_vel;
  double default_loc_stddev;
  double default_vel_stddev;
  double default_sick_stddev;
  double transition_stddev;
  double velodyne_stddev;
  double sick_stddev;
  double velodyne_max_range;
  double max_dist_correspondence;
  double sick_max_dist_correspondence;
  double confidence_increment_obs;
  double confidence_increment_unobs;
  double confidence_decay_base;
  double confidence_decay_dt_coeff;
  double confidence_max;
  double confidence_min;
  double confidence_initial_max;
  double confidence_initial_min;
  double confidence_track_min;
  double confidence_publish_min;
} kogmo_tracker_kf_parameters_t, *kogmo_tracker_kf_parameters_p;

typedef __gnu_cxx::hash_multimap<int, std::tr1::shared_ptr<dgc::TrackedObstacle> > observation_map_type;

class TrackerKF {

public:
  /** TO USE THIS WITHOUT THINKING **/

  //construct
  TrackerKF();

  //destroy
  virtual ~TrackerKF();

  //so we can interact with param editor
  void setParameters(tracker_kf_settings_p tracker_params);
  static void setParametersToDefault(tracker_kf_settings_p params);

  /** one time step **/
  // (1)
  void transitionUpdate(double timestamp);

  // (2)
  void classificationUpdate(std::tr1::shared_ptr<dgc::Obstacle> observation, double timestamp);
  void observationUpdate(std::vector< std::tr1::shared_ptr<dgc::Obstacle> >& observations, double timestamp);

  // can be called after (1) or (2)
  void getDynamicObstacles(std::vector<std::tr1::shared_ptr<dgc::TrackedObstacle> >& obstacles);
  void getDynamicObstacles(std::vector<std::tr1::shared_ptr<dgc::TrackedObstacle> >&, double confidence_min);

  virtual int boolObstacleOccluded(dgc_sensor_type sensor_type, double x, double y);

  void displayTrackedObstacles(double prediction_time);
protected:
  dgc::KalmanFilter default_filter_;

  void initDefaultFilter();

 public:

  /** STATIC **/

  // converts linear x-velocity/y-velocity to directional
  static void velocityXYtoVR(double x, double y, double* v, double* r);
  static gsl_vector* convertDGCPointToVector(point3d_t& p);
  static point3d_t   convertVectorToDGCPoint(gsl_vector* v);

 protected:
  static const int k_default_state_dim_ = 4;
  static const int k_obs_dim_laser_ = 2;
  static const int k_obs_dim_radar_ = 4;

  double timestamp_;

  std::vector<std::tr1::shared_ptr<dgc::TrackedObstacle> > obstacles_;
  observation_map_type observation_map_;

  int next_obstacle_id_;

  gsl_matrix *transition_mat_, *transition_cov_;
  gsl_matrix *observation_mat_, *observation_cov_;
  gsl_matrix *observation_mat_radar_, *observation_cov_radar_;

  tracker_kf_settings_p parameters_;
  uint my_params_;
};

#endif
