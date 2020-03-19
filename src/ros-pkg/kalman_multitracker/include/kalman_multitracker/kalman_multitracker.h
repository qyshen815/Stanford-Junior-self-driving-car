#ifndef KALMAN_MULTITRACKER_H
#define KALMAN_MULTITRACKER_H

#include <vector>
#include <list>
#include <float.h>
#include <linear_kalman_filter/linear_kalman_filter.h>
#include <matplotlib_interface/matplotlib_interface.h>

//! Expand to include cluster descriptors?
class MultiTrackerMeasurement {
 public:
  Eigen::VectorXd centroid_;
  double timestamp_;

  MultiTrackerMeasurement();
};

class KalmanMultiTracker {
 private:
  //! The the next id_ value to assign to a LinearKalmanFilter.
  int next_id_;
  //! Minimum value of the filter's prediction Gaussian (unnormalized) to allow when making correspondences.
  double correspondence_thresh_;
  //! Maximum allowable position uncertainty before deleting a filter.
  double pruning_thresh_;
  //! Measurement matrix common to all tracks.
  Eigen::MatrixXd measurement_matrix_;
  //! Transition matrix common to all tracks.  Modified every update step based on delta_time.
  Eigen::MatrixXd transition_matrix_;
  //! Initial state covariance common to all tracks.
  Eigen::MatrixXd initial_sigma_;
  //! Transition covariance common to all tracks.
  Eigen::MatrixXd transition_covariance_;
  //! Measurement covariance common to all tracks.
  Eigen::MatrixXd measurement_covariance_;
  double current_timestamp_;
  double prev_timestamp_;

  //! Prune out filters that have high uncertainty.
  void prune();
  void update(const std::vector<MultiTrackerMeasurement>& measurements, double timestamp);

 public:
  std::list<LinearKalmanFilter> filters_;

  //! @param initial_position_variance is the variance for both x and y in the initial sigma for new filters.
  KalmanMultiTracker(double correspondence_thresh, double pruning_thresh,
		     double measurement_variance, double position_variance,
		     double velocity_variance, double initial_position_variance,
		     double initial_velocity_variance);

  //! Computes scores, assigns correspondences, runs prediction and update steps for all filters, spawns new filters, prunes bad ones.
  //! @param timestamp is the time at which the update was made; this can be offset arbitrarily from the measurement timestamps.
  void step(const std::vector<MultiTrackerMeasurement>& measurements, double timestamp);
};

#endif //KALMAN_MULTITRACKER_H

