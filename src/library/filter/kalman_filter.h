/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <vector>
#include <stdint.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_multifit.h>
#include "gsl.h"

namespace dgc {

class KalmanFilter {

public:
  KalmanFilter();
  KalmanFilter(int state_dim);
  KalmanFilter(const KalmanFilter& kf);
  virtual ~KalmanFilter();

  void transitionUpdate();
  void observationUpdate(gsl_vector* innovation_vector);

  virtual gsl_vector* predictTransition(gsl_vector* orig);
  virtual void predictTransitions(std::vector<gsl_vector*>& orig_pts, std::vector<gsl_vector*>& next_pts);
  virtual gsl_vector* predictObservation(gsl_vector* pt);
  virtual void predictObservations(std::vector<gsl_vector*>& pts, std::vector<gsl_vector*>& obs);

protected:
  void calculateKalmanGain();

public:
  int getStateDim() const { return state_dim_; }
  void setStateDim(int state_dim);

  int getObservationDim() const { return observation_mat_->size1; }

  gsl_vector* getMean() const { return mean_; }
  gsl_matrix* getCovariance() const { return covariance_; }
  gsl_matrix* getKalmanGain() const { return kalman_gain_; }

  gsl_matrix* getTransitionMatrix() const { return transition_mat_; }
  void setTransitionMatrix(gsl_matrix* transition_mat) { transition_mat_ = transition_mat; }

  gsl_matrix* getTransitionCovariance() const { return transition_cov_; }
  void setTransitionCovariance(gsl_matrix* transition_cov) { transition_cov_ = transition_cov; }

  gsl_matrix* getObservationMatrix() const { return observation_mat_; }
  void setObservationMatrix(gsl_matrix* observation_mat) { observation_mat_ = observation_mat; }

  gsl_matrix* getObservationCovariance() const { return observation_cov_; }
  void setObservationCovariance(gsl_matrix* observation_cov) { observation_cov_ = observation_cov; }

  double getYaw() const { return yaw_; }
  void setYaw(double yaw) { yaw_ = yaw; }

  void print() const;

 protected:
  int state_dim_;

  gsl_vector* mean_;
  gsl_matrix* covariance_;

  // temp vars
  gsl_matrix* kalman_gain_;
  gsl_matrix* observation_mat_;
  gsl_matrix* observation_cov_;

  /*
   * TRANSITION MATRIX
   *
   * (for linear KF
   *  transition_mat_ is A)
   *
   *  A = [ 1 0 dt 0 ]
   *      [ 0 1 0 dt ]
   *      [ 0 0 1  0 ]
   *      [ 0 0 0  1 ]
   */
  gsl_matrix* transition_mat_;
  // do we own the pointer?
  uint32_t my_trans_mat_;
  /*
   * TRANSITION COVARIANCE
   *
   *  Q = [ 0 0  0  0  ]
   *      [ 0 0  0  0  ]
   *      [ 0 0 q^2 0  ]
   *      [ 0 0  0 q^2 ]
   */
  gsl_matrix* transition_cov_;
  // do we own the pointer?
  uint32_t my_trans_cov_;

  double yaw_;
};

} // namespace vlr

#endif /*KALMAN_FILTER_H_*/
