/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#include <vector>
#include <set>
#include <assert.h>
#include "kalman_filter.h"

namespace dgc {

KalmanFilter::KalmanFilter() {
  state_dim_ = 0;
  mean_ = NULL;
  covariance_ = NULL;

  kalman_gain_ = NULL;

  observation_mat_ = NULL;
  observation_cov_ = NULL;
  transition_mat_ = NULL;
  transition_cov_ = NULL;
}

KalmanFilter::KalmanFilter(int state_dim) {
  state_dim_ = state_dim;
  mean_ = vector_init(state_dim);
  covariance_ = matrix_init(state_dim, state_dim);

  kalman_gain_ = NULL;

  observation_mat_ = NULL;
  observation_cov_ = NULL;
  transition_mat_ = NULL;
  transition_cov_ = NULL;
}

KalmanFilter::KalmanFilter(const KalmanFilter& kf) {
  state_dim_ = kf.getStateDim();
  mean_ = vector_init(state_dim_);
  vector_copy(mean_, kf.getMean());
  covariance_ = matrix_init(state_dim_, state_dim_);
  kalman_gain_ = NULL;

  matrix_copy(covariance_, kf.getCovariance());

  transition_mat_ = kf.getTransitionMatrix();
  transition_cov_ = kf.getTransitionCovariance();
  observation_mat_ = kf.getObservationMatrix();
  observation_cov_ = kf.getObservationCovariance();
}

KalmanFilter::~KalmanFilter() {
  if(mean_ != NULL) {
    vector_free(mean_);
    mean_ = NULL;
  }
  if(covariance_ != NULL) {
    matrix_free(covariance_);
    covariance_ = NULL;
  }
  if(kalman_gain_ != NULL) {
    matrix_free(kalman_gain_);
    kalman_gain_ = NULL;
  }
}

gsl_vector* KalmanFilter::predictTransition(gsl_vector* state) {
  gsl_vector* next;
  if((int)state->size >= state_dim_) {
    next = vector_init(state->size);
    vector_copy(next, state);
  } else {
    next = vector_init(state_dim_);
    vector_copy(&(gsl_vector_subvector(next, 0, state->size)).vector, state);
  }
  gsl_vector* curr = vector_init(state_dim_);
  if((int)state->size > state_dim_)
    vector_copy(curr, &(gsl_vector_subvector(state, 0, state_dim_)).vector);
  else if((int)state->size < state_dim_) {
    vector_copy(curr, mean_);
    vector_copy(&(gsl_vector_subvector(curr, 0, state->size)).vector, state);
  } else
    vector_copy(curr, state);
  matrix_vector_multiply(&(gsl_vector_subvector(next, 0, state_dim_)).vector, transition_mat_, curr);
  vector_free(curr);
  if((int)state->size < state_dim_) {
    gsl_vector* temp = vector_init(state->size);
    vector_copy(temp, &(gsl_vector_subvector(next, 0, state->size)).vector);
    vector_free(next);
    next = temp;
  }
  return next;
}

void KalmanFilter::predictTransitions(std::vector<gsl_vector*>& orig_pts, std::vector<gsl_vector*>& next_pts) {
  for(std::vector<gsl_vector*>::iterator it = orig_pts.begin(); it != orig_pts.end(); it++) {
    gsl_vector *state = (gsl_vector*)*it;
    gsl_vector *next = predictTransition(state);
    next_pts.push_back(next);
  }
}

gsl_vector* KalmanFilter::predictObservation(gsl_vector* state) {
  int observation_dim = observation_mat_->size1;

  gsl_vector* obs;
  if((int)state->size <= state_dim_) {
    obs = vector_init(observation_dim);
    gsl_vector_set_zero(obs);
  } else {
    obs = vector_init(observation_dim + state->size - state_dim_);
    for(int i = 0; i < (int)state->size - state_dim_; ++i) {
      double val = vector_get(state, i + state_dim_);
      vector_set(obs, i + observation_dim, val);
    }
  }
  gsl_vector* curr = vector_init(state_dim_);
  if((int)state->size > state_dim_)
    vector_copy(curr, &(gsl_vector_subvector(state, 0, state_dim_)).vector);
  else if((int)state->size < state_dim_) {
    vector_copy(curr, mean_);
    vector_copy(&(gsl_vector_subvector(curr, 0, state->size)).vector, state);
  } else
    vector_copy(curr, state);
  matrix_vector_multiply(&(gsl_vector_subvector(obs, 0, observation_dim)).vector, observation_mat_, curr);
  vector_free(curr);
  return obs;
}

void KalmanFilter::predictObservations(std::vector<gsl_vector*>& pts, std::vector<gsl_vector*>& obs) {
  for(std::vector<gsl_vector*>::iterator it = pts.begin(); it != pts.end(); it++) {
    gsl_vector *pt = (gsl_vector*)*it;
    gsl_vector* result = predictObservation(pt);
    obs.push_back(result);
  }
}

/*
 * (for linear KF:
 *  transition_mat_ is A,
 *  transition_cov_ is Q)
 *
 * mean' = A*mean
 * cov'  = A*cov*A^T + Q
 */
void KalmanFilter::transitionUpdate() {
  assert(mean_);
  assert(covariance_);
  assert(transition_mat_);
  assert(transition_cov_);

  gsl_vector *mean_old = mean_;
  mean_ = vector_init(mean_->size);
  matrix_vector_multiply(mean_, transition_mat_, mean_old);
  vector_free(mean_old);

  gsl_matrix *cov_old = covariance_;
  gsl_matrix *temp = matrix_init(covariance_->size1, covariance_->size2);
  covariance_ = matrix_init(covariance_->size1, covariance_->size2);
  matrix_multiply(temp, transition_mat_, 0, cov_old, 0);
  matrix_multiply(covariance_, temp, 0, transition_mat_, 1);
  matrix_add_ip(covariance_, transition_cov_);

  matrix_free(cov_old);
  matrix_free(temp);
}

/*
 * (for linear KF:
 *  observation_mat_ is C,
 *  observation_cov_ is R)
 *
 * cov'  = (cov^-1 + C^T*R^-1*C)^-1
 * mean' = mean + cov'*C^T*R^-1*(z-C*mean)
 */
void KalmanFilter::calculateKalmanGain() {
  if(kalman_gain_ != NULL) {
    matrix_free(kalman_gain_);
    kalman_gain_ = NULL;
  }
  int obs_dim_temp = observation_mat_->size1;
  kalman_gain_ = matrix_init(state_dim_, obs_dim_temp);

  gsl_matrix *temp1 = matrix_init(obs_dim_temp, state_dim_);
  gsl_matrix *temp2 = matrix_init(obs_dim_temp, obs_dim_temp);
  matrix_multiply(temp1, observation_mat_, 0, covariance_, 0);
  matrix_multiply(temp2, temp1, 0, observation_mat_, 1);
  matrix_free(temp1);
  temp1 = matrix_init(state_dim_, obs_dim_temp);
  matrix_add_ip(temp2, observation_cov_);
  matrix_invert_ip(temp2);
  matrix_multiply(temp1, observation_mat_, 1, temp2, 0);
  matrix_multiply(kalman_gain_, covariance_, 0, temp1, 0);
  matrix_free(temp1);
  matrix_free(temp2);
}

void KalmanFilter::observationUpdate(gsl_vector *innovation_vector) {
  int obs_dim_temp = innovation_vector->size;
  assert(obs_dim_temp > 0);
  assert(mean_);
  assert(covariance_);
  assert(observation_mat_);
  assert(observation_cov_);

  calculateKalmanGain();

  gsl_vector *tempv = vector_init(mean_->size);
  matrix_vector_multiply(tempv, kalman_gain_, innovation_vector);
  vector_add_ip(mean_, tempv);
  vector_free(tempv);

  gsl_matrix* temp1 = matrix_init(state_dim_, state_dim_);
  gsl_matrix_set_identity(temp1);
  gsl_matrix* temp2 = matrix_init(state_dim_, state_dim_);
  matrix_multiply(temp2, kalman_gain_, 0, observation_mat_, 0);
  matrix_subtract_ip(temp1, temp2);
  matrix_multiply(temp2, temp1, 0, covariance_, 0);

  matrix_free(temp1);
  matrix_free(covariance_);
  covariance_ = temp2;
}

void KalmanFilter::setStateDim(int state_dim) {
  state_dim_ = state_dim;
  if(mean_ != NULL)
    vector_free(mean_);
  mean_ = vector_init(state_dim);
}

void KalmanFilter::print() const {
  fprintf(stderr, "===KF===\n");
  fprintf(stderr, "(state: %d\n", state_dim_);
  vector_print(mean_, "mean:");
  matrix_print(covariance_, "covariance:");
  fprintf(stderr, "============\n");
}

} // namespace dgc
