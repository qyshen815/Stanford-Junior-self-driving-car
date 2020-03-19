#include "kalman_multitracker.h"
#include "box.h"

using namespace dgc;
using namespace std;
using namespace std::tr1;
using namespace Eigen;

/* Removed to simply use Obstacle inputs, this may still be wanted to clean things up later
   MultiTrackerMeasurement::MultiTrackerMeasurement() :
   centroid_(VectorXd::Zero(2)),
   timestamp_(-1)
   {
   }*/

/*MultiTrackerTrack::MultiTrackerTrack( LinearKalmanFilter kf, shared_ptr<TrackedObstacle> obs) :
  filter(kf),
  obstacle(obs)
{
}*/

KalmanMultiTracker::KalmanMultiTracker(
    double correspondence_thresh, double pruning_thresh,
    double measurement_variance, double position_variance,
    double velocity_variance, double initial_position_variance,
    double initial_velocity_variance) :
  next_id_(0),
  correspondence_thresh_(correspondence_thresh),
  pruning_thresh_(pruning_thresh),
  current_timestamp_(0),
  prev_timestamp_(0)
{
  measurement_matrix_ = MatrixXd::Identity(2, 4);
  transition_matrix_ = MatrixXd::Identity(4, 4);
  measurement_covariance_ = MatrixXd::Identity(2, 2) * measurement_variance;

  initial_sigma_ = MatrixXd::Zero(4, 4);
  initial_sigma_(0, 0) = initial_position_variance;
  initial_sigma_(1, 1) = initial_position_variance;
  initial_sigma_(2, 2) = initial_velocity_variance;
  initial_sigma_(3, 3) = initial_velocity_variance;

  transition_covariance_ = MatrixXd::Zero(4, 4);
  transition_covariance_(0, 0) = position_variance;
  transition_covariance_(1, 1) = position_variance;
  transition_covariance_(2, 2) = velocity_variance;
  transition_covariance_(3, 3) = velocity_variance;
}

void KalmanMultiTracker::prune(double timestamp_current) {
  // -- Prune filters with high position uncertainty.
  list<shared_ptr<TrackedObstacle> >::iterator it;
  int i = 0;
  for(it = tracks_.begin(); it != tracks_.end(); ++it) {
    // cout << "Track " << (*it)->filter->id_ << endl;
    // cout << (*it)->filter->sigma_ << endl << endl;

    // x, y variances are equal, so just prune on x variance.
    if((*it)->filter->sigma_(0, 0) >= pruning_thresh_) {
      it = tracks_.erase(it);
      i++;
      it--;
      continue;
    }

      // If we haven't observed the track for pruning_thresh_ seconds
    // delete it.
      //if((*it)->timestamp_observation_ + pruning_thresh_ < timestamp_current) {
    //   it = tracks_.erase(it);
    //   i++;
    //   it--;
    //   continue;
    // }
    
  }
  printf("Pruned %d tracks\n", i);
}

void KalmanMultiTracker::step(const vector< shared_ptr<Obstacle> >& measurements, double timestamp) {
  update(measurements, timestamp);
  prune(timestamp);
}

void KalmanMultiTracker::update(const vector< shared_ptr<Obstacle> >& measurements, double timestamp) {
  prev_timestamp_ = current_timestamp_;
  current_timestamp_ = timestamp;

  vector<bool> matched_measurements(measurements.size(), false);
  vector<bool> matched_filters(tracks_.size(), false);
  //printf("Center: %f %f\n", measurements[0]->pose.x, measurements[0]->pose.y);
  if(!tracks_.empty() && !measurements.empty()) { 

    // -- For all filter / measurement pairs, compute the score.
    MatrixXd scores((int)tracks_.size(), (int)measurements.size());
    list<shared_ptr<TrackedObstacle> >::iterator it = tracks_.begin();
    double starttime = dgc_get_time();
    for(int i = 0; i < (int)tracks_.size(); ++i, ++it) {
      shared_ptr<TrackedObstacle> track = *it;
      for(int j = 0; j < (int)measurements.size(); ++j) {
        const shared_ptr<Obstacle>& measurement = measurements[j];
        double delta_t = measurement->time_ - track->filter->timestamp_;
        // Compute the prediction for the timestamp of this measurement.
        transition_matrix_(0, 2) = delta_t;
        transition_matrix_(1, 3) = delta_t;

        VectorXd mu_bar(2);
        mu_bar(0) = delta_t * track->filter->mu_(2) + track->filter->mu_(0);
        mu_bar(1) = delta_t * track->filter->mu_(3) + track->filter->mu_(1);
        // Compute the score for this prediction / measurement pair.
        // @TODO Centroid calculation here should be cleaned up
        Eigen::VectorXd centroid = VectorXd::Zero(2);
        double x, y;
        measurement->getCenterOfPoints(&x, &y);
        centroid(0) = x;
        centroid(1) = y;
        VectorXd innovation = mu_bar - centroid;
        if( innovation.norm() > 3.0 ) {
          scores(i, j) = 0;
          continue;
        }
        MatrixXd sigma_bar = transition_matrix_ * track->filter->sigma_ * transition_matrix_.transpose() + track->filter->transition_covariance_;
        MatrixXd sigma_bar_inv_pos = sigma_bar.block(0, 0, 2, 2).inverse(); // The covariance matrix for the marginal of a Gaussian is just that block of the full covariance matrix.
        scores(i, j) = exp(-0.5 * (innovation.transpose() * sigma_bar_inv_pos * innovation)[0]) / sqrt(( 2 * M_PI * sigma_bar.block(0, 0, 2, 2)).determinant());
        //printf("score: %f\n", exp(-0.5 * (innovation.transpose() * sigma_bar_inv_pos * innovation)[0]));
      }
    }

    printf("Scoring took %f\n", dgc_get_time() - starttime);
    // -- Assigning correspondence in order of decreasing score, and run prediction and update for filters with matched measurements.
    int number = 0;
    while(true) {
      int f = 0;
      int m = 0;
      double max_score = scores.maxCoeff(&f, &m);
      //printf("max_score: %f\n", max_score);
      if(max_score < correspondence_thresh_)
        break;
      number++;
      // Find the filter with the corresponding measurement.
      // This has bad asymptotic complexity but probably small constant.
      it = tracks_.begin();
      advance(it, f);

      // Run the prediction and update step for this delta_time.
      double delta_time = measurements[m]->time_ - (*it)->filter->timestamp_;
      transition_matrix_(0, 2) = delta_time;
      transition_matrix_(1, 3) = delta_time;
      Eigen::VectorXd centroid = VectorXd::Zero(2);
      double x, y;
      measurements[m]->getCenterOfPoints(&x, &y);
      centroid(0) = x;
      centroid(1) = y;
      /*printf("-------------------------\n");
      printf("-------------------------\n");
      printf("-------------------------\n");
      printf("DT: %f\n", delta_time);
      printf("old_pos: %f %f\n", (*it)->pose.x, (*it)->pose.y);
      printf("obs_timestamp: %f\n", measurements[m]->time_);
      printf("filter_timestamp: %f\n", (*it)->filter->timestamp_);
      cout << "-------\n" << (*it)->filter->sigma_ << endl << "--------\n";*/
      (*it)->filter->predict(transition_matrix_, measurements[m]->time_);
      (*it)->filter->update(centroid, measurements[m]->time_);
      (*it)->update(measurements[m], measurements[m]->time_);
      // Clear the scores from the matched filter and measurement.
      scores.row(f) = VectorXd::Zero(scores.cols());
      scores.col(m) = VectorXd::Zero(scores.rows());
      if((*it)->getVelocity() > 30000) {
        printf("---------------------------------------------\n");
        printf("Position uncertainty: %f\n", (*it)->filter->getPositionUncertainty());
        cout << "-------\n" << (*it)->filter->sigma_ << endl << "--------\n";
        cout << "-------\n" << (*it)->filter->mu_ << endl << "--------\n";
        printf("num_obs: %d\n", (*it)->getNumObservations());
        printf("pos: %f %f\n", (*it)->pose.x, (*it)->pose.y);
        printf("obs pos: %f %f\n", x, y);
      } 
      matched_filters[f] = true;
      matched_measurements[m] = true;
    }
    cout << "Matched " << number << " of  " << tracks_.size() << endl;
  }

  // -- For any unmatched measurements predict forward in time
  list<shared_ptr<TrackedObstacle> >::iterator it = tracks_.begin();
  for(size_t i = 0; i < matched_filters.size(); ++i, ++it) {
    if(matched_filters[i])
      continue;
    (*it)->update(current_timestamp_);
    transition_matrix_(0, 2) = current_timestamp_ - prev_timestamp_;
    transition_matrix_(1, 3) = current_timestamp_ - prev_timestamp_;
    (*it)->filter->predict(transition_matrix_, (*it)->filter->timestamp_ + current_timestamp_ - prev_timestamp_);
  }    

  // -- For all remaining measurements, spawn new filters.
  for(size_t i = 0; i < matched_measurements.size(); ++i) {
    if(matched_measurements[i])
      continue;

    align_bounding_box(&*measurements[i], 0.1, 0.1);
    if(measurements[i]->width < 0.5 && measurements[i]->length < 0.5)
      continue;
    // Max length of a school bus is ~40 feet, so cut off anything greater than 42 feet.
    if(measurements[i]->width > 12.8 || measurements[i]->length > 12.8)
      continue;
    VectorXd initial_state = VectorXd::Zero(4);
    Eigen::VectorXd centroid = VectorXd::Zero(2);
    double x,y;
    measurements[i]->getCenterOfPoints(&x, &y);
    centroid(0) = x;
    centroid(1) = y;
    initial_state.segment(0, 2) = centroid;
    shared_ptr<LinearKalmanFilter> new_kf(new LinearKalmanFilter(next_id_, 
          measurements[i]->time_, initial_state, initial_sigma_, 
          measurement_matrix_, transition_covariance_, 
          measurement_covariance_));
    shared_ptr<TrackedObstacle> new_obs(new TrackedObstacle(next_id_, measurements[i], current_timestamp_));
    new_obs->filter = new_kf;
    new_obs->update(measurements[i], measurements[i]->time_);
    tracks_.push_front(new_obs);
    ++next_id_;
  }
}
