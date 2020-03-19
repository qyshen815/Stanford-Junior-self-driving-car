#include <math.h>
#include <assert.h>
#include <vector>
#include <set>
#include <algorithm>

#include <global.h>

#include "kalman_tracker.h"

#define CONSIDER_LOST                   0.5

#ifdef DEBUG
#undef DEBUG
#endif

#define DEBUG                           0
#define EPSILON                         0.000001
#define OBSTACLE_SPEED_THRESHOLD				1.0

using namespace dgc;
using namespace std;
using std::tr1::shared_ptr;

inline int within_epsilon(double e) {
  return fabs(e) < EPSILON;
}

inline int within_epsilon(double x, double y) {
  return fabs(x - y) < EPSILON;
}

// constructor
TrackerKF::TrackerKF() :
  default_filter_(k_default_state_dim_)
{
  timestamp_ = 0;
  next_obstacle_id_ = 0;

  transition_mat_ = matrix_init(k_default_state_dim_, k_default_state_dim_);
  transition_cov_ = matrix_init(k_default_state_dim_, k_default_state_dim_);
  observation_mat_ = matrix_init(k_obs_dim_laser_, k_default_state_dim_);
  observation_cov_ = matrix_init(k_obs_dim_laser_, k_obs_dim_laser_);

  observation_mat_radar_ = matrix_init(k_obs_dim_radar_, k_default_state_dim_);
  observation_cov_radar_ = matrix_init(k_obs_dim_radar_, k_obs_dim_radar_);

  parameters_ = new tracker_kf_settings_t;
  setParametersToDefault(parameters_);
  my_params_ = 1;

  initDefaultFilter();
}

void TrackerKF::initDefaultFilter() {
  gsl_matrix_set_identity(transition_mat_);
  gsl_matrix_set_identity(transition_cov_);
  gsl_matrix_set_identity(observation_mat_);
  gsl_matrix_set_identity(observation_cov_);
  gsl_matrix_set_identity(observation_mat_radar_);
  gsl_matrix_set_identity(observation_cov_radar_);

  gsl_vector* mean = default_filter_.getMean();
  gsl_vector_set_zero(mean);

  // set initial covariance values
  gsl_matrix* covariance = default_filter_.getCovariance();
  double l_cov = parameters_->default_loc_stddev * parameters_->default_loc_stddev;
  double v_cov = parameters_->default_vel_stddev * parameters_->default_vel_stddev;
  matrix_set(covariance, 0, 0, l_cov);
  matrix_set(covariance, 1, 1, l_cov);
  matrix_set(covariance, 2, 2, v_cov);
  matrix_set(covariance, 3, 3, v_cov);

  // set transition model to default
  default_filter_.setTransitionMatrix(transition_mat_);
  default_filter_.setTransitionCovariance(transition_cov_);

  // set observation model to default to laser
  default_filter_.setObservationMatrix(observation_mat_);
  default_filter_.setObservationCovariance(observation_cov_);
}

// destructor
TrackerKF::~TrackerKF() {
  if (my_params_ && parameters_ != NULL)
    delete parameters_;
}

void TrackerKF::setParameters(tracker_kf_settings_p tracker_params) {
  if (tracker_params != NULL) {
    if (my_params_ && parameters_ != NULL)
      delete parameters_;
  }
  my_params_ = 0;
  parameters_ = tracker_params;

  TrackedObstacle::setParams(parameters_);
  initDefaultFilter();
}

void TrackerKF::setParametersToDefault(tracker_kf_settings_p params) {
  if (params == NULL)
    return;
  params->default_loc_stddev = 1.0;
  params->default_vel_stddev = 2.5;
  params->transition_stddev = 1.3;
  params->velodyne_stddev = 2.0;
  params->radar_stddev = 1.0;
  params->velodyne_max_range = 120;
  params->radar_max_range = 200;
  params->max_dist_correspondence = 5.0;
  params->confidence_increment_obs = 1.0;
  params->confidence_increment_unobs = -1.0;
  params->confidence_decay = -1.5;
  params->confidence_max = 20.0;
  params->confidence_min = -20.0;
  params->confidence_initial_max = 1.0;
  params->confidence_initial_min = -1.0;
  params->confidence_track_min = -10.0;
  params->confidence_publish_min = 2.0;

  TrackedObstacle::setParams(params);
}

// (1)
void TrackerKF::transitionUpdate(double timestamp) {
  double dt = timestamp - timestamp_;
  timestamp_ = timestamp;


  if (DEBUG >= 1)
    fprintf(stderr, "TIMESTAMP DIFF %f @ %f", dt, timestamp);

  if (within_epsilon(dt)) {
    return;
  }

  gsl_matrix_set_zero(transition_cov_);
  // TODO: update parameter, transition_stddev, to reflect new dt term
  //       should account for possible accelerations we expect to see
  //       if we expect to accelerations of 2 m/s/s to be common,
  //       you could set transition_std_dev to 1.6 or so

  // TODO: process noise (accounting for acceleration) should account for the
  //       fact that cars don't accelerate laterally

  // TODO: we could add acceleration to the state
  for(int i = 0; i < (int)transition_mat_->size2/2; ++i)
    matrix_set(transition_mat_, i, i + transition_mat_->size2/2, dt);

  for (int i = k_default_state_dim_/2; i < k_default_state_dim_; ++i)
    matrix_set(transition_cov_, i, i, dt * parameters_->transition_stddev * parameters_->transition_stddev );

  for (vector<shared_ptr<TrackedObstacle> >::iterator it = obstacles_.begin(); it != obstacles_.end(); it++) {
    (*it)->transitionUpdate(timestamp);
  }
}

int TrackerKF::boolObstacleOccluded(dgc_sensor_type sensor_type, double x, double y) {
  int occluded = 0;

  ApplanixPose* robot_pose = applanix_current_pose();
  dgc_transform_t smooth_to_robot;
  dgc_transform_identity(smooth_to_robot);
  dgc_transform_translate( smooth_to_robot, -robot_pose->smooth_x, -robot_pose->smooth_y, -robot_pose->smooth_z );
  dgc_transform_rotate_z( smooth_to_robot, -robot_pose->yaw );
  dgc_transform_rotate_y( smooth_to_robot, -robot_pose->pitch );
  dgc_transform_rotate_x( smooth_to_robot, -robot_pose->roll );

  dgc_transform_t smooth_to_sensor;
  double z = 0.0;

  switch (sensor_type) {
  case SENSOR_RADAR1:
    dgc_transform_inverse(radar_offset[0], smooth_to_sensor);
    break;
  case SENSOR_RADAR2:
    dgc_transform_inverse(radar_offset[1], smooth_to_sensor);
    break;
  case SENSOR_RADAR3:
    dgc_transform_inverse(radar_offset[2], smooth_to_sensor);
    break;
  case SENSOR_RADAR4:
    dgc_transform_inverse(radar_offset[3], smooth_to_sensor);
    break;
  case SENSOR_RADAR5:
    dgc_transform_inverse(radar_offset[4], smooth_to_sensor);
    break;
  case SENSOR_RADAR6:
    dgc_transform_inverse(radar_offset[5], smooth_to_sensor);
    break;
  case SENSOR_VELODYNE:
    dgc_transform_inverse(velodyne_offset, smooth_to_sensor);
    break;
  }

  dgc_transform_left_multiply(smooth_to_sensor, smooth_to_robot);
  dgc_transform_point(&x, &y, &z, smooth_to_sensor);

  double range = hypot(x, y);
  double angle = atan2(y, x);

  switch (sensor_type) {
  case SENSOR_RADAR1:
  case SENSOR_RADAR2:
  case SENSOR_RADAR4:
  case SENSOR_RADAR5:
    occluded = ((range > parameters_->radar_max_range) || (fabs(angle) > dgc_d2r(7.5)));
    break;

  case SENSOR_RADAR3:
  case SENSOR_RADAR6:
    occluded = ((range > parameters_->radar_max_range) || (fabs(angle) > dgc_d2r(15)));
    break;

  case (SENSOR_VELODYNE):
    // TODO: could add blind spot right behind vehicle, though this is picked up by LDLRS in grid observations
    occluded = (range > parameters_->velodyne_max_range);

    break;
  }

  return occluded;
}

// a second observation update using observations filtered by the classifier
void TrackerKF::classificationUpdate(shared_ptr<Obstacle> observation, double timestamp) {
  bool matched = false;
  static float threshold = 0.0; // TODO: learn better threshold

  // check to see if this observation was already used
  __gnu_cxx::pair<observation_map_type::iterator, observation_map_type::iterator> range = observation_map_.equal_range(observation->id);
  for (observation_map_type::iterator it = range.first; it != range.second; it++) {
    shared_ptr<TrackedObstacle> track = it->second;
//    printf("updating track with %s classification, %f\n", obstacle_type2str(observation->type), track->getConfidence());
    track->incorporateClassification(observation->type, observation->response_);
    track->classified_this_frame_ = true;
    matched = true;
  }

  if (matched)
    return;

  if (observation->type != OBSTACLE_UNKNOWN) {
    // match this observation to current tracks
    for (vector< shared_ptr<TrackedObstacle> >::iterator to = obstacles_.begin(); to != obstacles_.end(); to++) {
      shared_ptr<TrackedObstacle> track = (*to);

      if (track->getObserved()) // skip tracks that have already been observed
        continue;

      float score = track->scoreObservation(observation.get());
      if (score > threshold) {
        shared_ptr<TrackedObstacle> new_track(new TrackedObstacle(*track.get(), observation, timestamp));
        new_track->estimateModel();
        *to = new_track;
        matched = true;
        break;
      }
    }

    if (matched)
      return;

    // make new track for an observation with a good classification
    shared_ptr<TrackedObstacle> track(new TrackedObstacle(next_obstacle_id_++, default_filter_, observation, timestamp));
    track->estimateModel();
    track->incorporateClassification(observation->type, observation->response_);
//    printf("adding new track with %s classification, %f\n", obstacle_type2str(observation->type), track->getConfidence());
    obstacles_.push_back(track);
  }
}

// (2)
//template<class T> void TrackerKF::observationUpdate(double timestamp, vector< shared_ptr<T> >& observations, dgc_sensor_type type) {
//
//  gsl_matrix_set_identity(observation_mat_);
//  gsl_matrix_set_identity(observation_cov_);
//  matrix_scale(observation_cov_, (parameters_->radar_stddev)*parameters_->radar_stddev);
//}

void TrackerKF::observationUpdate(vector< shared_ptr<Obstacle> >& observations, double timestamp) {
  // set up laser matrices
  gsl_matrix_set_identity(observation_mat_);
  gsl_matrix_set_identity(observation_cov_);
  matrix_scale(observation_cov_, (parameters_->velodyne_stddev)*parameters_->velodyne_stddev);

  vector< shared_ptr<TrackedObstacle> > new_tracks;
  // Consider every possible combination of currently tracked obstacles and observations
  // TODO: consider combining sets of highest scoring correspondances

  sort(obstacles_.begin(), obstacles_.end(), compareTrackedObstaclesConfidence);

  static float threshold = 0.0; // TODO: learn better threshold
  for (vector< shared_ptr<TrackedObstacle> >::iterator to = obstacles_.begin(); to != obstacles_.end(); to++) {
    shared_ptr<TrackedObstacle> track = (*to);

    float max_score = -FLT_MAX;
    shared_ptr<TrackedObstacle> max_obstacle;
    shared_ptr<Obstacle> matched_observation;

    for (vector< shared_ptr<Obstacle> >::iterator obs = observations.begin(); obs != observations.end(); obs++) {
      shared_ptr<Obstacle> observation = (*obs);
      if (observation->getMatched())
        continue;

      float score = track->scoreObservation(observation.get());
      if (score > threshold) {
        shared_ptr<TrackedObstacle> new_obstacle(new TrackedObstacle(*track.get(), observation, timestamp));
        //tracked_obstacles.insert(new_obstacle);
        if (score > max_score) {
          max_score = score;
          max_obstacle = new_obstacle;
          matched_observation = observation;
        }
      }
    }

    // TODO: consider occlusion

    // Best matching obstacle gets to keep the original id
    if (max_obstacle.get() == NULL) {
      // consider case where no observation corresponds to tracked obstacle
      shared_ptr<TrackedObstacle> new_obstacle( new TrackedObstacle(*track.get(), timestamp));
      max_obstacle = new_obstacle;
      new_obstacle->id = track->id;
    }

    if (matched_observation.get() != NULL) { 
      matched_observation->setMatched(true);
    }
    
    max_obstacle->id = track->id;
    max_obstacle->estimateModel();
    new_tracks.push_back(max_obstacle);
  }

  // initialize new obstacles out of un-matched observations
  for (vector< shared_ptr<Obstacle> >::iterator obs = observations.begin(), end = observations.end(); obs != end; obs++) {
    shared_ptr<Obstacle> observation = (*obs);
    if (observation->getMatched())
      continue;

    shared_ptr<TrackedObstacle> new_obstacle(new TrackedObstacle(next_obstacle_id_++, default_filter_, (*obs), timestamp));
    new_tracks.push_back(new_obstacle);

    // TODO: add moving obstacle hypothesis, perhaps hypothesis with 90 degree rotation
  }


  // remove tracked obstacles with low confidence
  for (vector< shared_ptr<TrackedObstacle> >::iterator to = new_tracks.begin(); to != new_tracks.end(); ) {
    shared_ptr<TrackedObstacle> track = (*to);
    if ( track->getConfidence() <= parameters_->confidence_track_min ) {
      to = new_tracks.erase(to);
      continue;
    }
    to++;
  }

  // add tracks to map from observations to tracks
  observation_map_.clear();
  for (vector< shared_ptr<TrackedObstacle> >::iterator to = new_tracks.begin(); to != new_tracks.end(); ) {
    shared_ptr<TrackedObstacle> track = (*to);
    shared_ptr<Obstacle> observation = track->getLastObservation();
    observation_map_.insert(  observation_map_type::value_type(observation->id, track));
    to++;
  }

  obstacles_.swap(new_tracks);

//  std::set<Obstacle*> unassigned;
//  if (sensor_valid) {
//    std::vector<KalmanFilter*> corresponded;
//    std::map<KalmanFilter*, Obstacle*> corres;
//    corresponded.insert(corresponded.begin(), obstacles.size(), NULL);
//    assert(corresponded.size() == obstacles.size());
//    for (std::vector<KalmanFilter*>::iterator it = corresponded.begin(); it != corresponded.end(); it++)
//      assert(*it == NULL);
//
//    std::set<TrackedObstacle*> unobserved(obstacles_);
//    /* get correspondence between obstacles and filters:
//     * - obstacles is vector of all obstacles
//     * - corresponded is vector of pointers to filters that
//     *   complete parallel to obstacles vector...some pointers
//     *   are NULL, indicating that some obstacles were not
//     *   corresponded with any filter...they will be added
//     *   to the unassigned set below and used to make new filters
//     * - unobserved is a set of KFs that weren't corresponded
//     *   to any obstacle...they will not have an observation
//     *   update this timestamp (meaning they will call the
//     *   observationUpdateUnseen function to reduce their
//     *   confidence...)
//     */
//
//    preprocessCorrespondence(obstacles, corresponded, unobserved, sensor_type);
//    assert(obstacles.size() == corresponded.size());
//    getCorrespondence(obstacles, corresponded, unobserved);
//    assert(obstacles.size() == corresponded.size());
//    postprocessCorrespondence(obstacles, corresponded, unobserved, sensor_type);
//    assert(obstacles.size() == corresponded.size());
//
//    // reduce confidence of unseen obstacles
//    // TODO:
//    //  1) detect occlusion and reduce confidence penalty
//    //     for unseen obstacles that are occluded
//    //  2) reduce confidence penalty if obstacle outside
//    //     sensor's field of view (i.e., laser max range
//    //     or radar cone)
//    for (std::set<KalmanFilter*>::iterator kfit = unobserved.begin(); kfit != unobserved.end(); kfit++) {
//      KalmanFilter* kf = *kfit;
//      gsl_vector* mean = kf->getMean();
//      if (!boolObstacleOccluded(sensor_type, vector_get(mean, 0), vector_get(mean, 1)))
//        updateKFConfidence(kf, parameters_->confidence_increment_unobs);
//    }
//
//    // do observation updates for those corresponded KFs/obstacles
//    // (obstacles whose KF pointer is NULL were not corresponded
//    // and will be used to generate new KFs)
//    for (unsigned int i = 0; i < corresponded.size(); ++i) {
//      Obstacle* obs = obstacles[i];
//      KalmanFilter* kf = corresponded[i];
//      if (kf != NULL) {
//        // calculate innovation for observation update
//        gsl_vector* innovation_vector = calculateInnovation(kf, obs, sensor_type);
//        kf->observationUpdate(timestamp, obs_mat, obs_cov, innovation_vector);
//        kf->setYaw(obs->pose.yaw);
//        vector_free(innovation_vector);
//        // store this timestep's observation points
//        kf->setData((void*)obs);
//
//        obs->id = kf->getID();
//        // -Todo- : distinguish between two sensors
//        updateKFConfidence(kf, parameters_->confidence_increment_obs);
//      } else {
//        unassigned.insert(obs);
//      }
//    }
//  }
//
//  // kill KFs with low confidence
//  for (std::set<KalmanFilter*>::iterator kfit = kf_->begin(); kfit != kf_->end(); kfit++) {
//    if (boolRemoveFilter(*kfit)) {
//    	delete *kfit;
//      kf_->erase(kfit);
//    }
//  }
//
//  // create new filters for "new" obstacles
//  for (std::set<Obstacle*>::iterator unit = unassigned.begin(); unit != unassigned.end(); unit++) {
//    if (boolAddNewFilter(*unit, sensor_type)) {
//      KalmanFilter* kf = new KalmanFilter(next_kf_id_++, k_default_state_dim_, timestamp);
//      initFilter(kf, *unit, sensor_type);
//      kf->setData((void*) *unit);
//      kf_->insert(kf);
//      (*unit)->id = kf->getID();
//    }
//  }
}

// (3)
void TrackerKF::getDynamicObstacles(vector<shared_ptr<TrackedObstacle> >& obstacles) {
  getDynamicObstacles(obstacles, parameters_->confidence_min - 10);
}

void TrackerKF::getDynamicObstacles(vector< shared_ptr<TrackedObstacle> >& obstacles, double confidence_min) {
  obstacles.clear();
  for (vector< shared_ptr<TrackedObstacle> >::iterator obs = obstacles_.begin(); obs != obstacles_.end(); obs++) {
    shared_ptr<TrackedObstacle> track = *obs;
    if (track->getConfidence() >= confidence_min) {
      obstacles.push_back(track);
    }
  }
}

//void TrackerKF::getCorrespondence(std::vector<dgc::Obstacle*>& obstacles, std::vector<dgc::TrackedObstacle*>& corresponded,
//    std::set<dgc::TrackedObstacle*>& available) {
//  /** APPROACH: use closest laser point OR bounding box center,
//   * whichever is closer, to do correspondence...
//   */
//  int already = 0;
//  unsigned int start = available.size();
//  for (unsigned int i = 0; i < corresponded.size(); ++i)
//    if (corresponded[i] != NULL)
//      already++;
//  if (DEBUG >= 1)
//    fprintf(stderr, "beg corr: %d already, %u avail\n", already, (unsigned int)available.size());
//  unsigned int matched = 0;
//
//  if (corresponded.size() < obstacles.size()) {
//    if (DEBUG >= 1)
//      fprintf(stderr, "corr size %u < obs size %u...BAD\n", (unsigned int)corresponded.size(), (unsigned int)obstacles.size());
//    corresponded.insert(corresponded.end(), obstacles.size() - corresponded.size(), NULL);
//  }
//  for (unsigned int obs_index = 0; obs_index < obstacles.size(); ++obs_index) {
//    Obstacle* obs = obstacles[obs_index];
//    if (corresponded[obs_index] != NULL) {
//      if (DEBUG >= 1)
//        fprintf(stderr, "already assigned!\n");
//      continue;
//    }
//    KalmanFilter* kfp= NULL;
//    double best_dist = -1;
//    for (std::set<KalmanFilter*>::iterator kfit = available.begin(); kfit != available.end(); kfit++) {
//      KalmanFilter* kf = *kfit;
//      assert(kf != NULL);
//      gsl_vector* mean = kf->getMean();
//      double xd= vector_get(mean, 0) - obs->pose.x;
//      double yd= vector_get(mean, 1) - obs->pose.y;
//      double dist = sqrt(xd*xd + yd*yd);
//      std::vector<point3d_t>& points = obs->getPoints();
//      for (std::vector<point3d_t>::iterator pit = points.begin(); pit != points.end(); pit++) {
//        point3d_t& p = *pit;
//        xd = vector_get(mean, 0) - p.x;
//        yd = vector_get(mean, 1) - p.y;
//        double temp = sqrt(xd*xd + yd*yd);
//        dist = temp < dist ? temp : dist;
//      }
//      if (best_dist < 0|| dist < best_dist) {
//        best_dist = dist;
//        if (dist < parameters_->max_dist_correspondence) {
//          kfp = kf;
//        }
//      }
//    }
//    corresponded[obs_index] = kfp;
//    if (kfp != NULL) {
//      matched++;
//      available.erase(kfp);
//    } else if (DEBUG >= 2)
//      fprintf(stderr, "obs(%d): kfp NULL, best %f vs. thresh %f\n", obs_index, best_dist,
//      parameters_->max_dist_correspondence);
//  }
//  if (DEBUG >= 1) {
//    assert(start == available.size() + matched);
//    fprintf(stderr, "end corr: %d already, %d matched, %u avail\n", already, matched, (unsigned int)available.size());
//  }
//}

//#define USE_CALC_INNOV_APPROACH_ONE 0
//
//gsl_vector* TrackerKF::calculateInnovationLaser(KalmanFilter* kf, Obstacle* obstacle, gsl_matrix* obs_mat) {
//
//  gsl_vector* innovation_vector = vector_init(k_obs_dim_laser_);
//  gsl_vector* pobs = kf->predictObservation(kf->getMean(), obs_mat);
//
//  if (USE_CALC_INNOV_APPROACH_ONE) {
//    /** APPROACH ONE: use laser point CLOSEST to predicted
//     * observation as evidence -- i.e., choose point from
//     * segmented cluster that minimizes the norm of the
//     * innovation and use that innovation...
//     **/
//    double ix = 0, iy = 0;
//    double best_d = -1;
//    std::vector<point3d_t>& points = obstacle->getPoints();
//    for (std::vector<point3d_t>::iterator pit = points.begin(); pit != points.end(); pit++) {
//      point3d_t& p = *pit;
//      ix = p.x - vector_get(pobs, 0);
//      iy = p.y - vector_get(pobs, 1);
//      double d = ix*ix + iy*iy;
//      if (best_d < 0|| d < best_d) {
//        best_d = d;
//        vector_set(innovation_vector, 0, ix);
//        vector_set(innovation_vector, 1, iy);
//      }
//    }
//  } else {
//    /** APPROACH TWO: use "center" of "bounding box"
//     * as evidence (i.e., innovation is difference
//     * between predicted observation and bounding
//     * box center...
//     **/
//    vector_set(innovation_vector, 0, obstacle->pose.x);
//    vector_set(innovation_vector, 1, obstacle->pose.y);
//    vector_subtract_ip(innovation_vector, pobs);
//  }
//
//  // TODO: add IPC for innovation step
//
//  vector_free(pobs);
//  return innovation_vector;
//}

//void TrackerKF::initFilterRadar(KalmanFilter* kf, dgc::RadarObstacle* obstacle, double max_range) {
//  gsl_vector* mean = kf->getMean();
//  if (obstacle != NULL) {
//    vector_set(mean, 0, obstacle->pose.x);
//    vector_set(mean, 1, obstacle->pose.y);
//    // TODO: figure out velocity direction
//    vector_set(mean, 2, obstacle->x_vel);
//    vector_set(mean, 3, obstacle->y_vel);
//    kf->setYaw(obstacle->pose.yaw);
//  } else
//    gsl_vector_set_zero(mean);
//
//  // set initial covariance values
//  gsl_matrix* covariance = kf->getCovariance();
//  double l_cov = parameters_->default_loc_stddev * parameters_->default_loc_stddev;
//  double v_cov = parameters_->default_vel_stddev * parameters_->default_vel_stddev;
//  matrix_set(covariance, 0, 0, l_cov);
//  matrix_set(covariance, 1, 1, l_cov);
//  matrix_set(covariance, 2, 2, v_cov);
//  matrix_set(covariance, 3, 3, v_cov);
//
//  // set transition model to default
//  kf->setTransitionMatrix(transition_mat_, 0);
//  kf->setTransitionCovariance(transition_cov_, 0);
//
//  double confidence = 0;
//  if (obstacle != NULL) {
//    double range = kogmo_range(obstacle->pose.x, obstacle->pose.y);
//    confidence = kogmo_logistic(range, max_range, parameters_->confidence_initial_max, parameters_->confidence_initial_min);
//  }
//}

//void TrackerKF::initFilter(KalmanFilter* kf, Obstacle* obstacle, dgc_sensor_type sensor_type) {
//  switch (sensor_type) {
//  case SENSOR_RADAR:
//    // TODO:
//    break;
//  case SENSOR_VELODYNE:
//    initFilterLaser(kf, obstacle, parameters_->velodyne_max_range);
//    break;
//  default:
//    fprintf(stderr, "Unknown sensor type...cannot initialize new kf!\n");
//    if (kf != NULL)
//      delete kf;
//    break;
//  }
//}
//
//gsl_vector* TrackerKF::calculateInnovation(KalmanFilter* kf, Obstacle* obstacle, dgc_sensor_type sensor_type) {
//  switch (sensor_type) {
//  case SENSOR_RADAR:
//    // TODO:
//    break;
//
//  case SENSOR_VELODYNE:
//    return calculateInnovationLaser(kf, obstacle, obs_mat_velodyne_);
//    break;
//
//  default:
//    fprintf(stderr, "Unknown sensor type...cannot calculate innovation!\n");
//    return NULL;
//    break;
//  }
//  return NULL;
//}

//void TrackerKF::updateKFConfidence(KalmanFilter* kf, double increment) {
//  kf->setConfidence(kf->getConfidence() + increment);
////  confidence = confidence < parameters_->confidence_max ? confidence : parameters_->confidence_max;
////  confidence = confidence > parameters_->confidence_min ? confidence : parameters_->confidence_min;
////  kf->setConfidence(confidence);
//}

void TrackerKF::velocityXYtoVR(double xv, double yv, double* v, double* r) {
  (*v) = sqrt(xv*xv + yv*yv);
  (*r) = atan2(yv, xv);
}

//TrackedObstacle* TrackerKF::makeTrackedObstacleFromFilter(KalmanFilter* kf) {
//  double velocity, yaw;
//
//  TrackedObstacle* obstacle = new TrackedObstacle(kf->getID());
//  obstacle->confidence = kf->getConfidence();
//
//  gsl_vector* mean = kf->getMean();
//  obstacle->pose.x = vector_get(mean, 0);
//  obstacle->pose.y = vector_get(mean, 1);
//  obstacle->x_vel = vector_get(mean, 2);
//  obstacle->y_vel = vector_get(mean, 3);
//
//  velocity = hypot(vector_get(mean, 2), vector_get(mean, 3));
//  kf->setMaxSpeed(std::max(kf->getMaxSpeed(), fabs(velocity)));
//  yaw = kf->getYaw();
//  obstacle->max_speed = kf->getMaxSpeed();
//  obstacle->pose.yaw = yaw;
//
//  gsl_matrix* covariance = kf->getCovariance();
//  obstacle->x_var = matrix_get(covariance, 0, 0);
//  obstacle->y_var = matrix_get(covariance, 1, 1);
//  obstacle->xy_cov = matrix_get(covariance, 1, 0);
//
////  obstacle->type = 0;
////  obstacle->first_timestamp = kf->getFirstTimestamp();
////  obstacle->timestamp = kf->getCurrentTimestamp();
////
////  obstacle->points.clear();
//
////  std::vector<gsl_vector*>* points = kf->getAuxiliaryPoints();
////  for (std::vector<gsl_vector*>::iterator it = points->begin(); it != points->end(); it++)
////    obstacle->points.push_back(convertVectorToDGCPoint(*it));
//
//  if (kf->getFirstTimestamp() == kf->getCurrentTimestamp())
//    obstacle->tracking_state = TRACKING_STATE_INITIALIZED;
//  else if (fabs(kf->getLastObservationTimestamp() - kf->getCurrentTimestamp()) > CONSIDER_LOST)
//    obstacle->tracking_state = TRACKING_STATE_LOST;
//  else
//    obstacle->tracking_state = TRACKING_STATE_TRACKING;
//
//  /** things I don't know about **/
//  obstacle->pose.z = obstacle->length = obstacle->width = -1;
//
//  return obstacle;
//}

gsl_vector* TrackerKF::convertDGCPointToVector(point3d_t& p) {
  gsl_vector* v = vector_init(k_default_state_dim_ + 1);
  gsl_vector_set_zero(v);
  vector_set(v, 0, p.x);
  vector_set(v, 1, p.y);
  vector_set(v, k_default_state_dim_, p.z);
  return v;
}

point3d_t TrackerKF::convertVectorToDGCPoint(gsl_vector* v) {
  point3d_t p;
  p.x = vector_get(v, 0);
  p.y = vector_get(v, 1);
  if (v->size > (unsigned int)k_default_state_dim_)
    p.z = vector_get(v, k_default_state_dim_);
  else
    p.z = 0;
  return p;
}

//void TrackerKF::replaceKFAuxiliaryPoints(KalmanFilter* kf, Obstacle& obs) {
//  if (obs.points.empty())
//    return;
//  std::vector<gsl_vector*>* aux_points = kf->getAuxiliaryPoints();
//  for (std::vector<gsl_vector*>::iterator it = aux_points->begin(); it != aux_points->end(); it++)
//    vector_free(*it);
//  aux_points->clear();
//
//  int counter = 0;
//  for (std::vector<point3d_t>::iterator it = obs.points.begin(); it != obs.points.end(); it++)
//  {
//    aux_points->push_back(convertDGCPointToVector(*it));
//    counter++;
//  }
//  //printf("Anzahl der Punke: %i \n", counter);
//}

