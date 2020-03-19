#include <assert.h>
#include <lltransform.h>
#include <aw_roadNetworkSearch.h>

#include "perception.h"
#include "utils.h"
#include "obstacle.h"
#include "segment.h"
#include "laser_segment.h"
#include "box.h"
//REMOVE: #include "kalman_tracker.h"
#include "kalman_multitracker.h"


#include <iostream>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <pthread.h>
#include <queue>

using namespace std;
using namespace std::tr1;
using namespace dgc;

using namespace vlr::rndf;

using namespace Eigen;

//TODO: turn into param?
#define MIN_CONFIDENCE_CLUSTER      1.0

extern dgc::VelodyneRings* rings;
//REMOVE: TrackerKF* tracker_ = NULL;
KalmanMultiTracker* tracker_ = NULL;

char* obstacle_type_str[] =
{
    "unknown", "car", "pedestrian", "bicyclist"
};

char* obstacle_type2str(dgc_obstacle_type type) {
  if (type <= OBSTACLE_BICYCLIST )
    return obstacle_type_str[type];
  else
    return obstacle_type_str[0];
}


class ClassifierStack {
public:
  ClassifierStack(){}

  void push_inbox(shared_ptr<Obstacle> obstacle) {
    boost::mutex::scoped_lock lock(inbox_mutex_);
    inbox_.push_back(obstacle);
  }
  shared_ptr<Obstacle> pop_inbox() {
    boost::mutex::scoped_lock lock(inbox_mutex_);
    shared_ptr<Obstacle> obstacle = inbox_.back();
    inbox_.pop_back();
    return obstacle;
  }
  int inbox_empty() {
    boost::mutex::scoped_lock lock(inbox_mutex_);
    return inbox_.empty();
  }
  int clear_inbox() {
    boost::mutex::scoped_lock lock(inbox_mutex_);
    int s = inbox_.size();
    inbox_.clear();
    return s;
  }

  void push_outbox(shared_ptr<Obstacle> obstacle) {
    boost::mutex::scoped_lock lock(outbox_mutex_);
    outbox_.push_back(obstacle);
  }
  shared_ptr<Obstacle> pop_outbox() {
    boost::mutex::scoped_lock lock(outbox_mutex_);
    shared_ptr<Obstacle> obstacle = outbox_.back();
    outbox_.pop_back();
    return obstacle;
  }
  int outbox_empty() {
    boost::mutex::scoped_lock lock(outbox_mutex_);
    return outbox_.empty();
  }

private:
  boost::mutex inbox_mutex_;
  boost::mutex outbox_mutex_;

  vector< shared_ptr<Obstacle> > inbox_;
  vector< shared_ptr<Obstacle> > outbox_;
};

ClassifierStack classifier_stack;

void classifier_thread()
{
  // TODO(teichman): Alex, you may want to match this with the number of threads you're using...
  static const unsigned int batch_size = 8;

  vector< shared_ptr<Obstacle> > filtered;
  filtered.reserve(batch_size);
  while (true) {
//  while (! classifier_stack.inbox_empty()) {
  pthread_mutex_lock(&shutdown_mutex);
  if(shutdown == 2) {
    printf("Classifier shutting down\n");
    shutdown = 1;
    pthread_mutex_unlock(&shutdown_mutex);
    break;
  }
  pthread_mutex_unlock(&shutdown_mutex);  
    if (classifier_stack.inbox_empty()) {
      usleep(1000);
    } else {
      filtered.clear();

      // process batch_size at a time
      while ((!classifier_stack.inbox_empty()) && (filtered.size() < batch_size)) {
        filtered.push_back(classifier_stack.pop_inbox());
      }

//      printf("classifying %d obstacles\n", filtered.size());

      // -- Convert dgc clouds into Eigen clouds.
      vector< boost::shared_ptr<MatrixXf> > clouds(filtered.size());
      vector< boost::shared_ptr<VectorXf> > intensities(filtered.size());
      for(size_t i = 0; i < filtered.size(); ++i) {
        dgcToEigen(filtered[i]->getPoints(), &clouds[i], &intensities[i]);
      }
      
      // -- Classify them.
      vector<VectorXf> responses = g_classifier_pipeline->classify(clouds, intensities);
      // -- Set the label for each.
      assert(responses.size() == filtered.size());
      for(size_t i = 0; i < filtered.size(); ++i) {
        shared_ptr<Obstacle> observation = filtered[i];
        observation->response_ = responses[i];
        int idx;
        float max = observation->response_.maxCoeff(&idx);
        if(max <= 0)
          observation->type = OBSTACLE_UNKNOWN;
        else {
          if(booster->class_map_.toName(idx).compare("car") == 0)
            observation->type = OBSTACLE_CAR;
          else if(booster->class_map_.toName(idx).compare("pedestrian") == 0)
            observation->type = OBSTACLE_PEDESTRIAN;
          else if(booster->class_map_.toName(idx).compare("bicyclist") == 0)
            observation->type = OBSTACLE_BICYCLIST;
          else
            assert(0);
//          printf("found %s\n", obstacle_type2str(observation->type));
        }

        classifier_stack.push_outbox(observation);
      }
    }
  }
}

bool compareObstaclesRndfDist (shared_ptr<Obstacle> a, shared_ptr<Obstacle> b) {
  return (a->getRndfDist() < b->getRndfDist());
}

void
perception_classify_obstacles(vector< shared_ptr<Obstacle> >& obstacles, double timestamp) {
  if(getenv("HALT_EARLY")) { 
    static int cts = 0;
    cts++;
    if(cts == 5)
      exit(0);
  }

  if(!booster || obstacles.empty())
    return;

  static boost::thread thread(classifier_thread);

  // sort by distance to rndf
  sort(obstacles.begin(), obstacles.end(), compareObstaclesRndfDist);

  // Add new obstacles to classify()
  int cleared = classifier_stack.clear_inbox();
  printf("unable to classify %d obstacles\n", cleared);

  int num_to_classify = 0;
  for(size_t i = 0; i < obstacles.size(); ++i) {

    if(obstacles[i]->getPoints().size() < 10) {
      // @TODO move this somewhere more appropriate.  
      obstacles[i]->response_ = 0.5 * booster->prior_;
    }
    else {
      // @TODO move this somewhere more appropriate.  
      classifier_stack.push_inbox(obstacles[i]);
      num_to_classify++;
    }
  }

  obstacles_segmented.clear();
  // Handle classified obstacles
  double starttime = dgc_get_time();
  while (obstacles_segmented.size() < num_to_classify) {
    //if(starttime + 0.1 < dgc_get_time())
    //  break;
    while(classifier_stack.outbox_empty()) {
      pthread_mutex_lock(&shutdown_mutex);
      if(shutdown > 0) {
        pthread_mutex_unlock(&shutdown_mutex);
        thread.join();
        return;
      }
      pthread_mutex_unlock(&shutdown_mutex);
      usleep(100);
    }
    shared_ptr<Obstacle> observation = classifier_stack.pop_outbox();
    obstacles_segmented.push_back(observation);
    //tracker_->classificationUpdate(observation, observation->time_);
  }
//  classifier_thread();
}

//   vector<Object*> objs;
//   if(booster) {
//     // -- Collect features, put into objects.
//     vector< vector<point3d_t> > clusters(obstacles.size());
//     size_t cluster_id = 0;
//     for (vector< shared_ptr<Obstacle> >::iterator it = obstacles.begin(), end = obstacles.end(); it != end; it++, ++cluster_id) {
//       shared_ptr<Obstacle> obstacle = *it;
//       std::vector<point3d_t>& points = obstacle->getPoints();
//       clusters[cluster_id] = points;
//     }
//     if(!clusters.empty()) {
// //      cout << "z: " << get_z() << endl;
//       collectDatasetForSpin(clusters, get_z(), &objs); //TODO: replace 0 with robot z height.
//     }
//   }

//   size_t cluster_id = 0;
//   //  int count = 0;
//   for (vector< shared_ptr<Obstacle> >::iterator it = obstacles.begin(), end = obstacles.end(); it != end; it++, ++cluster_id) {
//     shared_ptr<Obstacle> obstacle = *it;
//     obstacle->type = OBSTACLE_UNKNOWN;

//     if(booster && !objs.empty() && objs[cluster_id]) {
//       obstacle->response_ = booster->treeClassify(*objs[cluster_id]);
//       int idx;
//       float max = obstacle->response_.maxCoeff(&idx);
//       if(max <= 0)
//         obstacle->type = OBSTACLE_UNKNOWN;
//       else {
//         if(booster->class_map_.toName(idx).compare("car") == 0)
//           obstacle->type = OBSTACLE_CAR;
//         else if(booster->class_map_.toName(idx).compare("pedestrian") == 0)
//           obstacle->type = OBSTACLE_PEDESTRIAN;
//         else if(booster->class_map_.toName(idx).compare("bicyclist") == 0)
//           obstacle->type = OBSTACLE_BICYCLIST;
//         else
//           assert(0);
//       }
//     }
//   }

//   for(size_t i=0; i<objs.size(); ++i) {
//     delete objs[i];
//   }


//inline double wrap_angle(double y) {
//  if (y > M_PI)
//    y = M_2_PI - y;
//  return y;
//}
//
//inline double dist_angles(double y1, double y2) {
//  double dist = y1 - y2;
//
//  while (dist < 0)
//    dist += M_2_PI;
//  while (dist > M_PI)
//    dist -= M_2_PI;
//
//  return fabs(dist);
//}
//
//// coarse yaw is coarse, but is in the right quadrant
//// refined yaw is fine, but ambigous relative to quadrant
//double disambiguate_yaw(double refined_yaw, double coarse_yaw) {
//  double yaw = refined_yaw - M_PI;
//  double dist = dist_angles(yaw, coarse_yaw);
//  if (dist < M_PI_2)
//    return yaw;
//
//  yaw += M_PI_2;
//  dist = dist_angles(yaw, coarse_yaw);
//  if (dist < M_PI_2)
//    return yaw;
//
//  yaw += M_PI_2;
//  dist = dist_angles(yaw, coarse_yaw);
//  if (dist < M_PI_2)
//    return yaw;
//
//  return yaw + M_PI_2;
//}

void
perception_track_obstacles(vector< shared_ptr<Obstacle> >& detected_obstacles, vector< shared_ptr<TrackedObstacle> >& tracked_obstacles, double timestamp)
{
  static bool init = false;

  //MOVE: shouldn't this go into a seperate method?
  if (!init) {
    tracker_ = new KalmanMultiTracker(
        settings.kf_settings.correspondence_threshold,
        settings.kf_settings.pruning_threshold,
        settings.kf_settings.measurement_variance,
        settings.kf_settings.position_variance,
        settings.kf_settings.velocity_variance,
        settings.kf_settings.initial_position_variance,
        settings.kf_settings.initial_velocity_variance
        );
    //REMOVE: tracker_ = new TrackerKF();
    //REMOVE: tracker_->setParameters(&settings.kf_settings);
    init = true;
  }

  /*vector<MultiTrackerMeasurement> observations;
  vector< shared_ptr<Obstacle> >::iterator it;
  for(it = detected_obstacles.begin(); it != detected_obstacles.end(); it++) {
    observations.push_back(MultiTrackerMeasurement());
    observations.back().centroid_(0) = (*it)->pose.x;
    observations.back().centroid_(1) = (*it)->pose.y;
    // @TODO: give each obstacle it's own timestamp
    observations.back().timestamp = timestamp;
  }*/
  tracker_->step(detected_obstacles, timestamp);
  //REMOVE: tracker_->transitionUpdate(timestamp);

//  tracker_->trackGetDynamicObstacles(obstacles_predicted, MIN_CONFIDENCE_CLUSTER);
//  std::sort(obstacles_predicted.begin(), obstacles_predicted.end(), compareObstaclesConfidence);
//  if(params.predictive_clustering) {
//    cluster_obstacles_bb(&frame, &predicted_obstacle_list_, &segmented_obstacle_list_, local_x_vel, local_y_vel, 1);
//  }
  tracked_obstacles.clear();
  list<shared_ptr<TrackedObstacle> >::iterator it;
  for(it = tracker_->tracks_.begin(); it != tracker_->tracks_.end(); it++) {
    tracked_obstacles.push_back(*it);
  }

  //REMOVE: tracker_->observationUpdate(detected_obstacles, timestamp);
  //REMOVE: tracker_->getDynamicObstacles(tracked_obstacles, settings.kf_settings.confidence_publish_min);
}

void perception_filter_obstacles_height(vector< shared_ptr<Obstacle> >& obstacles, double min_height)
{
//  static double min_width = 0.5;
//  static double max_width = 25.0;
//
//  static double min_length = 0.5;
//  static double max_length = 25.0;

  vector< shared_ptr<Obstacle> >::iterator it;
  for(it = obstacles.begin(); it != obstacles.end(); )
  {
    float height = (*it)->maxHeight();
    if (height < min_height) {
      it = obstacles.erase(it);
      continue;
    }

    ++it;
  }
}

/*REMOVE:
void perception_filter_obstacles_tracked(vector< shared_ptr<TrackedObstacle> >& obstacles)
{
  static double min_width = 0.5;
  static double max_width = 25.0;

  static double min_length = 0.5;
  static double max_length = 25.0;
  vector< shared_ptr<TrackedObstacle> >::iterator it;
  for(it = obstacles.begin(); it != obstacles.end(); )
  {
    shared_ptr<TrackedObstacle> track = *it;

    if (track->type != OBSTACLE_UNKNOWN) {
      ++it;
      continue;
    }

    double length = track->length;
    if ((length > max_length) || (length < min_length)){
      it = obstacles.erase(it);
      continue;
    }

    double width = track->width;
    if ((width > max_width) || (width < min_width)){
      it = obstacles.erase(it);
      continue;
    }

    // TODO(charles): filter small obstacles whose movement does not match velocity over time

    ++it;
  }
//  printf("Erased %d obstacles from tracked list\n", erased);
}
*/

void perception_merge_near_obstacles(vector<GridObstacle*>& obstacles)
{
//  double dist;
//  double x1,y1,z1;
//  double x2,y2,z2;
//  double length,width;
//
//  vector<GridObstacle*>::iterator it;
//  vector<GridObstacle*>::iterator it2;
//  for(it = obstacles.begin(); it != obstacles.end(); it++)
//  {
//   Obstacle* obstacle_ref = *it;
//   x1 = obstacle_ref->pose.x;
//   y1 = obstacle_ref->pose.y;
//   z1 = obstacle_ref->pose.z;
//   width = obstacle_ref->width+obstacle_ref->width*0.2;
//   length = obstacle_ref->length + settings.tracker_settings.merge_dist;
//
//   it2 = it;
//   it2++;
//   for(; it2 != obstacles.end(); )
//   {
//     Obstacle* obstacle = *it2;
//     x2=obstacle->pose.x;
//     y2=obstacle->pose.y;
//     z2=obstacle->pose.z;
//     dist=hypot(obstacle_ref->pose.x-obstacle->pose.x,
//                obstacle_ref->pose.y-obstacle->pose.y);
//  //      fprintf(stderr, "1: (%f, %f) and 2: (%f, %f) at %f m\n", x1, y1, x2, y2, dist);
//  //      fprintf(stderr, "width: %f; length: %f max_dist:%f\n",width, length, max_dist);
//     // two obstacles close to each other in the same driving direction
//     if ((x2 < (x1 + length / 2)) &&
//         (x2 > (x1 - length / 2)) &&
//         (y2 < (y1 + width / 2)) &&
//         (y2 > (y1 - width / 2))  ) {
//       // inside the bounding box
//       // merge both obstacles
//
//  //        if(DEBUG >= 2) {
//  //          fprintf(stderr, "**merged %d and %d at %f\n",obstacle_ref.id,obstacle->id, dist);
//  //          fprintf(stderr, "**width: %f; length: %f max_dist:%f\n",width, length, max_dist);
//  //        }
//       obstacle_ref->merge(*obstacle);
//
//       delete (*it2);
//       it2 = obstacles.erase(it2);
//     }
//     else {
//        it2++;
//     }
//   }
//  }
//
//  sla::Vec2d c1,c2;
//  bool bintersect;
//  for (it = obstacles.begin(); it != obstacles.end(); it++) {
//   Obstacle* obstacle_ref = *it;
//   c1[0] = obstacle_ref->pose.x;
//   c1[1] = obstacle_ref->pose.y;
//
//   it2 = it;
//   it2++;
//   for (; it2 != obstacles.end();) {
//     Obstacle* obstacle = *it2;
//     c2[0] = obstacle->pose.x;
//     c2[1] = obstacle->pose.y;
//     bintersect = AnnieWAY::rect_rect_X(c1, obstacle_ref->pose.yaw, obstacle_ref->width, obstacle_ref->length,
//                                         c2, obstacle->pose.yaw, obstacle->width, obstacle->length);
//     if(bintersect) {
//       obstacle_ref->merge(*obstacle);
//       delete (*it2);
//       it2 = obstacles.erase(it2);
//     } else {
//       it2++;
//     }
//   }
//  }
}

void perception_filter_obstacles_rndf(vector< shared_ptr<Obstacle> >& obstacles, double max_distance)
{
  if(!settings.tracker_settings.filter_rndf) {
    return;
  }
  
  static RoadNetworkSearch *rn_search_ = NULL;

  if (rn_search_ == NULL) {
    rn_search_ = new RoadNetworkSearch(road_network);
  }

  double utm_x, utm_y;
  double distance;

  Lane* l;

  pthread_mutex_lock(&localize_mutex);
  double local_x_off = localize_pos.x_offset;
  double local_y_off = localize_pos.y_offset;
  pthread_mutex_unlock(&localize_mutex);
  vector< shared_ptr<Obstacle> >::iterator it;
  for(it = obstacles.begin(); it != obstacles.end(); )
  {
    shared_ptr<Obstacle> obstacle = *it;
    if (obstacle->type != OBSTACLE_UNKNOWN) {
      ++it;
      continue;
    }
    if (obstacle->getRndfDist() < 0) {
      double x,y;
      if (obstacle->getCenterOfPoints(&x, &y)) {
        // go to utm coordinates (from local coordinates)
        utm_x = x + local_x_off;
        utm_y = y + local_y_off;

        rn_search_->closest_lane(utm_x, utm_y, l, distance);
        obstacle->setLane(l);
        obstacle->setRndfDist(distance);
      } else {
        it = obstacles.erase(it);
          continue;
      }
    }

      // coarse filtering
    if(obstacle->getRndfDist() > max_distance) {
      it = obstacles.erase(it);
      continue;
    }

    ++it;
  }
//  printf("%u obstacles remaining after RNDF filtering.\n", obstacles.size());
}

void
perception_filter_obstacles_fine(vector< shared_ptr<Obstacle> >& obstacles)
{
  perception_filter_obstacles_rndf(obstacles, settings.tracker_settings.filter_rndf_max_distance);
  printf("(3)Obstacles: %d\n", obstacles.size());
}

void
perception_filter_obstacles_coarse(vector< shared_ptr<Obstacle> >& obstacles)
{
  printf("(0)Obstacles: %d\n", obstacles.size());
  perception_filter_obstacles_height(obstacles, settings.segmentation_settings.min_height);
  printf("(1)Obstacles: %d\n", obstacles.size());
  perception_filter_obstacles_rndf(obstacles, settings.tracker_settings.filter_rndf_max_pedestrian_distance);
  printf("(2)Obstacles: %d\n", obstacles.size());
//  perception_merge_near_obstacles(obstacles);
//  printf("%d obstacles after merging\n", obstacles->obstacles.size());
}

void
perception_add_radar(vector< shared_ptr<TrackedObstacle> >& obstacles, double timestamp) {

}

void
perception_track_frame(double timestamp)
{
#ifndef USE_GRID_SEGMENTER
  static LaserSegmenter segmenter(rings);
#else
    static GridSegmenter segmenter;
#endif

  static double last_timestamp = 0;

  if (timestamp == last_timestamp) // just for simulation purposes (perception_viz)
    return;
  last_timestamp = timestamp;

  double time = dgc_get_time();
  obstacles_s->timestamp = timestamp;

#ifndef USE_GRID_SEGMENTER
  segmenter.segmentVelo(lscan, &obstacles_segmented);
#else
  segmenter.segmentGrid(grid, obstacles_s, &obstacles_segmented, timestamp);
#endif
  display_time("Segmenting", time); time = dgc_get_time();
  perception_filter_obstacles_coarse(obstacles_segmented);
  display_time("Filtering", time); time = dgc_get_time();
  //perception_classify_obstacles(obstacles_segmented, timestamp);
  display_time("Classification", time); time = dgc_get_time();
  perception_filter_obstacles_fine(obstacles_segmented);
  display_time("Filtering", time); time = dgc_get_time();
  printf("(0)Tracks: %d\n", (int)obstacles_tracked.size());
  perception_track_obstacles(obstacles_segmented, obstacles_tracked, timestamp);
  printf("(1)Tracks: %d\n", (int)obstacles_tracked.size());
  display_time("Tracking", time);
  //perception_filter_obstacles_tracked(obstacles_tracked);
  //printf("(2)Tracks: %d\n", obstacles_tracked.size());

  vector< shared_ptr<TrackedObstacle> >::iterator it;
  for(it = obstacles_tracked.begin(); it != obstacles_tracked.end(); it++) {
    shared_ptr<TrackedObstacle> track = (*it);
    track->markDynamic(grid, obstacles_s, counter);
  }
  // -- Report stats.
  map<int, int> num_identified;
  for(int i=0; i<3; ++i)
    num_identified[i] = 0;
  //  cout << booster->class_map_.serialize() << endl;
  for(size_t i=0; i<obstacles_tracked.size(); ++i) {
    //    cout << obstacles_tracked[i]->type << " with id " << obstacles_tracked[i]->id << ": " << obstacles_tracked[i]->getLogOdds().transpose() << endl;
    num_identified[obstacles_tracked[i]->type]++;
  }
//  cout << num_identified[OBSTACLE_UNKNOWN] << " background." << endl;
//  cout << num_identified[OBSTACLE_CAR] << " car." << endl;
//  cout << num_identified[OBSTACLE_PEDESTRIAN] << " ped." << endl;
//  cout << num_identified[OBSTACLE_BICYCLIST] << " bike." << endl;
  
  //  perception_classify_obstacles(obstacles_tracked);
//  perception_add_radar(obstacles_tracked, timestamp);

//  if (settings.gls_output) {
//    // setup GLS header //
//    gls->coordinates = GLS_SMOOTH_COORDINATES;
//    gls->origin_x = 0;
//    gls->origin_y = 0;
//    gls->origin_z = applanix_current_pose()->smooth_z;
//    glsColor3f( gls, 1.0, 0.0, 0.0 );
//    glsLineWidth(gls, 2.0);
//    for (int i=0; i < obstacles_tracked.size(); i++) {
//      glsPushMatrix(gls);
//      shared_ptr<TrackedObstacle> obstacle = obstacles_tracked[i];
//      glsTranslatef(gls, obstacle->pose.x, obstacle->pose.y, 0);
//      glsRotatef(gls, dgc_r2d(obstacle->pose.yaw), 0, 0, 1);
//
//      float l = obstacle->length;
//      float w = obstacle->width;
//      glsBegin(gls, GLS_LINE_LOOP);
//      glsVertex3f(gls, l / 2, w / 2, 0);
//      glsVertex3f(gls, l / 2, -w / 2, 0);
//      glsVertex3f(gls, -l / 2, -w / 2, 0);
//      glsVertex3f(gls, -l / 2, w / 2, 0);
//      glsVertex3f(gls, l / 2, 0, 0);
//      glsVertex3f(gls, l / 2 + obstacle->getVelocity(), 0, 0);
//      glsVertex3f(gls, l / 2, 0, 0);
//      glsVertex3f(gls, -l / 2, -w / 2, 0);
//      glsVertex3f(gls, -l / 2, w / 2, 0);
//      glsEnd(gls);
//
//      glsPopMatrix(gls);
//    }
//  }

}

