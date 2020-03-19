#include <boost/format.hpp>
#include <IL/il.h>
#include <ippi.h>

#include <aw_kogmo_math.h>
#include <vlrException.h>
#include <perception_interface.h>

#include "aw_ChsmPlanner.hpp"
#include "collisionCheck.h"
#include "obstaclePrediction.h"

namespace vlr {

#undef TRACE
#define TRACE(str) std::cout << "[ChsmPlanner] "<< str << std::endl

using namespace std;

void ChsmPlanner::updateStaticObstacleMapSize(double new_width, double new_height, double new_resolution) {
  pthread_mutex_lock(&static_obstacle_map_mutex_);
  pthread_mutex_lock(&static_obstacle_points_mutex_);
  if(static_obstacle_map_raw_) {delete[] static_obstacle_map_raw_;}
  if(static_obstacle_map_tmp_) {delete[] static_obstacle_map_tmp_;}
  if(static_obstacle_map_tmp_raw_) {delete[] static_obstacle_map_tmp_raw_;}
  if(static_obstacle_map_tmp_h_) {delete[] static_obstacle_map_tmp_h_;}
  if(static_obstacle_map_tmp_v_) {delete[] static_obstacle_map_tmp_v_;}
  if(static_obstacle_map_) {delete[] static_obstacle_map_;}
  if (gauss_filter_buf_) {delete[] gauss_filter_buf_;}

  static_obstacle_map_width_  = int(new_width  / new_resolution + .5);
  static_obstacle_map_height_ = int(new_height / new_resolution + .5);
  static_obstacle_map_res_    = new_resolution;
  static_obstacle_points_     = new PerceptionObstaclePoint[static_obstacle_map_width_*static_obstacle_map_height_];
  static_obstacle_num_points_ = 0;
  static_obstacle_map_raw_    = new uint8_t[static_obstacle_map_width_ * static_obstacle_map_height_];
  static_obstacle_map_tmp_    = new uint8_t[static_obstacle_map_width_ * static_obstacle_map_height_];
  static_obstacle_map_tmp_raw_    = new int16_t[static_obstacle_map_width_ * static_obstacle_map_height_];
  static_obstacle_map_tmp_h_    = new int16_t[static_obstacle_map_width_ * static_obstacle_map_height_];
  static_obstacle_map_tmp_v_    = new int16_t[static_obstacle_map_width_ * static_obstacle_map_height_];
  static_obstacle_map_        = new uint8_t[static_obstacle_map_width_ * static_obstacle_map_height_];
  memset(static_obstacle_map_raw_, 0, static_obstacle_map_width_ * static_obstacle_map_height_ * sizeof(uint8_t));
  memset(static_obstacle_map_tmp_, 0, static_obstacle_map_width_ * static_obstacle_map_height_ * sizeof(uint8_t));
  memset(static_obstacle_map_tmp_raw_, 0, static_obstacle_map_width_ * static_obstacle_map_height_ * sizeof(int16_t));
  memset(static_obstacle_map_tmp_h_, 0, static_obstacle_map_width_ * static_obstacle_map_height_ * sizeof(int16_t));
  memset(static_obstacle_map_tmp_v_, 0, static_obstacle_map_width_ * static_obstacle_map_height_ * sizeof(int16_t));
  memset(static_obstacle_map_, 0, static_obstacle_map_width_ * static_obstacle_map_height_ * sizeof(uint8_t));

  double rc = traj_eval_->params().safety_width / 2;
  static_obstacle_map_cs_mask_size_ = uint32_t(rc / static_obstacle_map_res_ + .5);
  static_obstacle_map_cs_anchor_    = static_obstacle_map_cs_mask_size_ / 2;

  createGaussFilterMask();

  static_obstacle_map_region_size_.width = static_obstacle_map_width_   - static_obstacle_map_cs_mask_size_;
  static_obstacle_map_region_size_.height = static_obstacle_map_height_ - static_obstacle_map_cs_mask_size_;

  pthread_mutex_unlock(&static_obstacle_points_mutex_);
  pthread_mutex_unlock(&static_obstacle_map_mutex_);
}

void ChsmPlanner::updateStaticObstacleMap(PerceptionObstaclePoint* points, int num_points, double timestamp) {

  if(!points || emergency_map_clear_) {return;}

  pthread_mutex_lock(&static_obstacle_points_mutex_);
  if(num_points>0) {
   memcpy(static_obstacle_points_, points, num_points*sizeof(PerceptionObstaclePoint));
  }
  static_obstacle_num_points_ = num_points;
  static_obstacle_timestamp_ = timestamp;

  new_static_obstacle_map_ready_=true;
  pthread_mutex_lock(&static_obstacle_map_cycle_mutex_);
  pthread_cond_signal(&static_obstacle_map_cycle_cv_);
  pthread_mutex_unlock(&static_obstacle_map_cycle_mutex_);

  pthread_mutex_unlock(&static_obstacle_points_mutex_);
}

//void* ChsmPlanner::updateStaticObstacleMapThread(void*) {
//  while (!quit_chsm_planner_) {
//
//    pthread_mutex_lock(&static_obstacle_map_cycle_mutex_);
//    pthread_cond_wait(&static_obstacle_map_cycle_cv_, &static_obstacle_map_cycle_mutex_);
//    pthread_mutex_unlock(&static_obstacle_map_cycle_mutex_);
//
//    if(quit_chsm_planner_) {return NULL;}
////    static int for_test=0;
////    if(for_test<10) {for_test++;}
////    else { continue;}
//     double dt = Time::current();
//
//    double cx, cy, cx_utm, cy_utm;
//    pthread_mutex_lock(&pose_mutex_);
//    try {
//    pose_queue_.xyAndUtmXY(static_obstacle_timestamp_, cx, cy, cx_utm, cy_utm);
//    }
//    catch(Exception& e) {
//      pthread_mutex_unlock(&pose_mutex_);
//      std::cout << "Warning: Updating static obstacle map failed because position could not be determined: " << e.what() << std::endl;
//      continue;
//    }
//    pthread_mutex_unlock(&pose_mutex_);
//
//    pthread_mutex_lock(&static_obstacle_map_mutex_);
//    pthread_mutex_lock(&static_obstacle_points_mutex_);
//    staticObstacleMessage2Map(cx, cy);
//    new_static_obstacle_map_ready_ = false;
//    pthread_mutex_unlock(&static_obstacle_points_mutex_);
//    uint8_t gauss_scale_ = 65;  // ok, this was found by trial and error and results in best binary circle approximation
//    static uint8_t* circle_mask=NULL;
//
//    if(!circle_mask) {
////      double rc = traj_eval_->params().safety_width / 2;
//  //    circle_mask_ = new uint8_t[static_obstacle_map_cs_mask_size_*static_obstacle_map_cs_mask_size_];
//  //    memset(circle_mask_, 0, static_obstacle_map_cs_mask_size_*static_obstacle_map_cs_mask_size_);
//      circle_mask = new uint8_t[static_obstacle_map_cs_mask_size_*static_obstacle_map_cs_mask_size_];
//      memset(circle_mask, 0, static_obstacle_map_cs_mask_size_*static_obstacle_map_cs_mask_size_);
//      float center = 0.5*static_obstacle_map_cs_mask_size_;
//      float rc2 = 0.25*static_obstacle_map_cs_mask_size_*static_obstacle_map_cs_mask_size_;
//      for (uint32_t y = 0; y < static_obstacle_map_cs_mask_size_; y++) {
//        for (uint32_t x = 0; x < static_obstacle_map_cs_mask_size_; x++) {
//          float r2 = (x - center) * (x - center) + (y - center) * (y - center);
//          if (r2<=rc2) {circle_mask[y*static_obstacle_map_cs_mask_size_+x]=1;}
//        }
//      }
//    }
//
//memset(static_obstacle_map_, 0, static_obstacle_map_width_*static_obstacle_map_height_);
//IppiSize roi = {static_obstacle_map_cs_mask_size_, static_obstacle_map_cs_mask_size_};
//for(int32_t y=static_obstacle_map_cs_mask_size_; y<static_obstacle_map_height_-static_obstacle_map_cs_mask_size_; y++) {
//      for(int32_t x=static_obstacle_map_cs_mask_size_; x<static_obstacle_map_width_-static_obstacle_map_cs_mask_size_; x++) {
//       if(static_obstacle_map_raw_[y*static_obstacle_map_width_+x]) {
//         ippiCopy_8u_C1R(circle_mask, static_obstacle_map_cs_mask_size_,
//             &static_obstacle_map_[(y-static_obstacle_map_cs_mask_size_/2)*static_obstacle_map_width_+x-static_obstacle_map_cs_mask_size_/2],
//             static_obstacle_map_width_, roi);
//       }
//      }
//    }
//
//    uint8_t gauss_thresh_ = 0;
//    uint8_t* data = static_obstacle_map_;
//    int16_t* data2 = static_obstacle_map_tmp_raw_;
//    uint8_t* data_end = static_obstacle_map_ + static_obstacle_map_width_*static_obstacle_map_height_;
//    for(; data !=data_end; data++, data2++) {
//      if(*data > gauss_thresh_) {
//        *data=OSM_BLOCKED; *data2=32767;
//      }
//      else {
//        *data=OSM_FREE; *data2=0;
//      }
//    }
//
//    static const int32_t derive_kernel[] = {1, 0, -1};
//    static const int32_t derive_kernel_size_ = 3;
//    static const int32_t derive_kernel_anchor_ = derive_kernel_size_ / 2;
//    static const int32_t derive_kernel_coeff_sum_ = 2;
//
//      // create "maybe layer"
//    ippiFilterColumn_16s_C1R(static_obstacle_map_tmp_raw_, static_obstacle_map_width_*sizeof(int16_t),
//                        static_obstacle_map_tmp_h_, static_obstacle_map_width_*sizeof(int16_t),
//                        static_obstacle_map_region_size_, derive_kernel, derive_kernel_size_, derive_kernel_anchor_, derive_kernel_coeff_sum_);
//
//    ippiFilterRow_16s_C1R(static_obstacle_map_tmp_raw_, static_obstacle_map_width_*sizeof(int16_t),
//                        static_obstacle_map_tmp_v_, static_obstacle_map_width_*sizeof(int16_t),
//                        static_obstacle_map_region_size_, derive_kernel, derive_kernel_size_, derive_kernel_anchor_, derive_kernel_coeff_sum_);
//
//
//    int16_t* hdata = static_obstacle_map_tmp_h_;
//    int16_t* vdata = static_obstacle_map_tmp_v_;
//    int16_t* hdata_end = hdata + static_obstacle_map_width_*static_obstacle_map_height_;
//    uint8_t* res = reinterpret_cast<uint8_t*>(static_obstacle_map_tmp_raw_);
//    memset(res, 0, static_obstacle_map_width_*static_obstacle_map_height_);
//    for(; hdata !=hdata_end; hdata++, vdata++, res++) {
//      if( (*hdata || *vdata) && ! *res)  {*res=255;}
//    }
//
//    ippiFilterLowpass_8u_C1R(reinterpret_cast<uint8_t*>(static_obstacle_map_tmp_raw_), static_obstacle_map_width_,
//        static_obstacle_map_tmp_, static_obstacle_map_width_,
//        static_obstacle_map_region_size_, ippMskSize5x5);
//
//    data = static_obstacle_map_tmp_;
//    data_end = data + static_obstacle_map_width_*static_obstacle_map_height_;
//    res = static_obstacle_map_;
//    for(; data !=data_end; data++, res++) {
//      if( *data && ! *res)  {*res=OSM_MAYBE_BLOCKED;}
//    }
//
//
//
//    traj_eval_->SetStaticObstacleMap(static_obstacle_map_, static_obstacle_map_width_, static_obstacle_map_height_,
//                                    static_obstacle_map_res_, cx_utm, cy_utm, static_obstacle_timestamp_);
//
////    char buf[100];
////    static uint32_t frame_num=0;
////    sprintf(buf, "static_map%04d.png", frame_num);
////    printf("%s\n", buf);
////    ilTexImage(static_obstacle_map_width_, static_obstacle_map_height_, 1, 1, IL_LUMINANCE, IL_UNSIGNED_BYTE, static_obstacle_map_raw_);
////    ilSave(IL_PNG, buf);
////    frame_num++;
////    printf("obstacle map update time: %f (rc: %f, res: %f, mask size: %u)\n", Time::current() - dt, rc, static_obstacle_map_res_, msize);
//    printf("obstacle map update time: %f (mask size: %u)\n", Time::current() - dt, static_obstacle_map_cs_mask_size_);
//    pthread_mutex_unlock(&static_obstacle_map_mutex_);
//  }
//
//  return NULL;
//}

void* ChsmPlanner::updateStaticObstacleMapThread(void*) {
  while (!quit_chsm_planner_) {

    pthread_mutex_lock(&static_obstacle_map_cycle_mutex_);
    pthread_cond_wait(&static_obstacle_map_cycle_cv_, &static_obstacle_map_cycle_mutex_);
    pthread_mutex_unlock(&static_obstacle_map_cycle_mutex_);

    if(quit_chsm_planner_) {return NULL;}
//    static int for_test=0;
//    if(for_test<10) {for_test++;}
//    else { continue;}
     double dt = Time::current();

    double cx, cy, cx_utm, cy_utm;
    pthread_mutex_lock(&pose_mutex_);
    try {
    pose_queue_.xyAndUtmXY(static_obstacle_timestamp_, cx, cy, cx_utm, cy_utm);
    }
    catch(Exception& e) {
      pthread_mutex_unlock(&pose_mutex_);
      std::cout << "Warning: Updating static obstacle map failed because position could not be determined: " << e.what() << std::endl;
      continue;
    }
    pthread_mutex_unlock(&pose_mutex_);


    pthread_mutex_lock(&static_obstacle_map_mutex_);
    pthread_mutex_lock(&static_obstacle_points_mutex_);
    staticObstacleMessage2Map(cx, cy);
    new_static_obstacle_map_ready_ = false;
    pthread_mutex_unlock(&static_obstacle_points_mutex_);

    uint8_t gauss_scale_ = 65;  // ok, this was found by trial and error and results in best binary circle approximation

    ippiFilterColumn_8u_C1R(static_obstacle_map_raw_, static_obstacle_map_width_,
                        static_obstacle_map_tmp_, static_obstacle_map_width_,
                        static_obstacle_map_region_size_, gauss_filter_buf_,
                        static_obstacle_map_cs_mask_size_, static_obstacle_map_cs_anchor_,
                        gauss_filter_coeff_sum_/gauss_scale_);

    ippiFilterRow_8u_C1R(static_obstacle_map_tmp_, static_obstacle_map_width_,
                        static_obstacle_map_, static_obstacle_map_width_,
                        static_obstacle_map_region_size_, gauss_filter_buf_,
                        static_obstacle_map_cs_mask_size_, static_obstacle_map_cs_anchor_,
                        gauss_filter_coeff_sum_/gauss_scale_);

    uint8_t gauss_thresh_ = 0;
    uint8_t* data = static_obstacle_map_;
    int16_t* data2 = static_obstacle_map_tmp_raw_;
    uint8_t* data_end = static_obstacle_map_ + static_obstacle_map_width_*static_obstacle_map_height_;
    for(; data !=data_end; data++, data2++) {
      if(*data > gauss_thresh_) {
        *data=OSM_BLOCKED; *data2=32767;
      }
      else {
        *data=OSM_FREE; *data2=0;
      }
    }

    static const int32_t derive_kernel[] = {1, 0, -1};
    static const int32_t derive_kernel_size_ = 3;
    static const int32_t derive_kernel_anchor_ = derive_kernel_size_ / 2;
    static const int32_t derive_kernel_coeff_sum_ = 2;

      // create "maybe layer"
    ippiFilterColumn_16s_C1R(static_obstacle_map_tmp_raw_, static_obstacle_map_width_*sizeof(int16_t),
                        static_obstacle_map_tmp_h_, static_obstacle_map_width_*sizeof(int16_t),
                        static_obstacle_map_region_size_, derive_kernel, derive_kernel_size_, derive_kernel_anchor_, derive_kernel_coeff_sum_);

    ippiFilterRow_16s_C1R(static_obstacle_map_tmp_raw_, static_obstacle_map_width_*sizeof(int16_t),
                        static_obstacle_map_tmp_v_, static_obstacle_map_width_*sizeof(int16_t),
                        static_obstacle_map_region_size_, derive_kernel, derive_kernel_size_, derive_kernel_anchor_, derive_kernel_coeff_sum_);


    int16_t* hdata = static_obstacle_map_tmp_h_;
    int16_t* vdata = static_obstacle_map_tmp_v_;
    int16_t* hdata_end = hdata + static_obstacle_map_width_*static_obstacle_map_height_;
    uint8_t* res = reinterpret_cast<uint8_t*>(static_obstacle_map_tmp_raw_);
    memset(res, 0, static_obstacle_map_width_*static_obstacle_map_height_);
    for(; hdata !=hdata_end; hdata++, vdata++, res++) {
      if( (*hdata || *vdata) && ! *res)  {*res=255;}
    }

    ippiFilterLowpass_8u_C1R(reinterpret_cast<uint8_t*>(static_obstacle_map_tmp_raw_), static_obstacle_map_width_,
        static_obstacle_map_tmp_, static_obstacle_map_width_,
        static_obstacle_map_region_size_, ippMskSize5x5);

    data = static_obstacle_map_tmp_;
    data_end = data + static_obstacle_map_width_*static_obstacle_map_height_;
    res = static_obstacle_map_;
    for(; data !=data_end; data++, res++) {
      if( *data && ! *res)  {*res=OSM_MAYBE_BLOCKED;}
    }

    traj_eval_->SetStaticObstacleMap(static_obstacle_map_, static_obstacle_map_width_, static_obstacle_map_height_,
                                    static_obstacle_map_res_, cx_utm, cy_utm, static_obstacle_timestamp_);

//    char buf[100];
//    static uint32_t frame_num=0;
//    sprintf(buf, "static_map%04d.png", frame_num);
//    printf("%s\n", buf);
//    ilTexImage(static_obstacle_map_width_, static_obstacle_map_height_, 1, 1, IL_LUMINANCE, IL_UNSIGNED_BYTE, static_obstacle_map_raw_);
//    ilSave(IL_PNG, buf);
//    frame_num++;
//    printf("obstacle map update time: %f (rc: %f, res: %f, mask size: %u)\n", Time::current() - dt, rc, static_obstacle_map_res_, msize);
 //   printf("obstacle map update time: %f (mask size: %u)\n", Time::current() - dt, static_obstacle_map_cs_mask_size_);
    pthread_mutex_unlock(&static_obstacle_map_mutex_);
  }

  return NULL;
}
  void ChsmPlanner::updateVehicles(PerceptionDynamicObstacle* dyn_obstacles,
                                  uint32_t num_dyn_obstacles, double t0) {
  double offset_x, offset_y;
  pthread_mutex_lock(&pose_mutex_);
  try {
    pose_queue_.offsets(t0, offset_x, offset_y);
  }
  catch(Exception& e) {
    pthread_mutex_unlock(&pose_mutex_);
    std::cout << "Warning: Updating dynamic objects (e.g. vehicles) failed because position could not be determined: " << e.what() << std::endl;
    return;
  }
  pthread_mutex_unlock(&pose_mutex_);

  pthread_mutex_lock(&dyn_obstacles_mutex_);

  pthread_mutex_lock(&topology_mutex_);
  vehicle_manager->updateVehicles(dyn_obstacles, num_dyn_obstacles, offset_x, offset_y, t0, traj_eval_->params().checked_horizon, traj_eval_->params().time_sample_res);

  vehicle_manager->updateBlockages();


  predicted_pedestrians_.clear();

  pthread_mutex_unlock(&topology_mutex_);

  pthread_mutex_lock(&pedestrian_prediction_mutex_);
  MovingBox mb;
  ObstaclePrediction op;
  for(uint32_t i=0; i<num_dyn_obstacles; i++) {
    if(dyn_obstacles[i].obstacleType != OBSTACLE_PEDESTRIAN) {continue;}
    double x_step = dyn_obstacles[i].velocity * cos(dyn_obstacles[i].direction);
    double y_step = dyn_obstacles[i].velocity * sin(dyn_obstacles[i].direction);

    mb.length       = dyn_obstacles[i].length;
    mb.width        = dyn_obstacles[i].width;
    mb.ref_offset   = dyn_obstacles[i].length*0.5;
    for(double t=0; t<traj_eval_->params().checked_horizon; t+=traj_eval_->params().time_sample_res) {
      mb.t            = t0 + t;
      mb.x            = dyn_obstacles[i].x + t*x_step + offset_x;
      mb.y            = dyn_obstacles[i].y + t*y_step + offset_y;
//      mb.psi          = (dyn_obstacles[i].velocity >=0 ? dyn_obstacles[i].direction : dyn_obstacles[i].direction-M_PI); // TODO: velocity is not directly part of prediction
      mb.psi          = atan2(y_step, x_step);
      op.predicted_traj_.push_back(mb);
    }

    predicted_pedestrians_.push_back(op);
    op.predicted_traj_.clear();
  }
  pthread_mutex_unlock(&pedestrian_prediction_mutex_);
  //  std::map<int, Vehicle>::const_iterator vit = vehicle_manager->vehicle_map.begin(),  vit_end = vehicle_manager->vehicle_map.end();
  //  for(; vit!=vit_end; vit++) {
  //      const Vehicle& v = (*vit).second;
  //      printf(" Found vehicle (w: %f, l: %f) @ (%f, %f):\n matched on lane %s (%f, %f; dist from start: %f\n",
  //              v.width(), v.length(), v.xMatchedFrom(), v.yMatchedFrom(), v.edge()->name().c_str(), v.xMatched(), v.yMatched(), v.distFromStart());
  //      if(!v.edge()->leftEdges.empty()) {
  //          TRndfEdgeSet::const_iterator eit=v.edge()->leftEdges.begin(), eit_end=v.edge()->leftEdges.end();
  //          for(; eit!=eit_end; eit++){
  //              printf("lane has left neighbor %s\n", (*eit)->name().c_str());
  //          }
  //      }
  //      if(!v.edge()->rightEdges.empty()) {
  //          TRndfEdgeSet::const_iterator eit=v.edge()->rightEdges.begin(), eit_end=v.edge()->rightEdges.end();
  //          for(; eit!=eit_end; eit++){
  //              printf("lane has right neighbor %s\n", (*eit)->name().c_str());
  //          }
  //      }
  //      if(!v.edge()->leftOppositeEdges.empty()) {
  //          TRndfEdgeSet::const_iterator eit=v.edge()->leftOppositeEdges.begin(), eit_end=v.edge()->leftOppositeEdges.end();
  //          for(; eit!=eit_end; eit++){
  //              printf("lane has oncoming neighbor %s\n", (*eit)->name().c_str());
  //          }
  //      }
  //  }

  //  std::map<int, Vehicle>::const_iterator vit =
  //      vehicle_manager->vehicle_map.begin(), vit_end = vehicle_manager->vehicle_map.end();
  //  for(; vit != vit_end; vit++) {
  //    const Vehicle& veh = (*vit).second;
  //    predictor_->predict(veh);
  //  }

  pthread_mutex_unlock(&dyn_obstacles_mutex_);
}

  RoutePlanner::RndfEdge* ChsmPlanner::bestPredictedEdge(const std::map<RoutePlanner::RndfEdge*, double>& edges, double& dist_to_end) {
    if(edges.empty()) {return NULL;}
    std::map<RoutePlanner::RndfEdge*, double>::const_iterator eit = edges.begin(), eit_end = edges.end();
    for(; eit!=eit_end; eit++) {
      if(!(*eit).first->isVirtualEdge()) {
        dist_to_end = (*eit).first->getLength() - (*eit).second;
        return eit->first;
      }
    }

      // only virtual edges available, pick first one
    dist_to_end = (*edges.begin()).first->getLength() - (*edges.begin()).second;
    return edges.begin()->first;
  }

  RoutePlanner::RndfEdge* ChsmPlanner::bestPredictedEdge(const RoutePlanner::TRndfEdgeSet& edges) {
  if(edges.empty()) {return NULL;}
  RoutePlanner::TRndfEdgeSet::const_iterator eit = edges.begin(), eit_end = edges.end();
  for(; eit!=eit_end; eit++) {
    if(!(*eit)->isVirtualEdge()) {return *eit;}
  }

    // only virtual edges available, pick first one
  return *edges.begin();
}

void ChsmPlanner::detectBlockades() {
  printf("%s: reimplement...\n", __FUNCTION__);
//  const float thres_critical_zdif = 0.3;
//  const double borderDist = 1.8;
//  const int thres_cirticalN = 14;
//  const double back_sampl_dist = 0.0;
//  const double length = 30.0;
//  const int point_anz = 20;
//  int current_curvepoint=0;
//  double latDist, lonDist, lonOffset;
//  BlockadeInfo blockade_info;
//  bool valid;
//  CurvePoints sampled_curvepoints;
//  static CurvePoints stripped_curvepoints[1000];
//  std::vector<BlockadeInfo> blockade_info_list;
//  std::vector<CurvePoints*> curvepoints_list;
//  std::vector<RndfEdge*> edge_list;
//
//  if( !centered_obstacle_map ) return;
//  dgc_pose_t pose = centered_obstacle_map->robot_pose;
//
//  blockade_info.thres_critical_zdif = thres_critical_zdif;
//  blockade_info.borderDist = borderDist;
//  blockade_info.thres_cirticalN = thres_cirticalN;
//
//  // get current edge
//  RndfEdge* current_edge = (*topology->current_edge_it)->getEdge();
//
//  // get opposite edge
//  const TRndfEdgeSet& opposite_edges = current_edge->getLeftOppositeEdges();
//  if(opposite_edges.size()==0) return;
//  RndfEdge* opposite_edge = (*opposite_edges.begin());
//
//  // calculate offset to opposite edge
//  RndfVertex* v = opposite_edge->fromVertex();
//  current_edge->computeDistance(v->x(), v->y(), latDist, lonDist, lonOffset);
//  double sample_offset = fabs(latDist)/2.0;
//
//  // sample curvepoints in the middle between current edge and opposite edge
//  valid = route_sampler->samplePoints(&sampled_curvepoints, back_sampl_dist, length, point_anz, sample_offset);
//
//  if(valid && sampled_curvepoints.numberValidPoints > 2) {
//    // strip curvepoints for current lane
//    for(int i=0;i<sampled_curvepoints.numberValidPoints-2;++i) {
//      CurvePoints* cp = &stripped_curvepoints[current_curvepoint];
//      cp->numberValidPoints=3;
//      cp->curvepoints[0] = sampled_curvepoints.curvepoints[i];
//      cp->curvepoints[1] = sampled_curvepoints.curvepoints[i+1];
//      cp->curvepoints[2] = sampled_curvepoints.curvepoints[i+2];
//      blockade_info_list.push_back(blockade_info);
//      curvepoints_list.push_back(cp);
//      current_curvepoint++;
//    }
//  }
//
//  // do we have valid curvepoints?
//  if (curvepoints_list.size() == 0) return;
//
////  // check blockages
////  pTMaster->checkForBlockades(blockade_info_list, pose.x, pose.y, pose.yaw, curvepoints_list,centered_obstacle_map);
//
//  // add blocked edges to blockage manager
//  for(int i=0;i<current_curvepoint;++i) {
//    //fprintf(stderr,"[BM] id %d na %d av %f \n",i,blockade_info_list[i].nAboveThreshold,blockade_info_list[i].averageValue);
//
//    if(blockade_info_list[i].bVerificationWasSuccessfull &&
//       blockade_info_list[i].bBlocked) {
//      //fprintf(stderr,"[BM] blocked!!!!\n",blockade_info_list[i].averageValue);
//      CurvePoints* cp = curvepoints_list[i];
//      // get edge for blocked curvepoints
//      double x1 = cp->curvepoints[1].x;
//      double y1 = cp->curvepoints[1].y;
//      RndfEdge* edge = topology->complete_graph->findClosestEdge(x1,y1);
//      if(!edge) continue;
//      if(!edge->isLaneEdge()) continue;
//
//      // check if there is a vehicle on this edge
//      std::map<int, Vehicle>& vehicle_map = vehicle_manager->vehicle_map;
//      std::map<int, Vehicle>::iterator itv, itv_end;
//      bool vehicle_close = false;
//      for(itv=vehicle_map.begin(),itv_end=vehicle_map.end();itv!=itv_end;++itv) {
//        Vehicle& vehicle = itv->second;
//        double dist = hypot(x1-vehicle.x_matched_from,y1-vehicle.y_matched_from);
//
//        if(edge==vehicle.edge || dist < 10.0) {
//          vehicle_close = true;
//          break;
//        }
//      }
//
//      if(vehicle_close) continue;
//
//      // this lane is blocked -> check if blockade already exists
//      BlockadeManager::Blockade* blockade = blockade_manager->getBlockadeByEdge(edge);
//      if(blockade) {
//        // update blockade
//        blockade->update();
//      }
//      else {
//        // add new blockade
//        blockade_manager->addBlockade(edge);
//      }
//
//      // update left opposites edges
//      if(edge->getLeftOppositeEdges().size()>0) {
//      RndfEdge* edge2 = (*edge->getLeftOppositeEdges().begin());
//      BlockadeManager::Blockade* blockade = blockade_manager->getBlockadeByEdge(edge2);
//        if(blockade) {
//          // update blockade
//          blockade->update();
//        }
//        else {
//          // add new blockade
//          blockade_manager->addBlockade(edge2);
//        }
//      }
//    }
//  }
}

void ChsmPlanner::updateBlockades() {

  // detect new blockades
  if(!inPause && !bIntersection && params().enable_blockade_detection) detectBlockades();
  // update blockade manager
  blockade_manager->update();
}

void ChsmPlanner::clearBlockages()
{
  topology->complete_graph->clearBlockades();
}

void ChsmPlanner::enable_road_boundaries( bool enabled )
{
  road_boundaries_enabled = enabled;
  // setup perimeter for cspace, take them from lane boundaries
  NavigatorPerimeter& navigator_perimeter_ = navigator_perimeter;

  if( !enabled )
    {
      navigator_perimeter_.num_points = 0;
      return;
    }

  navigator_perimeter_.entry_present = false;
  navigator_perimeter_.exit_present = false;

  navigator_perimeter_.num_points = 0;
  for( std::vector<kogmo_line_2Df_t>::iterator it = road_boundaries.begin(); it!= road_boundaries.end(); it++ )
    {
      if( (uint32_t)(navigator_perimeter_.num_points) > NAVIGATOR_PERIMETER_MAX_POINTS )
  {
    std::cout << "WAAARNING! NavigatorPerimeter::MAX_POINTS exceeded! This is very bad!\n";
    continue;
  }
      navigator_perimeter_.points[ navigator_perimeter_.num_points ].x = it->x1;
      navigator_perimeter_.points[ navigator_perimeter_.num_points ].y = it->y1;
      navigator_perimeter_.num_points++;

      navigator_perimeter_.points[ navigator_perimeter_.num_points ].x = it->x2;
      navigator_perimeter_.points[ navigator_perimeter_.num_points ].y = it->y2;
      navigator_perimeter_.num_points++;
    }

//?!?  ipc_->Publish(AWNavigatorPerimeterID, &navigator_perimeter);
}

void ChsmPlanner::createGaussFilterMask() {
  gauss_filter_buf_ = new int32_t[static_obstacle_map_cs_mask_size_+1];

  uint32_t val1=1, val2;
  for (uint32_t i = 0; i <=static_obstacle_map_cs_mask_size_; i++) {
    for (uint32_t j = 1; j < i; j++) {
      if (j > 1) {
        val2 = gauss_filter_buf_[j - 1] + gauss_filter_buf_[j];
        gauss_filter_buf_[j - 1] = val1;
        val1 = val2;
      }
      else {
        val1 = gauss_filter_buf_[j - 1] + gauss_filter_buf_[j];
      }
    }

    gauss_filter_buf_[i] = 1;
  }

  for (uint32_t i = 0; i <static_obstacle_map_cs_mask_size_; i++) {gauss_filter_coeff_sum_+=gauss_filter_buf_[i];}
}

void ChsmPlanner::staticObstacleMessage2Map(double cx, double cy) {
  memset(static_obstacle_map_raw_, 0, static_obstacle_map_width_ * static_obstacle_map_height_ * sizeof(uint8_t));

  static_obstacle_map_cx_ = cx;
  static_obstacle_map_cy_ = cy;

  for (uint32_t i = 0; i < uint32_t(static_obstacle_num_points_); i++) {
    int32_t xi = static_obstacle_map_width_ / 2 - (int32_t) ((static_obstacle_points_[i].x - cx) / static_obstacle_map_res_+.5);
    int32_t yi = static_obstacle_map_height_ / 2 - (int32_t) ((static_obstacle_points_[i].y - cy) / static_obstacle_map_res_+.5);
    if (xi >= 0 && xi < int32_t(static_obstacle_map_width_) && yi >= 0 && yi < int32_t(static_obstacle_map_height_)) {
      uint8_t val = static_obstacle_points_[i].type;

      //        if (val == PERCEPTION_MAP_OBSTACLE_LOW || val == PERCEPTION_MAP_OBSTACLE_HIGH || val
      //            == PERCEPTION_MAP_OBSTACLE_UNKNOWN) { // val == PERCEPTION_MAP_OBSTACLE_DYNAMIC) {
      if (val == PERCEPTION_MAP_OBSTACLE_FREE) {
        val = 255;
        static_obstacle_map_raw_[yi * static_obstacle_map_width_ + xi] = val;
      }
    }
  }
}

bool ChsmPlanner::obstacleMapMsgData(int32_t& num_points, double& last_timestamp, PerceptionObstaclePoint& points, double& cx, double& cy) {
  if(last_timestamp >= static_obstacle_timestamp_) {return false;}

  if(static_obstacle_num_points_ > num_points) {
    throw Exception("Not enough memory to copy static obstacle map data.");
  }

  pthread_mutex_lock(&static_obstacle_points_mutex_);

  last_timestamp = static_obstacle_timestamp_;
  num_points = static_obstacle_num_points_;
  if(num_points==0) {
    pthread_mutex_unlock(&static_obstacle_points_mutex_);
    return true;
  }
  memcpy(&points, static_obstacle_points_, num_points*sizeof(PerceptionObstaclePoint));
  cx = static_obstacle_map_cx_;
  cy = static_obstacle_map_cy_;
  pthread_mutex_unlock(&static_obstacle_points_mutex_);
  return true;
}

void ChsmPlanner::offset_line( double x1, double y1, double x2, double y2, double& x1o, double& y1o, double& x2o, double& y2o, double offset )
{
  double dx = x2 - x1;
  double dy = y2 - y1;

  double norm = sqrt( dx*dx + dy*dy );

  double n1 = -dy / norm;
  double n2 =  dx / norm;

  x1o = x1 + n1 * offset;
  y1o = y1 + n2 * offset;
  x2o = x2 + n1 * offset;
  y2o = y2 + n2 * offset;

}

} // namespace vlr
