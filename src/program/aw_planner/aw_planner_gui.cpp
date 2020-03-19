#include <iostream>
#include <cmath>

#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glext.h>

#include <IL/il.h>
#include <imagery.h>
#include <passatmodel.h>
#include <gui3D.h>
#include <gl_support.h>
#include <textures.h>
#include <imagery.h>
#include <aw_roadNetwork.h>
#include <aw_ChsmPlanner.hpp>
#include <aw_StLaneChange.hpp>
#include "aw_planner_gui.h"

using namespace dgc;
using std::cout;
using std::endl;
using std::flush;

namespace vlr {

AWRoadPlannerGUI* AWRoadPlannerGUI::instance_ = NULL;

AWRoadPlannerGUI* AWRoadPlannerGUI::instance(AWRoadPlanner& awp, int argc, char** argv) {
  if(!instance_) {instance_ = new AWRoadPlannerGUI(awp, argc, argv);}
  return instance_;
}

AWRoadPlannerGUI* AWRoadPlannerGUI::instance() {
  if(!instance_) {
    std::cout << " GUI not instantiated.\n";
    return NULL;
    }
  return instance_;
}

AWRoadPlannerGUI::AWRoadPlannerGUI(AWRoadPlanner& awp, int argc, char** argv) : awp_(awp), car_(NULL),
    configuration_space_texture_(0), obstacle_map_texture_(0),
    create_rndf_display_list_(true), last_raw_map_timestamp_(0), last_configuration_space_timestamp_(0),
    smoothed_mission_points_buf_(NULL), smoothed_mission_points_buf_size_(0) {

  show_topology_ = 1;
  show_rndf_ = 1;
  show_vehicle_ = 1;
  show_imagery_ = 1;
  show_pose_history_ = 0;
  show_center_line_ = 0;
  show_complete_center_line_ = 1;
  show_trajectories_ = 0;
  show_best_trajectory_ = 1;
  show_complete_graph_ = 0;
  show_mission_graph_ = 1;
  show_matched_edge_ = 0;
  show_yellow_dot_ = 0;
  show_dynamic_objects_ = 1;
  show_complete_mission_graph_ = 0;
  show_obstacle_destinations_ = 1;
  show_obstacle_map_ = 1;
  show_configuration_space_ = 1;
  show_radius_ = 0;
  show_pose_accuracy_ = 0;
  show_ego_distances_ = 0;
  show_veh_distances_ = 0;
  show_intersection_merging_point_ = 1;
  show_lanchange_merging_points_ = 0;
  show_obstacle_predictions_ = 1;

  sphere_= gluNewQuadric();

  gui3D_initialize(argc, argv, 10, 10, 800, 600, 30.0);
  gui3D_setInitialCameraPos(180.0, 89.99, 100.0, 0, 0, 0);
  gui3D_setCameraParams(0.01, 0.3, 0.001, 0.009, 60, 0.4, 20000000);
  car_ = passatwagonmodel_load(0.0, 0.0, 0.5, 1.0);
//  car_ = passatwagonmodel_load(0.5, 0.5, 0.5, 1.0);
  porsche_ = new dgc_porsche_model;
  porsche_->Init();
  hummer_ = new dgc_hummer_model;
  hummer_->Init();
  elise_ = new dgc_elise_model;
  elise_->Init();

  gui3D_set_displayFunc(displayCBWrapper);
  gui3D_set_keyboardFunc(keyboardCBWrapper);

  glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);

  initializeGLUIControls();

  glGenTextures(1, &obstacle_map_texture_);

  const rndf::RoadNetwork& rn = awp_.chsm_planner_->topology->roadNetwork();
  coordinate_latlon_t rndf_center_latlon = const_cast<rndf::RoadNetwork*>(&rn)->center();
  latLongToUtm(rndf_center_latlon.lat, rndf_center_latlon.lon, &rndf_center_.x, &rndf_center_.y, rndf_center_.zone);

  dgc_imagery_set_imagery_type(DGC_IMAGERY_TYPE_LASER);
  //rndf_display_list_ = glGenLists(1);
  //glNewList(rndf_display_list_, GL_COMPILE);
  //rndf_dl_origin_x_ = rn.PointMap().begin()->second->utm_x();
  //rndf_dl_origin_y_ = rn.PointMap().begin()->second->utm_y();
  //rn.draw(rndf_dl_origin_x_, rndf_dl_origin_y_, 1, true);
  //glEndList();

  double blend = 0.5;
  bool dynamic = false;
  generate_rndf_display_list(rn, blend, dynamic, rndf_display_list2_);
}

void AWRoadPlannerGUI::run() {
  gui3D_mainloop();
}

void AWRoadPlannerGUI::keyboard(unsigned char key) {
  switch (key) {
//  case 27:
//  case 'q':
//  case 'Q':
//    break;
  case 'i':
  case 'I':
    show_imagery_ = 1 - show_imagery_;
    break;
  case 'm':
    show_obstacle_map_ = 1 - show_obstacle_map_;
    break;
  case 'n':
  case 'N':
    show_rndf_ = 1 - show_rndf_;
    break;
  case 'o':
  case 'O':
    show_dynamic_objects_ = 1 - show_dynamic_objects_;
    break;
  case 't':
    show_best_trajectory_ = 1 - show_best_trajectory_;
    break;
  case 'T':
    show_trajectories_ = 1 - show_trajectories_;
    break;
  case 'r':
  case 'R':
    show_radius_ = 1 - show_radius_;
    break;
  case 'h':
    show_pose_history_ = 1 - show_pose_history_;
    break;
  case 'H':
    awp_.pose_history_.clear();
    break;

  case 'b':
  case 'B':
    if(awp_.demo_mode_ && awp_.demo_) {
      for(uint32_t i=0; i<static_cast<CircleDemo*>(awp_.demo_)->getCarStates().size(); i++) {
        printf("braking...\n");
        static_cast<CircleDemo*>(awp_.demo_)->getCarStates()[i].v = 0;
      }
    }
    break;

  case 'v':
    if(awp_.demo_mode_ && awp_.demo_) {
      for(uint32_t i=0; i<static_cast<CircleDemo*>(awp_.demo_)->getCarStates().size(); i++) {
        printf("slowing down...\n");
        static_cast<CircleDemo*>(awp_.demo_)->getCarStates()[i].v -= .5;
      }
    }
    break;

  case 'V':
    if(awp_.demo_mode_ && awp_.demo_) {
      for(uint32_t i=0; i<static_cast<CircleDemo*>(awp_.demo_)->getCarStates().size(); i++) {
        printf("speeding up...\n");
        static_cast<CircleDemo*>(awp_.demo_)->getCarStates()[i].v += .5;
      }
    }
    break;

  default:
    break;
  }
  glui_->sync_live();
  gui3D_forceRedraw();
}

void AWRoadPlannerGUI::drawPoseHistory(double center_x, double center_y) {
  glLineWidth(4.0);

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_LINE_SMOOTH);
  glColor3f(0.6, 0.0, 0.0);
  glBegin(GL_LINES);
  int num_poses = (int)awp_.pose_history_.size();
  for (int i=0; i<num_poses-1; ++i) {
    glVertex3f(awp_.pose_history_[i].x - center_x, awp_.pose_history_[i].y - center_y, 0);
    glVertex3f(awp_.pose_history_[i+1].x - center_x, awp_.pose_history_[i+1].y - center_y, 0);
  }
  glEnd();

  glDisable(GL_LINE_SMOOTH);
  glDisable(GL_BLEND);
}

//void draw_corrected_trajectory(double center_x, double center_y) {
//  glLineWidth(4.0);
//
//  glEnable(GL_BLEND);
//  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//  glEnable(GL_LINE_SMOOTH);
//  glColor3f(0.6, 0.6, 0.0);
//  glBegin(GL_LINES);
//  int num_poses = (int)corrected_robot_pose_trajectory.size();
//  for (int i=0; i<num_poses-1; ++i) {
//    glVertex3f(corrected_robot_pose_trajectory[i ].x - center_x, corrected_robot_pose_trajectory[i ].y - center_y, 0);
//    glVertex3f(corrected_robot_pose_trajectory[i+1].x - center_x, corrected_robot_pose_trajectory[i+1].y - center_y, 0);
//  }
//  glEnd();
//
//  glDisable(GL_LINE_SMOOTH);
//  glDisable(GL_BLEND);
//}

//void AWRoadPlannerGUI::drawTrajectories(Pose& pose) {
//  std::multiset<PolyTraj2D>::const_iterator ptit, ptit_end;
//  uint32_t set_size;
//  switch(awp_.chsm_planner_->traj_eval_->trajectoryMode()) {
//    case TrajectoryEvaluator::TRAJ_MODE_VELOCITY:
//      ptit = awp_.chsm_planner_->traj_eval_->getLVSet2D().set_data_.begin();
//      ptit_end = awp_.chsm_planner_->traj_eval_->getLVSet2D().set_data_.end();
//      set_size = awp_.chsm_planner_->traj_eval_->getLVSet2D().set_data_.size();
//      break;
//
//    case TrajectoryEvaluator::TRAJ_MODE_FOLLOW:
//      ptit = awp_.chsm_planner_->traj_eval_->getFollowingSet2D().set_data_.begin();
//      ptit_end = awp_.chsm_planner_->traj_eval_->getFollowingSet2D().set_data_.end();
//      set_size = awp_.chsm_planner_->traj_eval_->getFollowingSet2D().set_data_.size();
//      break;
//
//    case TrajectoryEvaluator::TRAJ_MODE_STOP:
//      ptit = awp_.chsm_planner_->traj_eval_->stopSet2D().set_data_.begin();
//      ptit_end = awp_.chsm_planner_->traj_eval_->stopSet2D().set_data_.end();
//      set_size = awp_.chsm_planner_->traj_eval_->stopSet2D().set_data_.size();
//      break;
//
//    default:
//      return;
//  }
//
//  glMatrixMode(GL_MODELVIEW);
//  glPushMatrix();
////  glTranslated(-pose.x, -pose.y, 0.); // Don't do that with utm coordinates !!
//  glEnable(GL_BLEND);
//  glEnable(GL_DEPTH_TEST);
////  glDisable(GL_DEPTH_TEST);
//
//  for (unsigned int j = 0; ptit != ptit_end; ptit++) {
//    std::vector<TrajectoryPoint2D> trj = (*ptit).trajectory2D_;
//
//    int col_index = 255 - 255 * j / (double) (set_size-1);
//    //      int col_index = (int(256*j/(double)lane_and_velocity_set.set_data_.size()+70))%256;
//    glColor4f(cmap_rb2_red_[col_index], cmap_rb2_green_[col_index], cmap_rb2_blue_[col_index], .15);
//    //      glColor4f(cmap_rb1_red_[col_index], cmap_rb1_green_[col_index], cmap_rb1_blue_[col_index], .65);
//    glLineWidth(2);
//    glBegin(GL_LINE_STRIP);
//    for (std::vector<TrajectoryPoint2D>::const_iterator tit=trj.begin(); tit != trj.end(); tit++) {
////      glVertex3d((*tit).x, (*tit).y, (*tit).v);
//      glVertex3f((*tit).x-pose.utmX(), (*tit).y-pose.utmY(), (*tit).v);
//    }
//    glEnd();
//    j++;
//  }
//
//glDisable(GL_BLEND);
////glEnable(GL_DEPTH_TEST);
//
//glPopMatrix();
//}

void AWRoadPlannerGUI::drawTrajectories(Pose& pose) {
  std::multiset<PolyTraj2D>::const_iterator ptit, ptit_end;
  uint32_t set_size;
//  switch(awp_.chsm_planner_->traj_eval_->trajectoryMode()) {
//    case TrajectoryEvaluator::TRAJ_MODE_VELOCITY:
//      ptit = awp_.chsm_planner_->traj_eval_->getLVSet2D().set_data_.begin();
//      ptit_end = awp_.chsm_planner_->traj_eval_->getLVSet2D().set_data_.end();
//      set_size = awp_.chsm_planner_->traj_eval_->getLVSet2D().set_data_.size();
//      break;
//
//    case TrajectoryEvaluator::TRAJ_MODE_FOLLOW:
//      ptit = awp_.chsm_planner_->traj_eval_->getFollowingSet2D().set_data_.begin();
//      ptit_end = awp_.chsm_planner_->traj_eval_->getFollowingSet2D().set_data_.end();
//      set_size = awp_.chsm_planner_->traj_eval_->getFollowingSet2D().set_data_.size();
//      break;
//
//    case TrajectoryEvaluator::TRAJ_MODE_STOP:
//      ptit = awp_.chsm_planner_->traj_eval_->stopSet2D().set_data_.begin();
//      ptit_end = awp_.chsm_planner_->traj_eval_->stopSet2D().set_data_.end();
//      set_size = awp_.chsm_planner_->traj_eval_->stopSet2D().set_data_.size();
//      break;
//
//    default:
//      return;
//  }

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
//  glTranslated(-pose.x, -pose.y, 0.); // Don't do that with utm coordinates !!
  glEnable(GL_BLEND);
  glEnable(GL_DEPTH_TEST);
//  glDisable(GL_DEPTH_TEST);

//  ptit = awp_.chsm_planner_->traj_eval_->getLVSet2D().set_data_.begin();
//  ptit_end = awp_.chsm_planner_->traj_eval_->getLVSet2D().set_data_.end();
//  set_size = awp_.chsm_planner_->traj_eval_->getLVSet2D().set_data_.size();
//  for (unsigned int j = 0; ptit != ptit_end; ptit++) {
//    std::vector<TrajectoryPoint2D> trj = (*ptit).trajectory2D_;
//    TrajectoryPoint2D& last_tp = *(--trj.end());
//
//    int col_index = 255 - 255 * j / (double) (set_size-1);
//    glColor4f(cmap_rb2_red_[col_index], cmap_rb2_green_[col_index], cmap_rb2_blue_[col_index], .25);
//    glLineWidth(2);
//    glBegin(GL_LINE_STRIP);
//    for (std::vector<TrajectoryPoint2D>::const_iterator tit=trj.begin(); tit != trj.end(); tit++) {
//      glVertex3f((*tit).x-pose.utmX(), (*tit).y-pose.utmY(), (*tit).v);
//    }
//    glEnd();
//    j++;
//    glColor3f(.5,.9,.5);
//    draw_circle(last_tp.x-pose.utmX(), last_tp.y-pose.utmY(), .05, 1);
//  }

  ptit = awp_.chsm_planner_->traj_eval_->stopSet2D().set_data_.begin();
  ptit_end = awp_.chsm_planner_->traj_eval_->stopSet2D().set_data_.end();
  set_size = awp_.chsm_planner_->traj_eval_->stopSet2D().set_data_.size();
  for (unsigned int j = 0; ptit != ptit_end; ptit++) {
    std::vector<TrajectoryPoint2D> trj = (*ptit).trajectory2D_;
    TrajectoryPoint2D& last_tp = *(--trj.end());

    int col_index = 255 - 255 * j / (double) (set_size-1);
    glColor4f(cmap_rb2_red_[col_index], cmap_rb2_green_[col_index], cmap_rb2_blue_[col_index], .25);
    glLineWidth(2);
    glBegin(GL_LINE_STRIP);
    for (std::vector<TrajectoryPoint2D>::const_iterator tit=trj.begin(); tit != trj.end(); tit++) {
      glVertex3f((*tit).x-pose.utmX(), (*tit).y-pose.utmY(), (*tit).v);
    }
    glEnd();
    j++;
    glColor3f(.5,.9,.5);
    draw_circle(last_tp.x-pose.utmX(), last_tp.y-pose.utmY(), .05, 1);
  }

glDisable(GL_BLEND);
//glEnable(GL_DEPTH_TEST);

glPopMatrix();
}


void AWRoadPlannerGUI::drawBestTrajectory(Pose& pose) {
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glEnable(GL_BLEND);
//  glDisable(GL_DEPTH_TEST);

  const std::vector<TrajectoryPoint2D>& trj =  awp_.chsm_planner_->traj_eval_->bestTrajectory();
 glLineWidth(2);
  glColor4f(0.2, 1, 0.3, 1);
//  glColor4f(1, 1, 1, 1);
    glBegin(GL_LINE_STRIP);
    for (std::vector<TrajectoryPoint2D>::const_iterator tit=trj.begin(); tit != trj.end(); tit++) {
      glVertex3f((*tit).x-pose.utmX(), (*tit).y-pose.utmY(), (*tit).v);
    }
    glEnd();

//    for (std::vector<TrajectoryPoint2D>::const_iterator tit=trj.begin(); tit != trj.end(); tit++) {
//        glPushMatrix();
//        glColor4f(0.2, .3, 1, 1);
//        glTranslatef((*tit).x-pose.x, (*tit).y-pose.y, (*tit).v);
//        gluSphere(sphere_, .2, 16, 16);
//        glPopMatrix();
//    }
glDisable(GL_BLEND);
//  glEnable(GL_DEPTH_TEST);

//drawBestTrajectory_();
glPopMatrix();
}

  // TODO: fix cutting off of first/last 2 points
void AWRoadPlannerGUI::drawCenterline(std::map<double, CurvePoint>& center_line, double center_x, double center_y) {
  if(center_line.empty()) {return;}
  glDisable(GL_DEPTH_TEST);

  // Draw line between center line points
  glLineWidth(3.0);
  glEnable(GL_LINE_SMOOTH);
  glColor3f(0.6, 0.0, 0.0);
  glBegin(GL_LINE_STRIP);
  std::map<double, CurvePoint>::const_iterator clit = center_line.begin();
  std::map<double, CurvePoint>::const_iterator clit_end = --center_line.end();

  for(; clit != clit_end; clit++) {
    glVertex3f((*clit).second.x - center_x, (*clit).second.y - center_y, 0.0);
  }
  glEnd();
  glDisable(GL_LINE_SMOOTH);

  // Draw cones to show orientation, color represents curvature
  glPointSize(8.0);
  clit = center_line.begin();

  for(; clit != clit_end; clit++) {
    uint32_t col_index = std::min(255., 255*std::abs((*clit).second.kappa/0.2));
    glColor3f(cmap_rb2_red_[col_index], cmap_rb2_green_[col_index], cmap_rb2_blue_[col_index]);
    glPushMatrix();
    glTranslatef((*clit).second.x - center_x, (*clit).second.y - center_y, 0.0);
    glRotatef(  90.0 , 0., 1., 0.);
    glRotatef(  -dgc_r2d((*clit).second.theta), 1., 0., 0. );
    glutSolidCone(0.4, 1.1, 6, 1);
    glPopMatrix();
  }

  glEnable(GL_DEPTH_TEST);
}

void AWRoadPlannerGUI::drawCompleteCenterLine(const std::vector<CurvePoint>& mission_line_bez, const std::vector<CurvePoint>& mission_line, double center_x, double center_y) {
  if(mission_line.empty()) {return;}
  glDisable(GL_DEPTH_TEST);

  if(smoothed_mission_points_buf_size_<mission_line_bez.size()) {
    if(smoothed_mission_points_buf_) {delete[] smoothed_mission_points_buf_;}
    smoothed_mission_points_buf_ = new double[3*mission_line_bez.size()];
    smoothed_mission_points_buf_size_ = mission_line_bez.size();
  }


  for(uint32_t i=0, i3=0; i<mission_line_bez.size(); i++) {
    smoothed_mission_points_buf_[i3++]=mission_line_bez[i].x - center_x;
    smoothed_mission_points_buf_[i3++]=mission_line_bez[i].y - center_y;
    smoothed_mission_points_buf_[i3++]=0;
  }

  glEnable(GL_MAP1_VERTEX_3);

  // Draw line between center line points
  glLineWidth(3.0);
  glEnable(GL_LINE_SMOOTH);
  glColor3f(0.6, 0.0, 0.0);
  for (uint32_t segments = 0; segments < mission_line_bez.size()-1; segments += 3) {
    glMap1d(GL_MAP1_VERTEX_3, 0.0, 1.0, 3, 4, &smoothed_mission_points_buf_[3*segments]);
    glBegin(GL_LINE_STRIP);
    for (uint32_t i = 0; i <= 30; i++) {
      glEvalCoord1d((GLdouble) i / 30.0);
    }
    glEnd();
  }
  glDisable(GL_LINE_SMOOTH);
  glDisable(GL_MAP1_VERTEX_3);

  // Draw cones to show orientation, color represents curvature
  glPointSize(8.0);
  std::vector<CurvePoint>::const_iterator clit = mission_line.begin();
  std::vector<CurvePoint>::const_iterator clit_end = --mission_line.end();

  for(; clit != clit_end; clit++) {
    uint32_t col_index = std::min(255., 255*std::abs((*clit).kappa/0.2));
    glColor3f(cmap_rb2_red_[col_index], cmap_rb2_green_[col_index], cmap_rb2_blue_[col_index]);
    glPushMatrix();
    glTranslatef((*clit).x - center_x, (*clit).y - center_y, 0.0);
    glRotatef(  90.0 , 0., 1., 0.);
    glRotatef(  -dgc_r2d((*clit).theta), 1., 0., 0. );
    glutSolidCone(0.4, 1.1, 6, 1);
    glPopMatrix();
  }

  glEnable(GL_DEPTH_TEST);
}

void AWRoadPlannerGUI::drawTopology(double center_x, double center_y) {
  RoutePlanner::RndfEdge* edge = awp_.chsm_planner_->topology->ego_vehicle.edge();
  if (edge==NULL)
    return;
  double cx = awp_.chsm_planner_->topology->ego_vehicle.xMatched();
  double cy = awp_.chsm_planner_->topology->ego_vehicle.yMatched();

    // paint the whole graph in white
  if (show_complete_graph_)
    awp_.chsm_planner_->topology->paint_complete_graph(center_x, center_y, 1, 1, 1);

  // paint mission graph on top in pink
  if (show_mission_graph_) {
    pthread_mutex_lock(&awp_.chsm_planner_->mission_mutex_);
    awp_.chsm_planner_->topology->paint_mission_graph(center_x, center_y, 1, 0, 1, false);
    pthread_mutex_unlock(&awp_.chsm_planner_->mission_mutex_);
  }

  if (show_complete_mission_graph_) {
    pthread_mutex_lock(&awp_.chsm_planner_->mission_mutex_);
    awp_.chsm_planner_->topology->paint_complete_mission(center_x, center_y);
    pthread_mutex_unlock(&awp_.chsm_planner_->mission_mutex_);
  }

  // draw matched edge in green
  if (show_matched_edge_) {
    awp_.chsm_planner_->topology->draw_edge(center_x, center_y, *edge, 0., 1., 0.);
  }
  // draw this point in yellow
  if (show_yellow_dot_) {
    awp_.chsm_planner_->topology->draw_dot_at(cx-center_x, cy-center_y, 1., 1., 0.);
  }
  //glPopMatrix();
}

void AWRoadPlannerGUI::drawObstacles(double center_x, double center_y) {
  if(!awp_.chsm_planner_) return;

  if ( !awp_.chsm_planner_->vehicle_manager) {
    std::cout << "Warning: drawObstacles(): VehicleManager == 0!\n";
    return;
  }

  pthread_mutex_lock(&awp_.chsm_planner_->dyn_obstacles_mutex_);
  pthread_mutex_lock(&awp_.chsm_planner_->topology_mutex_);
  glPushMatrix();
 
 	int id_display = 0;
#ifdef HAVE_PROBT
  	if (awp_.chsm_planner_->topology->intersection_manager != NULL) {
	  	// Display turn signal for one vehicle only
	  	// Find relevant vehicle with smallest id
	  	id_display = awp_.chsm_planner_->vehicle_manager->vehicle_map.size();
	  	for (std::map<int, Vehicle>::iterator vehicle_map_it = awp_.chsm_planner_->vehicle_manager->vehicle_map.begin(); vehicle_map_it != awp_.chsm_planner_->vehicle_manager->vehicle_map.end(); ++vehicle_map_it) {
			if ((vehicle_map_it->second.id() < id_display) && (awp_.chsm_planner_->topology->intersection_manager->obstaclePredictor().relevant_vehicles_.find(vehicle_map_it->second.id()) != awp_.chsm_planner_->topology->intersection_manager->obstaclePredictor().relevant_vehicles_.end())) {
				id_display = vehicle_map_it->second.id();
			}
		}
	}
	//cout << "display turn signal vehicle " << id_display << endl;
#endif
 
//    pthread_mutex_lock(&awp_.chsm_planner_->intersection_predictor_mutex_);
    for (std::map<int, Vehicle>::iterator it = awp_.chsm_planner_->vehicle_manager->vehicle_map.begin(); it
  != awp_.chsm_planner_->vehicle_manager->vehicle_map.end(); it++) {
    const Vehicle& veh = it->second;

	  double width = (veh.width() != 0.0 ? veh.width() : 2.5);
      double length = (veh.length() != 0.0 ? veh.length() : 4.5);
      
      
      // Turn signals
	  double proba_none = 0.0;
      double proba_left = 0.0;
      double proba_right = 0.0;
      
      bool display_turn_signal = false;
      if (veh.id() == id_display) {
		  display_turn_signal = true;
	  }
      
#ifdef HAVE_PROBT
      if (display_turn_signal) {
      
	      if (awp_.chsm_planner_->topology->intersection_manager != NULL) {
		      
		      //IntersectionManager::VehIdTurnSignalProbaMap turn_signals_map;
			  //if (veh.isOnIntersection(awp_.chsm_planner_->topology->intersection_manager->getIntersection())) {
				  //turn_signals_map = awp_.chsm_planner_->topology->intersection_manager->veh_id_turn_signal_int_proba_map_;
			  //}
			  //else {
				  //turn_signals_map = awp_.chsm_planner_->topology->intersection_manager->veh_id_turn_signal_proba_map_;
			  //}
			  
			  //IntersectionManager::VehIdTurnSignalProbaMap turn_signals_map;
			  //turn_signals_map = awp_.chsm_planner_->topology->intersection_manager->veh_id_turn_signal_proba_map_;
			  
	        ObstaclePredictor::VehIdTurnSignalProbaMap turn_signals_map;
			  //if (awp_.chsm_planner_->topology->intersection_manager->relevant_vehicles_in_intersection_.find(veh.id()) != awp_.chsm_planner_->topology->intersection_manager->relevant_vehicles_in_intersection_.end()) {
				  //turn_signals_map = awp_.chsm_planner_->topology->intersection_manager->veh_id_turn_signal_int_proba_map_;
			  //}
			  //else if (awp_.chsm_planner_->topology->intersection_manager->relevant_vehicles_approaching_intersection_.find(veh.id()) != awp_.chsm_planner_->topology->intersection_manager->relevant_vehicles_approaching_intersection_.end()) {
				  //turn_signals_map = awp_.chsm_planner_->topology->intersection_manager->veh_id_turn_signal_proba_map_;
			  //}
			  
			  //if (awp_.chsm_planner_->topology->intersection_manager->relevant_vehicles_approaching_intersection_.find(veh.id()) != awp_.chsm_planner_->topology->intersection_manager->relevant_vehicles_approaching_intersection_.end()) {
			      
			      turn_signals_map = awp_.chsm_planner_->topology->intersection_manager->obstaclePredictor().veh_id_turn_signal_proba_map_;
			      
			      ObstaclePredictor::VehIdTurnSignal veh_id_turn_signal;
			      veh_id_turn_signal.veh_id_ = veh.id();
			      
			      veh_id_turn_signal.turn_signal_ = 0;
			      if (turn_signals_map.find(veh_id_turn_signal) != turn_signals_map.end()) {
					  proba_none = turn_signals_map[veh_id_turn_signal];
				  }
				  else {
					  cout << "Found no information about turn signal of vehicle " << veh.id() << endl;
				  }
			      veh_id_turn_signal.turn_signal_ = 1;
			      if (turn_signals_map.find(veh_id_turn_signal) != turn_signals_map.end()) {
					  proba_left = turn_signals_map[veh_id_turn_signal];
				  }
			      veh_id_turn_signal.turn_signal_ = 2;
			      if (turn_signals_map.find(veh_id_turn_signal) != turn_signals_map.end()) {
					  proba_right = turn_signals_map[veh_id_turn_signal];
				  }
			  //}
			  
		  }
		  
	  }
#endif
 //     pthread_mutex_unlock(&awp_.chsm_planner_->intersection_predictor_mutex_);
	  //cout << "speed=" << veh.speed() << endl;
//	  if ((awp_.chsm_planner_->topology->intersection_manager != NULL) && (veh.speed() != -100.0)) {
	  //if ((awp_.chsm_planner_->topology->intersection_manager != NULL) && (awp_.chsm_planner_->topology->intersection_manager->relevant_vehicles_.find(veh.id()) != awp_.chsm_planner_->topology->intersection_manager->relevant_vehicles_.end())) {
		  draw_observed_car(veh.xMatchedFrom()-center_x, veh.yMatchedFrom()-center_y, veh.yawMatchedFrom(), width, length, veh.id(),
  				veh.speed(), 1, 1, 0, 0, 0, 0, true, gui3D.camera_pose.pan, proba_none, proba_left, proba_right, display_turn_signal);
//		}


    // Multi Edge Kanten malen
    for (std::map< RndfEdge*, double >::const_iterator edge_it = veh.edges().begin(); edge_it != veh.edges().end(); ++edge_it) {
      RndfEdge* edge = edge_it->first;
      Point_2 p1( edge->fromVertex()->x(), edge->fromVertex()->y() );
      Point_2 p2( edge->toVertex()->x(), edge->toVertex()->y() );
      Point_2 dot = p1 + ( p2 - p1 ) * edge_it->second / edge->getLength();
      awp_.chsm_planner_->topology->draw_dot_at(dot.x()-center_x, dot.y()-center_y, 0.7, 0.2, 0);
      awp_.chsm_planner_->topology->draw_edge(center_x,center_y, *edge, 0.7, 0, 0.7);
    }

    // draw main matched edge
    awp_.chsm_planner_->topology->draw_dot_at(veh.xMatched()-center_x, veh.yMatched()-center_y, 0, .5, 0);
    awp_.chsm_planner_->topology->draw_edge(center_x,center_y, *(veh.edge()), 1, 0, 1);
  }
  glColor3f(0.0, 0.8, 0.0);
  for (std::map<int, Vehicle*>::iterator it = awp_.chsm_planner_->vehicle_manager->moving_map.begin(); it
      != awp_.chsm_planner_->vehicle_manager->moving_map.end(); it++) {
    const Vehicle* veh = it->second;
//  		std::cout << "moving " << veh->dist_to_end << " "<< veh->xMatchedFrom() <<" "<< veh->yMatchedFrom()<<" "<< veh->length << std::endl;
    draw_dashed_circle(veh->xMatchedFrom()-center_x, veh->yMatchedFrom()-center_y, veh->length());
  }
  pthread_mutex_unlock(&awp_.chsm_planner_->topology_mutex_);

  glColor3f(0.8, 0.0, 0.0);
  for (std::map<int, Vehicle*>::iterator it = awp_.chsm_planner_->vehicle_manager->blockage_map.begin(); it
      != awp_.chsm_planner_->vehicle_manager->blockage_map.end(); it++) {
    const Vehicle* veh = it->second;
//  		std::cout << "standing " << veh->dist_to_end << " " <<veh->xMatchedFrom() <<" "<< veh->yMatchedFrom()<<" "<< veh->length << std::endl;
    draw_dashed_circle(veh->xMatchedFrom()-center_x, veh->yMatchedFrom()-center_y, veh->length());
  }
  glPopMatrix();
  pthread_mutex_unlock(&awp_.chsm_planner_->dyn_obstacles_mutex_);

  glPushMatrix();

//  double d;
//  GraphTools::PlaceOnGraph obstacle_place;
//  Vehicle* next_obst = awp_.chsm_planner_->topology->next_obstacle(d, obstacle_place, MAX_SCAN_DISTANCE_OBSTACLE );
//
//  if (next_obst ) {
//    draw_observed_car(next_obst->xMatchedFrom()-center_x, next_obst->yMatchedFrom(), next_obst->yawMatchedFrom()-center_y, 2.5, 4.5, 0,
//        0, 0, 1, 0, 0, 0, 0, false, gui3D.camera_pose.pan);
//
//    awp_.chsm_planner_->topology->draw_dot_at(next_obst->xMatched()-center_x, next_obst->yMatched()-center_y, 1., 0.5, 0.0);
//  }

  glPopMatrix();
}

void AWRoadPlannerGUI::drawVehicle() {
  glPushMatrix();
  glTranslatef(1.25, 0, 0.9);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
  passatwagonmodel_draw(car_, 0, 0, 0);
  glDisable(GL_LIGHTING);
  glPopMatrix();
}

  // this function is used to either update the raw obstacle map texture
  // or the configuration space texture
void AWRoadPlannerGUI::updateObstacleMapTexture(const uint8_t* map_data, GLuint& texture, double& last_timestamp) {

  if(!map_data) {return;}

if(awp_.chsm_planner_->obstacleMapTimestamp() == last_timestamp) {return;}

  glEnable(GL_TEXTURE_2D);
  glEnable(GL_TEXTURE_RECTANGLE_ARB);

  glBindTexture(GL_TEXTURE_RECTANGLE_ARB, texture);

//  glTexParameterf(tex_type_, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
//  glTexParameterf(tex_type_, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);


  glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GL_ALPHA, awp_.chsm_planner_->obstacleMapWidth(), awp_.chsm_planner_->obstacleMapHeight(), 0, GL_ALPHA, GL_UNSIGNED_BYTE, map_data);

  glDisable(GL_TEXTURE_2D);
  glDisable(GL_TEXTURE_RECTANGLE_ARB);


  last_timestamp = awp_.chsm_planner_->obstacleMapTimestamp();
}

void AWRoadPlannerGUI::drawObstacleMap(GLuint texture, float r, float g, float b, double cx, double cy, int32_t width, int32_t height, double res, double smooth_x, double smooth_y) {
  glEnable(GL_TEXTURE_2D);
  glEnable(GL_TEXTURE_RECTANGLE_ARB);

  glPushMatrix();
//  glTranslatef(-width*res/2, -height*res/2, 0);
  double trans_x = -width*res/2 - smooth_x + cx;
  double trans_y = -height*res/2 - smooth_y + cy;
  glTranslatef(trans_x, trans_y, 0);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );
  glBindTexture(GL_TEXTURE_RECTANGLE_ARB, texture);

  glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

  float x1=0, y1=0;
  float x2 = width * res;
  float y2 = height * res;
  glColor3f(r, g, b);
  glBegin(GL_POLYGON);
  glTexCoord2f(0, 0); glVertex2f(x2, y2);
  glTexCoord2f(0, height); glVertex2f(x2, y1);
  glTexCoord2f(width, height); glVertex2f(x1, y1);
  glTexCoord2f(width, 0); glVertex2f(x1, y2);
  glEnd();
  glDisable(GL_TEXTURE_RECTANGLE_ARB);
  glDisable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_RECTANGLE_ARB, 0);

    // draw map boundary
  glColor4f(1, 1, 0.2, 0.5);
  glBegin(GL_LINE_LOOP);
  glVertex2f(0, 0);
  glVertex2f(x2, 0);
  glVertex2f(x2, y2);
  glVertex2f(0, y2);
  glEnd();
  glPopMatrix();
}

void AWRoadPlannerGUI::drawDestinations(const double center_x, const double center_y) {
	
#ifdef HAVE_PROBT
  if (awp_.chsm_planner_->topology->intersection_manager && awp_.chsm_planner_->topology->intersection_manager->obstaclePredictor().veh_id_exit_vertex_proba_map_.size()>0) {
	  
	  // Display for one vehicle only
  	  // Find relevant vehicle with smallest id
  	  int id_display = awp_.chsm_planner_->vehicle_manager->vehicle_map.size();
  	  for (std::map<int, Vehicle>::iterator vehicle_map_it = awp_.chsm_planner_->vehicle_manager->vehicle_map.begin(); vehicle_map_it != awp_.chsm_planner_->vehicle_manager->vehicle_map.end(); ++vehicle_map_it) {
		  if ((vehicle_map_it->second.id() < id_display) && (awp_.chsm_planner_->topology->intersection_manager->obstaclePredictor().relevant_vehicles_.find(vehicle_map_it->second.id()) != awp_.chsm_planner_->topology->intersection_manager->obstaclePredictor().relevant_vehicles_.end())) {
			  id_display = vehicle_map_it->second.id();
		  }
	  }
	  //cout << "display destination vehicle " << id_display << endl;
	  
	  ObstaclePredictor::VehIdVertexProbaMap destinations_map = awp_.chsm_planner_->topology->intersection_manager->obstaclePredictor().veh_id_exit_vertex_proba_map_;
	  for (ObstaclePredictor::VehIdVertexProbaMap::const_iterator map_it = destinations_map.begin(); map_it != destinations_map.end(); ++map_it) {
		  if (awp_.chsm_planner_->topology->intersection_manager->obstaclePredictor().relevant_vehicles_.find(map_it->first.veh_id_) != awp_.chsm_planner_->topology->intersection_manager->obstaclePredictor().relevant_vehicles_.end()) {
			  // Display for one vehicle only
			  if (map_it->first.veh_id_== id_display) {
				  //cout << "draw destination for vehicle " << id_display << endl;
				  double proba = map_it->second;
				  double x = map_it->first.vertex_->x();
				  double y = map_it->first.vertex_->y();
				  glPushMatrix();
				  glColor3f(0.0, 1.0, 0.0);
				  draw_circle(x-center_x, y-center_y, 4.0*sqrt(proba), 1);
				  glPopMatrix();
			  }
		  }
	  }
  }
#endif
  }

void AWRoadPlannerGUI::drawPoseAccuracy(dgc_pose_t robot_pose_accuracy) {
  glPushMatrix();
  glColor4f(0.6, 0.6, 0, 0.5);
  draw_ellipse(0, 0, robot_pose_accuracy.x, robot_pose_accuracy.y,0);
  glPopMatrix();
}

void AWRoadPlannerGUI::drawDistanceCond(const double x, const double y, const double r)
{
  if (r < std::numeric_limits<double>::max()) {
    draw_circle(x, y, std::abs(r), 0);
    draw_circle(x, y, 0.15, 1);
  }
}

void AWRoadPlannerGUI::drawEgoDistances(const double /*center_x*/, const double /*center_y*/, const double yaw, Topology* t)
{
  if (!t->isMissionPlanned()) return;

//  double x = FRONT_BUMPER_DELTA*cos(yaw) - center_x;
//  double y = FRONT_BUMPER_DELTA*sin(yaw) - center_y;
  double x = FRONT_BUMPER_DELTA*cos(yaw);
  double y = FRONT_BUMPER_DELTA*sin(yaw);
  glColor3f(0.0, 0.0, 1.0);
  drawDistanceCond(x, y, t->dist_to_next_standing_veh());
  glColor3f(0.0, 1.0, 0.0);
  drawDistanceCond(x, y, t->dist_to_next_moving_veh());
  glColor3f(1.0, 0.0, 0.0);
  drawDistanceCond(x, y, t->dist_to_next_stopline());
  glColor3f(1.0, 0.0, 0.0);
  drawDistanceCond(x, y, t->dist_to_next_traffic_light());
  glColor3f(1.0, 0.0, 0.0);
  drawDistanceCond(x, y, t->dist_to_next_crosswalk());
  glColor3f(0.5, 0.0, 0.0);
  drawDistanceCond(x, y, t->dist_to_next_intersection());
  glColor3f(0.7, 0.7, 0.0);
  drawDistanceCond(x, y, t->dist_to_next_kturn());
  glColor3f(0.3, 1.0, 0.3);
  drawDistanceCond(x, y, t->dist_to_next_zone());
  glColor3f(0.3, 0.7, 0.3);
  drawDistanceCond(x, y, t->dist_to_next_zone_exit());
  glColor3f(0.3, 0.7, 0.3);
  drawDistanceCond(BACK_BUMPER_DELTA*-cos(yaw), BACK_BUMPER_DELTA*-sin(yaw), t->dist_to_prev_veh());


}

void AWRoadPlannerGUI::drawVehDistances(const double center_x, const double center_y, Topology* t)
{
//	cout << "Draw Veh Distances" << endl;
  if (!t->vehicle_manager) return;

  // Nächste Kreuzung ermitteln
  RndfIntersection* isec = t->get_next_intersection();
//	if (!isec) return;
//	if (isec) cout << "  -> Intersec found ("<< isec->getId() <<")" << endl;

  glPushMatrix();
  glTranslatef(-center_x, -center_y, 0.);

  // Distanzen für die einzeln Fahrzuge ermitteln und zeichnen
  std::map<int, Vehicle>& vehicle_map = t->vehicle_manager->getVehicles();
  for (std::map<int, Vehicle>::iterator it=vehicle_map.begin(); it != vehicle_map.end(); ++it)
  {
    const Vehicle& veh = it->second;
    double x  = veh.xMatchedFrom();
    double y  = veh.yMatchedFrom();
    double fx = veh.xMatchedFrom() + (veh.length()/2.)*cos(veh.yawMatchedFrom());
    double fy = veh.yMatchedFrom() + (veh.length()/2.)*sin(veh.yawMatchedFrom());
    double bx = veh.xMatchedFrom() - (veh.length()/2.)*cos(veh.yawMatchedFrom());
    double by = veh.yMatchedFrom() - (veh.length()/2.)*sin(veh.yawMatchedFrom());

//		// Distanz zur Stoplinine visualisieren
//		double dist_to_stopline = veh.distToStopLine(isec);
//		cout << "  -> Veh "<< veh.id <<" dist to stopline: "<< dist_to_stopline <<" m" << endl;
//		if ( !isfinite(dist_to_stopline) ) continue;
//
//		if ( fabs(dist_to_stopline) <= VEH_DIST_FROM_STOPLINE_THRESHOLD) {
//			glColor3f(0.0, 1.0, 0.0);
//			drawDistanceCond(x, y, veh.length / 2);
//		} else if ( dist_to_stopline >= 0. ) {
//			glColor3f(0.0, 1.0, 0.0);
//			drawDistanceCond(fx, fy, dist_to_stopline);
//		} else {
//			glColor3f(1.0, 0.0, 0.0);
//			drawDistanceCond(fx, fy, -dist_to_stopline);
//		}

    // Distanz zur Intersection visualisieren
    double dist_to_intersection = veh.distToIntersection(isec);
//		cout << "  -> Veh "<< veh.id <<" dist to isec: "<< dist_to_intersection <<" m" << endl;
    if ( !std::isfinite(dist_to_intersection) ) {continue;}

    if ( std::abs(dist_to_intersection) == 0.) {
      glColor3f(0.0, 1.0, 0.0);
      drawDistanceCond(x, y, veh.length());
    } else if ( dist_to_intersection > 0.) {
      glColor3f(0.0, 0.0, 1.0);
      drawDistanceCond(fx, fy, dist_to_intersection);
    } else {
      glColor3f(1.0, 0.0, 0.0);
      drawDistanceCond(bx, by, -dist_to_intersection);
    }

  }

	glPopMatrix();
}

void AWRoadPlannerGUI::drawIntersectionMergingPoint(const double center_x, const double center_y, Topology* t)
{
	if (!t->vehicle_manager) return;

	RndfIntersection* isec = t->get_next_intersection();
	if (!isec) return;
	//if (isec) cout << "  -> Intersec found ("<< isec->getId() <<") center " << isec->center() << endl;

	CGAL_Geometry::Point_2 center = isec->center();
	glColor3f(1.0, 0.3, 0.3);
	draw_circle(center.x()-center_x, center.y()-center_y, 0.5, 1);
	draw_circle(center.x()-center_x, center.y()-center_y, isec->getRadius(), 0);

	// Speziellen MergingPoint ermiteln und malen
	glColor3f(1.0, 0.5, 0.1);
	GraphPlace ego_place( t->ego_vehicle.edge(), t->ego_vehicle.distFromStart() );
	for (std::map<int, Vehicle>::iterator it = t->vehicle_manager->vehicle_map.begin(); it != t->vehicle_manager->vehicle_map.end(); ++it)
	{
		GraphPlace veh_place( it->second.edge(), it->second.distFromStart() );
		GraphPlace cross_place = searchCrossingPoint(ego_place, veh_place, isec, 50.);
		if (cross_place.valid) {
			Point_2 cp = cross_place.point();
	//		std::cout << "[CrossPoint] Isec found: "<< cp << std::endl;
			draw_circle(cp.x()-center_x, cp.y()-center_y, 0.5, 1);
		}// else std::cout << "[CrossPoint] No Isec found"<< std::endl;
	}
}

void AWRoadPlannerGUI::drawLaneChangeMergingPoints(const double center_x, const double center_y, Topology* t)
{
	GraphPlace ego_place( t->ego_vehicle.edge(), t->ego_vehicle.distFromStart() );

	StLaneChange* lc = awp_.chsm_planner_->lane_change_data;
	if ( ! lc ) {
//		cout << "** No Lanechange Data available **" << endl;
		return;
	}
//	cout << "Lanechange Data available" << endl;

	// draw change point
	if ( lc->change_point.isValid() ) {
		Point_2 lcs_p = (*lc->change_point.edge)->getEdge()->point( lc->merge_point.offset );
		glColor3f(0.9, 0.3, 0.5);
		draw_circle(lcs_p.x()-center_x, lcs_p.y()-center_y, 0.5, 1);
	}

	// draw merge point
	if ( lc->merge_point.valid ) {
		glColor3f(0.9, 0.3, 0.5);
		draw_circle(lc->merge_point.point().x()-center_x, lc->merge_point.point().y()-center_y, 0.5, 1);
		if (lc->merge_allowed)
			glColor3f(0., 0.9, 0.);
		else
			glColor3f(0.9, 0., 0.);
		draw_circle(lc->merge_point.point().x()-center_x, lc->merge_point.point().y()-center_y, 0.9, 0);
	}

	// draw merge end point
	if ( lc->merge_end_point.valid ) {
		glColor3f(0.9, 0.3, 0.5);
		draw_circle(lc->merge_end_point.point().x()-center_x, lc->merge_end_point.point().y()-center_y, 0.5, 0);
		if (lc->merge_allowed)
			glColor3f(0., 0.9, 0.);
		else
			glColor3f(0.9, 0., 0.);
		draw_circle(lc->merge_end_point.point().x()-center_x, lc->merge_end_point.point().y()-center_y, 0.9, 0);
	}

	//
//	// merge left
//	GraphPlace merge_start = ego_place;
//	GraphPlace merge_end;
//	merge_start.goToLeftEdge();
//	if ( ! merge_start.valid ) goto merge_right;
//	glColor3f(0.9, 0.3, 0.5);
//	draw_circle(merge_start.point().x()-center_x, merge_start.point().y()-center_y, 0.5, 1);
//
//	merge_end = searchDistOnLane(merge_start, GraphSearchTraits::FORWARD, 30. );
//	if ( ! merge_end.valid ) goto merge_right;
//	glColor3f(0.9, 0.3, 0.6);
//	draw_circle(merge_end.point().x()-center_x, merge_end.point().y()-center_y, 0.8, 0);
//
//	merge_right:
//
//	// merge right
//	merge_start = ego_place;
//	merge_start.goToRightEdge();
//	if ( ! merge_start.valid ) goto merge_left_opposite;
//	glColor3f(0.9, 0.3, 0.5);
//	draw_circle(merge_start.point().x()-center_x, merge_start.point().y()-center_y, 0.5, 1);
//
//	merge_end = searchDistOnLane(merge_start, GraphSearchTraits::FORWARD, 30. );
//	if ( ! merge_end.valid ) goto merge_left_opposite;
//	glColor3f(0.9, 0.3, 0.6);
//	draw_circle(merge_end.point().x()-center_x, merge_end.point().y()-center_y, 0.8, 0);
//
//	merge_left_opposite:
//
//	// merge right
//	merge_start = ego_place;
//	merge_start.goToLeftOppositeEdge();
//	if ( ! merge_start.valid ) return;
//	glColor3f(0.9, 0.3, 0.5);
//	draw_circle(merge_start.point().x()-center_x, merge_start.point().y()-center_y, 0.5, 1);
//
//	merge_end = searchDistOnLane(merge_start, GraphSearchTraits::BACKWARD, 30. );
//	if ( ! merge_end.valid ) return;
//	glColor3f(0.9, 0.3, 0.6);
//	draw_circle(merge_end.point().x()-center_x, merge_end.point().y()-center_y, 0.8, 0);
}


  // create 2 rotated quads
void createCirclePointList(double r, float* points) {
  int32_t i=0;
    // quad 1
  points[i++] = r;  points[i++] = 0;
  points[i++] = 0;  points[i++] = r;
  points[i++] = -r; points[i++] = 0;
  points[i++] = 0;  points[i++] = -r;

    // quad 2
  double rd = r * M_SQRT1_2;
  points[i++] = rd; points[i++] = rd;
  points[i++] = -rd; points[i++] = rd;
  points[i++] = -rd; points[i++] = -rd;
  points[i++] = rd; points[i++] = -rd;
}

void AWRoadPlannerGUI::updateObstacleMapFromMessage(double& cx, double& cy, int32_t& width, int32_t& height, double& res) {
  static PerceptionObstaclePoint* static_obstacle_points=NULL;
  static int32_t static_obstacle_max_points=0;
  static double last_timestamp=0, r=0, r_maybe=0;
  static float* point_buf=NULL, *maybe_buf=NULL, circle_buf[2*2*4], maybe_circle[2*2*4];

  if(!static_obstacle_points) {
    width = awp_.chsm_planner_->obstacleMapWidth();
    height = awp_.chsm_planner_->obstacleMapHeight();
    res = awp_.chsm_planner_->obstacleMapResolution();
    static_obstacle_max_points = width*height;
    static_obstacle_points = new PerceptionObstaclePoint[static_obstacle_max_points];
    r = awp_.chsm_planner_->traj_eval_->params().safety_width / (2 * res);
    r_maybe = r + 5;
    createCirclePointList(r, circle_buf);
    createCirclePointList(r_maybe, maybe_circle);
    point_buf = new float[2*8*static_obstacle_max_points];
    maybe_buf = new float[2*8*static_obstacle_max_points];
  }

  double t0=Time::current();
  int32_t static_obstacle_num_points = static_obstacle_max_points;
  bool new_data_ready=false;
  try {
    new_data_ready = awp_.chsm_planner_->obstacleMapMsgData(static_obstacle_num_points, last_timestamp, *static_obstacle_points, cx, cy);
  }
  catch(Exception& e) {
    std::cout << e.what() << std::endl;
    return;
  }

  if(!new_data_ready) {return;}

  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, osm_circleconv_frame_buffer_);

  glPushAttrib(GL_TRANSFORM_BIT | GL_VIEWPORT_BIT);
  glMatrixMode(GL_PROJECTION);
  glPushMatrix(); glLoadIdentity();
  glOrtho(0., width, 0., height, -1, 1);

  double x1=0, y1=0;
  double x2=(double)width;
  double y2=(double)height;

  glViewport(x1, y1, x2, y2);

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix(); glLoadIdentity();

  glDisable(GL_DEPTH_TEST);

  glClearColor(0.0, 0.0, 0.0, 0.0);
  glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

  float* maybe_ptr=maybe_buf;
  float* point_ptr=point_buf;
  uint32_t num_points=0;

  for (uint32_t i = 0; i < uint32_t(static_obstacle_num_points); i++) {
    int32_t xi = width / 2 - (int32_t) ((static_obstacle_points[i].x - cx) / res+.5);
    int32_t yi = height / 2 - (int32_t) ((static_obstacle_points[i].y - cy) / res+.5);
    if (xi >= 0 && xi < int32_t(width) && yi >= 0 && yi < int32_t(height)) {
      uint8_t val = static_obstacle_points[i].type;

      //        if (val == PERCEPTION_MAP_OBSTACLE_LOW || val == PERCEPTION_MAP_OBSTACLE_HIGH || val
      //            == PERCEPTION_MAP_OBSTACLE_UNKNOWN) { // val == PERCEPTION_MAP_OBSTACLE_DYNAMIC) {
      if (val == PERCEPTION_MAP_OBSTACLE_FREE) {
        *maybe_ptr = xi+maybe_circle[0]; maybe_ptr++; *maybe_ptr = yi+maybe_circle[1]; maybe_ptr++;
        *maybe_ptr = xi+maybe_circle[2]; maybe_ptr++; *maybe_ptr = yi+maybe_circle[3]; maybe_ptr++;
        *maybe_ptr = xi+maybe_circle[4]; maybe_ptr++; *maybe_ptr = yi+maybe_circle[5]; maybe_ptr++;
        *maybe_ptr = xi+maybe_circle[5]; maybe_ptr++; *maybe_ptr = yi+maybe_circle[7]; maybe_ptr++;

        *maybe_ptr = xi+maybe_circle[8]; maybe_ptr++; *maybe_ptr = yi+maybe_circle[9]; maybe_ptr++;
        *maybe_ptr = xi+maybe_circle[10]; maybe_ptr++; *maybe_ptr = yi+maybe_circle[11]; maybe_ptr++;
        *maybe_ptr = xi+maybe_circle[12]; maybe_ptr++; *maybe_ptr = yi+maybe_circle[13]; maybe_ptr++;
        *maybe_ptr = xi+maybe_circle[14]; maybe_ptr++; *maybe_ptr = yi+maybe_circle[15]; maybe_ptr++;


        *point_ptr = xi+circle_buf[0]; point_ptr++; *point_ptr = yi+circle_buf[1]; point_ptr++;
        *point_ptr = xi+circle_buf[2]; point_ptr++; *point_ptr = yi+circle_buf[3]; point_ptr++;
        *point_ptr = xi+circle_buf[4]; point_ptr++; *point_ptr = yi+circle_buf[5]; point_ptr++;
        *point_ptr = xi+circle_buf[5]; point_ptr++; *point_ptr = yi+circle_buf[7]; point_ptr++;

        *point_ptr = xi+circle_buf[8]; point_ptr++; *point_ptr = yi+circle_buf[9]; point_ptr++;
        *point_ptr = xi+circle_buf[10]; point_ptr++; *point_ptr = yi+circle_buf[11]; point_ptr++;
        *point_ptr = xi+circle_buf[12]; point_ptr++; *point_ptr = yi+circle_buf[13]; point_ptr++;
        *point_ptr = xi+circle_buf[14]; point_ptr++; *point_ptr = yi+circle_buf[15]; point_ptr++;
        num_points++;
      }
    }
  }
  glEnableClientState(GL_VERTEX_ARRAY);
  glColor3ub(OSM_MAYBE_BLOCKED, OSM_MAYBE_BLOCKED, OSM_MAYBE_BLOCKED);
  glVertexPointer(2, GL_FLOAT, 0, maybe_buf);
  glDrawArrays(GL_QUADS, 0, 8*num_points);

  glColor3ub(OSM_BLOCKED, OSM_BLOCKED, OSM_BLOCKED);
  glVertexPointer(2, GL_FLOAT, 0, point_buf);
  glDrawArrays(GL_QUADS, 0, 8*num_points);
  glDisableClientState(GL_VERTEX_ARRAY);


//  glBegin(GL_QUADS);
//
//    for (uint32_t i = 0; i < uint32_t(static_obstacle_num_points); i++) {
//      int32_t xi = width / 2 - (int32_t) ((static_obstacle_points[i].x - cx) / res+.5);
//      int32_t yi = height / 2 - (int32_t) ((static_obstacle_points[i].y - cy) / res+.5);
//      if (xi >= 0 && xi < int32_t(width) && yi >= 0 && yi < int32_t(height)) {
//        uint8_t val = static_obstacle_points[i].type;
//
//        //        if (val == PERCEPTION_MAP_OBSTACLE_LOW || val == PERCEPTION_MAP_OBSTACLE_HIGH || val
//        //            == PERCEPTION_MAP_OBSTACLE_UNKNOWN) { // val == PERCEPTION_MAP_OBSTACLE_DYNAMIC) {
//        if (val == PERCEPTION_MAP_OBSTACLE_FREE) {
//          glColor3ub(OSM_MAYBE_BLOCKED, OSM_MAYBE_BLOCKED, OSM_MAYBE_BLOCKED);
//          glVertex2i(xi+maybe_circle[0], yi+maybe_circle[1]);
//          glVertex2i(xi+maybe_circle[2], yi+maybe_circle[3]);
//          glVertex2i(xi+maybe_circle[4], yi+maybe_circle[5]);
//          glVertex2i(xi+maybe_circle[6], yi+maybe_circle[7]);
//
//          glVertex2i(xi+maybe_circle[8], yi+maybe_circle[9]);
//          glVertex2i(xi+maybe_circle[10], yi+maybe_circle[11]);
//          glVertex2i(xi+maybe_circle[12], yi+maybe_circle[13]);
//          glVertex2i(xi+maybe_circle[14], yi+maybe_circle[15]);
//
//          glColor3ub(OSM_BLOCKED, OSM_BLOCKED, OSM_BLOCKED);
//          glVertex2i(xi+circle_buf[0], yi+circle_buf[1]);
//          glVertex2i(xi+circle_buf[2], yi+circle_buf[3]);
//          glVertex2i(xi+circle_buf[4], yi+circle_buf[5]);
//          glVertex2i(xi+circle_buf[6], yi+circle_buf[7]);
//
//          glVertex2i(xi+circle_buf[8], yi+circle_buf[9]);
//          glVertex2i(xi+circle_buf[10], yi+circle_buf[11]);
//          glVertex2i(xi+circle_buf[12], yi+circle_buf[13]);
//          glVertex2i(xi+circle_buf[14], yi+circle_buf[15]);
//        }
//      }
//    }
//    glEnd();

  glPopMatrix();

  glMatrixMode(GL_PROJECTION);
  glPopMatrix();

  glPopAttrib();
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
  printf("render dt: %f\n",  Time::current()-t0);
  }


void AWRoadPlannerGUI::display() {
//  static double last_t=0;
//  double t = Time::current();
//  double dt = t-last_t;
//  last_t=t;
//  printf("dt=%f\n", dt*1000);

    // clear window
  glClearColor(0.13, 0.32, 0.17, 1.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  if(!awp_.received_localize_pose_ || !awp_.received_applanix_pose_) {return;}

  Pose pose = awp_.chsm_planner_->currentPose();
  double smooth_x = pose.x();
  double smooth_y = pose.y();
  pthread_mutex_lock(&awp_.chsm_planner_->pose_mutex_);
  dgc_pose_t robot_pose_accuracy_copy = awp_.pose_accuracy_;
//  double robot_lat_copy = awp_.lat_;
//  double robot_lon_copy = awp_.lon_;
  pthread_mutex_unlock(&awp_.chsm_planner_->pose_mutex_);

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_LINE_SMOOTH);

  // draw things in the robot coordinate system
  glDisable(GL_DEPTH_TEST);
  glPushMatrix();

  // draw imagery
  bool show_flat_imagery = true;
  double current_time = Time::current();
  static double last_time = 0;
  if (show_imagery_) {
    if (current_time - last_time > 0.05) {
      dgc_imagery_update();
      last_time = current_time;
    }
    glPushMatrix();
    {
    std::string imagery_folder = "/driving/imagery";
    glTranslatef(0, 0, -DGC_PASSAT_HEIGHT);
      dgc_imagery_draw_3D(imagery_folder.c_str(), gui3D.camera_pose.distance, gui3D.camera_pose.x_offset,
          gui3D.camera_pose.y_offset, rndf_center_.x, rndf_center_.y, rndf_center_.zone, show_flat_imagery, 1.0, 1);
    }
    glPopMatrix();
  }

  // draw convoluted obstacle map (configuration space)
  static bool fbo_init_done_ = false;
  if(!fbo_init_done_ && awp_.chsm_planner_->obstacleMapRaw()) {
    if(!initFrameBufferObject(awp_.chsm_planner_->obstacleMapWidth(), awp_.chsm_planner_->obstacleMapHeight(),
        osm_circleconv_frame_buffer_, osm_circleconv_texture_)) {
    exit(0);
    }
    fbo_init_done_=true;
  }

  if (show_configuration_space_) {
    double cx, cy, res;
    int32_t width, height;
    pthread_mutex_lock(&awp_.chsm_planner_->static_obstacle_map_mutex_);
    cx = awp_.chsm_planner_->obstacleMapCenterX();
    cy = awp_.chsm_planner_->obstacleMapCenterY();
    width = awp_.chsm_planner_->obstacleMapWidth();
    height = awp_.chsm_planner_->obstacleMapHeight();
    res = awp_.chsm_planner_->obstacleMapResolution();
 //   updateObstacleMapFromMessage(cx, cy, width, height, res);
    updateObstacleMapTexture(awp_.chsm_planner_->obstacleMapCS(), configuration_space_texture_, last_configuration_space_timestamp_);
    pthread_mutex_unlock(&awp_.chsm_planner_->static_obstacle_map_mutex_);
 //   drawObstacleMap(osm_circleconv_texture_, 1, .8, 0, cx, cy, width, height, res, smooth_x, smooth_y);
    drawObstacleMap(configuration_space_texture_, 1, .8, 0, cx, cy, width, height, res, smooth_x, smooth_y);
  }

  // draw raw obstacle map
  if (show_obstacle_map_) {
    pthread_mutex_lock(&awp_.chsm_planner_->static_obstacle_map_mutex_);
    double cx, cy, width, height, res;
    cx = awp_.chsm_planner_->obstacleMapCenterX();
    cy = awp_.chsm_planner_->obstacleMapCenterY();
    width = awp_.chsm_planner_->obstacleMapWidth();
    height = awp_.chsm_planner_->obstacleMapHeight();
    res = awp_.chsm_planner_->obstacleMapResolution();
    updateObstacleMapTexture(awp_.chsm_planner_->obstacleMapRaw(), obstacle_map_texture_, last_raw_map_timestamp_);
    pthread_mutex_unlock(&awp_.chsm_planner_->static_obstacle_map_mutex_);
    drawObstacleMap(obstacle_map_texture_, 1, .2, 0, cx, cy, width, height, res, smooth_x, smooth_y);
  }

  // draw rndf
  if (show_rndf_) {
      glPushMatrix();
      awp_.chsm_planner_->topology->roadNetwork().draw(pose.utmX(), pose.utmY(), 1, true);
//      draw_rndf_display_list(&rndf_display_list2_, 0, 1, 1, 1, 0, 0, pose.utmX(), pose.utmY());
      glPopMatrix();
    //glPushMatrix();
    //glTranslatef(rndf_dl_origin_x_ - pose.utmX(), rndf_dl_origin_y_ - pose.utmY(), 0);
    //glCallList(rndf_display_list_);
//    if(draw_stops) {
//      if(threeD_signs)
//        glCallList(dl->threeD_stops_dl);
//      else
//        glCallList(dl->flat_stops_dl);
//    }
//    glPopMatrix();
  }

  // draw the trajectory
  if (show_pose_history_) {
    drawPoseHistory(pose.utmX(), pose.utmY());
//    draw_corrected_trajectory(pose.utmX(), pose.utmY());
  }

  // draw ego vehicle distances
  if (show_ego_distances_) {
    pthread_mutex_lock(&awp_.chsm_planner_->topology_mutex_);
    drawEgoDistances(pose.utmX(), pose.utmY(), pose.yaw(), awp_.chsm_planner_->topology);
    pthread_mutex_unlock(&awp_.chsm_planner_->topology_mutex_);
  }


  // draw other vehicles distances
  if (show_veh_distances_) {
    pthread_mutex_lock(&awp_.chsm_planner_->dyn_obstacles_mutex_);
    drawVehDistances(pose.utmX(), pose.utmY(), awp_.chsm_planner_->topology);
    pthread_mutex_unlock(&awp_.chsm_planner_->dyn_obstacles_mutex_);
  }

  // draw vehicle destination probabilities
  if (show_obstacle_destinations_) {
     // TODO: replace topology mutex
     pthread_mutex_lock(&awp_.chsm_planner_->topology_mutex_);
    //    pthread_mutex_lock(&awp_.chsm_planner_->intersection_predictor_mutex_);
    drawDestinations(pose.utmX(), pose.utmY());
     pthread_mutex_unlock(&awp_.chsm_planner_->topology_mutex_);
//    pthread_mutex_unlock(&awp_.chsm_planner_->intersection_predictor_mutex_);
  }

  // draw lane change merging points
  if (show_lanchange_merging_points_) {
    pthread_mutex_lock(&awp_.chsm_planner_->topology_mutex_);
	  drawLaneChangeMergingPoints(pose.utmX(), pose.utmY(), awp_.chsm_planner_->topology);
    pthread_mutex_unlock(&awp_.chsm_planner_->topology_mutex_);
  }

  // draw distance debug circles
  pthread_mutex_lock(&awp_.chsm_planner_->topology_mutex_);
  glPushMatrix();
	glTranslatef(-pose.utmX(), -pose.utmY(), 0);
	std::vector<Topology::DistanceVisualisation>::const_iterator iter = awp_.chsm_planner_->topology->debug_distances.begin();
	for (; iter != awp_.chsm_planner_->topology->debug_distances.end(); ++iter ) {
		glColor3f(iter->cr, iter->cg, iter->cb);
		drawDistanceCond(iter->x, iter->y , iter->r);
	}
	glPopMatrix();
  pthread_mutex_unlock(&awp_.chsm_planner_->topology_mutex_);


	if (show_intersection_merging_point_) {
    pthread_mutex_lock(&awp_.chsm_planner_->topology_mutex_);
	  drawIntersectionMergingPoint(pose.utmX(), pose.utmY(), awp_.chsm_planner_->topology);
    pthread_mutex_unlock(&awp_.chsm_planner_->topology_mutex_);
	}

  // draw the topology
  if (show_topology_) {
    pthread_mutex_lock(&awp_.chsm_planner_->topology_mutex_);
    drawTopology(pose.utmX(), pose.utmY());
    pthread_mutex_unlock(&awp_.chsm_planner_->topology_mutex_);
  }

  // draw radius rings
  if( show_radius_ ) {
    draw_distance_rings(0, 0, pose.yaw(), 120, 10);
  }

  // draw pose accuracy ellipse
  if (show_pose_accuracy_) {
    drawPoseAccuracy(robot_pose_accuracy_copy);
  }
  glEnable(GL_DEPTH_TEST);

  if (show_complete_center_line_) {
    pthread_mutex_lock(&awp_.chsm_planner_->center_line_mutex_);
    drawCompleteCenterLine( awp_.chsm_planner_->missionPointsBezier(), awp_.chsm_planner_->smoothed_mission_points_, pose.utmX(), pose.utmY());
    pthread_mutex_unlock(&awp_.chsm_planner_->center_line_mutex_);
  }

  if (show_center_line_) {
    pthread_mutex_lock(&awp_.chsm_planner_->center_line_mutex_);
    drawCenterline(awp_.chsm_planner_->center_line_, pose.utmX(), pose.utmY());
    pthread_mutex_unlock(&awp_.chsm_planner_->center_line_mutex_);
  }

  if (show_trajectories_) {
    pthread_mutex_lock(&awp_.chsm_planner_->trajectory_mutex_);
    drawTrajectories(pose);
    pthread_mutex_unlock(&awp_.chsm_planner_->trajectory_mutex_);
  }

  if (show_best_trajectory_) {
    pthread_mutex_lock(&awp_.chsm_planner_->trajectory_mutex_);
    drawBestTrajectory(pose);
    pthread_mutex_unlock(&awp_.chsm_planner_->trajectory_mutex_);
  }

  // draw obstacles
  if (show_dynamic_objects_) {
    drawObstacles(pose.utmX(), pose.utmY());
  }

  if(show_obstacle_predictions_) {
    pthread_mutex_lock(&awp_.chsm_planner_->dyn_obstacles_mutex_);
    for (std::map<int, Vehicle>::iterator it = awp_.chsm_planner_->vehicle_manager->vehicle_map.begin(); it != awp_.chsm_planner_->vehicle_manager->vehicle_map.end(); it++) {
    const Vehicle& veh = it->second;
      for (int j = veh.predictedTrajectory().size()-1; j>=0; j--) {
        const MovingBox& box = veh.predictedTrajectory()[j];
        glPushMatrix();
        glTranslatef(box.x - pose.utmX(), box.y - pose.utmY(), 0.0);
        glTranslatef(-box.length/2+box.ref_offset, 0, 0.9);
//        glRotatef(box.psi / M_PI * 180 + 90, 0, 0, 1);
        glRotatef(box.psi / M_PI * 180, 0, 0, 1);
        glEnable(GL_BLEND);
        glDisable(GL_DEPTH_TEST);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_LINE_SMOOTH);

        glLineWidth(2);
        float val = float(veh.predictedTrajectory().size()-1-j)/float(veh.predictedTrajectory().size()-1);
        float val2 = val*.4;
        if(j==0) {glColor4f(1, 0, 0, 0.7);} else {glColor4f(1-val, 1-val2, 1-val2, 0.2);}
        float l = box.length;
        float w = box.width;

        glBegin(GL_POLYGON);
        glVertex2f(l / 2, w / 2);
        glVertex2f(l / 2, -w / 2);
        glVertex2f(-l / 2, -w / 2);
        glVertex2f(-l / 2, w / 2);
        glEnd();

        glBegin(GL_LINE_LOOP);
        glVertex2f(l / 2, w / 2);
        glVertex2f(l / 2, -w / 2);
        glVertex2f(-l / 2, -w / 2);
        glVertex2f(-l / 2, w / 2);
        glEnd();

        glLineWidth(1);

        glDisable(GL_LINE_SMOOTH);
        glDisable(GL_BLEND);
        glEnable(GL_DEPTH_TEST);

//        glEnable(GL_LIGHTING);
//        glScalef(5.0, 5.0, 5.0);
//        if(box.length > 5.5) {
//          hummer_->draw();
//        }
//        else if(box.length < 5.2) {
//          elise_->draw();
//        }
//        else {
//          porsche_->draw();
//        }
//        glDisable(GL_LIGHTING);
        glPopMatrix();
      }

    }
    pthread_mutex_unlock(&awp_.chsm_planner_->dyn_obstacles_mutex_);

      //draw pedestrians
    pthread_mutex_lock(&awp_.chsm_planner_->pedestrian_prediction_mutex_);
    for (uint32_t i = 0; i < awp_.chsm_planner_->predicted_pedestrians_.size(); i++) {
      for (int j = awp_.chsm_planner_->predicted_pedestrians_[i].predicted_traj_.size()-1; j>=0; j--) {
        MovingBox& box = awp_.chsm_planner_->predicted_pedestrians_[i].predicted_traj_[j];
        glPushMatrix();
        glTranslatef(box.x - pose.utmX(), box.y - pose.utmY(), 0.0);
        glTranslatef(-box.length/2+box.ref_offset, 0, 0.9);
//        glRotatef(box.psi / M_PI * 180 + 90, 0, 0, 1);
        glRotatef(box.psi / M_PI * 180, 0, 0, 1);
        glEnable(GL_BLEND);
        glDisable(GL_DEPTH_TEST);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_LINE_SMOOTH);

        glLineWidth(2);
        float val = float(awp_.chsm_planner_->predicted_pedestrians_[i].predicted_traj_.size()-1-j)/float(awp_.chsm_planner_->predicted_pedestrians_[i].predicted_traj_.size()-1);
        float val2 = val*.4;
        if(j==0) {glColor4f(1, 0.7, .3, 0.7);} else {glColor4f(1-val, 1-val2, 1-val2, 0.2);}
        float l = box.length;
        float w = box.width;

        glBegin(GL_POLYGON);
        glVertex2f(l / 2, w / 2);
        glVertex2f(l / 2, -w / 2);
        glVertex2f(-l / 2, -w / 2);
        glVertex2f(-l / 2, w / 2);
        glEnd();

        glBegin(GL_LINE_LOOP);
        glVertex2f(l / 2, w / 2);
        glVertex2f(l / 2, -w / 2);
        glVertex2f(-l / 2, -w / 2);
        glVertex2f(-l / 2, w / 2);
        glEnd();

        glLineWidth(1);

        glDisable(GL_LINE_SMOOTH);
        glDisable(GL_BLEND);
        glEnable(GL_DEPTH_TEST);
        glPopMatrix();
      }
    }
    pthread_mutex_unlock(&awp_.chsm_planner_->pedestrian_prediction_mutex_);
  }

  glPushMatrix();
  glRotatef(dgc_r2d(pose.yaw()), 0, 0, 1);
  glRotatef(dgc_r2d(pose.pitch()), 0, 1, 0);
  glRotatef(dgc_r2d(pose.roll()), 1, 0, 0);

//#ifdef DRAW_VEHICLE_MODELL_3D
//  drawVehicle();
//#else
  //kogmo_graphics_draw_coordinate_frame(2.0);
  if(show_vehicle_) {
      drawVehicle();
  }
//#endif

  glPopMatrix();

  glEnable(GL_DEPTH_TEST);

  glPopMatrix();

  char str[100];
  set_display_mode_2D(gui3D.window_width, gui3D.window_height);
  const int lower_panel_width = 350;
  const int lower_panel_height = 100;
  const int right_panel_width = 200;
  const int right_panel_height = 200;

  // draw the transparent panel background
  glEnable (GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glColor4f(0.0, 0.0, 0.0, 0.5);
  glBegin(GL_QUADS);
  glVertex2f(0, 0);
  glVertex2f(0, lower_panel_height);
  glVertex2f(lower_panel_width, lower_panel_height);
  glVertex2f(lower_panel_width, 0);

  glVertex2f(gui3D.window_width-right_panel_width, gui3D.window_height-right_panel_height);
  glVertex2f(gui3D.window_width-right_panel_width, gui3D.window_height);
  glVertex2f(gui3D.window_width, gui3D.window_height);
  glVertex2f(gui3D.window_width, gui3D.window_height-right_panel_height);

  glVertex2f(gui3D.window_width-right_panel_width, 0);
  glVertex2f(gui3D.window_width-right_panel_width, 150);
  glVertex2f(gui3D.window_width, 150);
  glVertex2f(gui3D.window_width, 0);
  glEnd();

  glDisable (GL_BLEND);

  // draw text
  glColor3f(1, 1, 1);
  sprintf(str, "x: %.4f y: %.4f yaw: %.4f", pose.utmX(), pose.utmY(), pose.yaw());
  renderBitmapString(10, 70, GLUT_BITMAP_HELVETICA_12, str);
  sprintf(str, "lat: %.7f lon: %.7f", awp_.lat_,awp_.lon_);
  renderBitmapString(10, 50, GLUT_BITMAP_HELVETICA_12, str);
  sprintf(str, "speed: %.2f desired: %.2f follow: %.2f", dgc_ms2mph(pose.v()), dgc_ms2mph(awp_.chsm_planner_->velocity_desired_), dgc_ms2mph(awp_.chsm_planner_->velocity_following_));
  renderBitmapString(10, 30, GLUT_BITMAP_HELVETICA_12, str);

  // draw message queue
  int n=0;
  std::deque<std::string>& messages = awp_.chsm_planner_->getMessages();
  std::deque<std::string>::reverse_iterator it, it_end;
  for(it=messages.rbegin(), it_end=messages.rend(),n=0;it!=it_end;++it,++n) {
    sprintf(str, "%s", it->c_str());
    renderBitmapString(gui3D.window_width-right_panel_width+10, n*12+gui3D.window_height-right_panel_height+10, GLUT_BITMAP_HELVETICA_10, str);
  }

  int y_delta = 11;
  int y=y_delta*11;
  pthread_mutex_lock(&awp_.chsm_planner_->mission_mutex_);
  sprintf(str, "mission end: %f", awp_.chsm_planner_->topology->dist_to_mission_end());
  pthread_mutex_unlock(&awp_.chsm_planner_->mission_mutex_);
  renderBitmapString(gui3D.window_width-right_panel_width+10, y, GLUT_BITMAP_HELVETICA_10, str);
  y -=y_delta;
//  double standing, moving;
  pthread_mutex_lock(&awp_.chsm_planner_->dyn_obstacles_mutex_);
  pthread_mutex_lock(&awp_.chsm_planner_->topology_mutex_);
  pthread_mutex_lock(&awp_.chsm_planner_->mission_mutex_);
//  awp_.chsm_planner_->getVehicleDistances(standing, moving);
//  sprintf(str, "mv: %f (%f)", awp_.chsm_planner_->topology->dist_to_next_moving_veh(), moving);
//  renderBitmapString(gui3D.window_width-right_panel_width+10, y, GLUT_BITMAP_HELVETICA_10, str);
//  y -=y_delta;
//  sprintf(str, "sv: %f (%f)", awp_.chsm_planner_->topology->dist_to_next_standing_veh(), standing);
//  renderBitmapString(gui3D.window_width-right_panel_width+10, y, GLUT_BITMAP_HELVETICA_10, str);
//  y -=y_delta;
//  sprintf(str, "speed: %f", awp_.chsm_planner_->topology->speed_of_next_veh());
//  renderBitmapString(gui3D.window_width-right_panel_width+10, y, GLUT_BITMAP_HELVETICA_10, str);
//  y -=y_delta;
//  sprintf(str, "pv: %f", awp_.chsm_planner_->topology->dist_to_prev_veh());
//  renderBitmapString(gui3D.window_width-right_panel_width+10, y, GLUT_BITMAP_HELVETICA_10, str);
//  y -=y_delta;
  pthread_mutex_unlock(&awp_.chsm_planner_->mission_mutex_);
  pthread_mutex_unlock(&awp_.chsm_planner_->topology_mutex_);
  pthread_mutex_unlock(&awp_.chsm_planner_->dyn_obstacles_mutex_);

  pthread_mutex_lock(&awp_.chsm_planner_->topology_mutex_);
  sprintf(str, "stop: %f", awp_.chsm_planner_->topology->dist_to_next_stopline());
  renderBitmapString(gui3D.window_width-right_panel_width+10, y, GLUT_BITMAP_HELVETICA_10, str);
  y -=y_delta;
  sprintf(str, "lanechange: %f", awp_.chsm_planner_->topology->dist_to_next_lanechange());
  renderBitmapString(gui3D.window_width-right_panel_width+10, y, GLUT_BITMAP_HELVETICA_10, str);
  y -=y_delta;
  sprintf(str, "intersect: %f", awp_.chsm_planner_->topology->dist_to_next_intersection());
  renderBitmapString(gui3D.window_width-right_panel_width+10, y, GLUT_BITMAP_HELVETICA_10, str);
  y -=y_delta;
  sprintf(str, "kturn: %f", awp_.chsm_planner_->topology->dist_to_next_kturn());
  renderBitmapString(gui3D.window_width-right_panel_width+10, y, GLUT_BITMAP_HELVETICA_10, str);
  y -=y_delta;
  sprintf(str, "zone: %f", awp_.chsm_planner_->topology->dist_to_next_zone());
  renderBitmapString(gui3D.window_width-right_panel_width+10, y, GLUT_BITMAP_HELVETICA_10, str);
  y -=y_delta;
  sprintf(str, "zone exit: %f", awp_.chsm_planner_->topology->dist_to_next_zone_exit());
  renderBitmapString(gui3D.window_width-right_panel_width+10, y, GLUT_BITMAP_HELVETICA_10, str);
  y -=y_delta;
  sprintf(str, "next turn: %d", awp_.chsm_planner_->topology->nextTurnDirection());
  renderBitmapString(gui3D.window_width-right_panel_width+10, y, GLUT_BITMAP_HELVETICA_10, str);
  y -=y_delta;

  pthread_mutex_unlock(&awp_.chsm_planner_->topology_mutex_);

  pthread_mutex_lock(&awp_.chsm_planner_->trajectory_mutex_);
  TrajectoryEvaluator::traj_mode_t mode = awp_.chsm_planner_->traj_eval_->trajectoryMode();
  pthread_mutex_unlock(&awp_.chsm_planner_->trajectory_mutex_);
  switch (mode) {
  case TrajectoryEvaluator::TRAJ_MODE_FOLLOW:
    sprintf(str, "following");
    break;
  case TrajectoryEvaluator::TRAJ_MODE_VELOCITY:
    sprintf(str, "drive");
    break;
  case TrajectoryEvaluator::TRAJ_MODE_STOP:
    sprintf(str, "stop");
    break;
  }

  renderBitmapString(gui3D.window_width-right_panel_width+10, y, GLUT_BITMAP_HELVETICA_10, str);
  y -=y_delta;

//  printf("display time: %f\n", (Time::current() - last_t)*1000);

/*
bool record_images_=false;
  if (record_images_) {

    static uint8_t* img_buf = NULL;
    static uint32_t frame_num = 0;
    unsigned int img_id = 0;
    uint32_t width = 800, height = 600;

    if (!img_buf) {
      img_buf = new uint8_t[width * height * 3];
      ilInit();
      ilGenImages(1, &img_id);
      ilBindImage(img_id);
    }

    char buf[100];
    sprintf(buf, "planner%04d.JPG", frame_num);
    printf("%s\n", buf);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, img_buf);
    ilTexImage(width, height, 1, 3, IL_RGB, IL_UNSIGNED_BYTE, img_buf);
    ilSave(IL_JPG, buf);
    //if(!png->writePNGImage(buf, *sns_img)) {
    //  printf("Couldn't write image %s\n",buf);
    //}
    frame_num++;
  }
*/
}

// GLUI control callback
void AWRoadPlannerGUI::gluiControlCallback(int control) {
  if (control == CB_QUIT) {
    cout << "Exiting..." << endl << flush;
    awp_.quitPlanner();
  }
  else if (control == CB_REFRESH) {
    gui3D_forceRedraw();
  }
  else if (control == CB_UPDATE_PLANNER_PARAMS) {
    planner_params_.max_speed_global = (double) dgc_mph2ms(max_velocity_);
    planner_params_.curvature_slowdown_factor = (double) curvature_slowdown_factor_;
    awp_.chsm_planner_->params(planner_params_);
    glui_->sync_live();
  }
  else if (control == CB_UPDATE_TRAJECTORY_PARAMS) {
    TrajectoryEvaluator::parameters trj_params = awp_.chsm_planner_->traj_eval_->params();
    trj_params.weight = (double) urgency_;

//    trj_params.ptraj_2d.max_curvature                  = 100; //0.2; //[1/m]
//    trj_params.ptraj_2d.max_center_line_angular_offset = 40.* M_PI / 180.; // 30.* M_PI / 180.; //[]  25.* M_PI / 180.;
//    trj_params.ptraj_2d.max_lat_acceleration           = 10;// 1.0; // [m/s²]
//    trj_params.ptraj_2d.max_lon_acceleration           = 20;// 2.0; // [m/s²]
//    trj_params.ptraj_2d.min_lon_acceleration           = -60; //-6.0; // [m/s²]
//    trj_params.orientation_thresh                      = 40.* M_PI / 180.;

    pthread_mutex_lock(&awp_.chsm_planner_->trajectory_mutex_);
    awp_.chsm_planner_->traj_eval_->setParams(trj_params);
    pthread_mutex_unlock(&awp_.chsm_planner_->trajectory_mutex_);

    glui_->sync_live();
  }
  else if (control == CB_ACTIVATE) {
    awp_.chsm_planner_->resetEmergencyStopState();
    awp_.sendEStopRun();
  }
  else if (control == CB_PAUSE) {
    awp_.sendEStopPause();
  }
  gui3D_forceRedraw();
}

void AWRoadPlannerGUI::initializeGLUIControls() {
  cout << "- Starting GLUI version "<< GLUI_Master.get_version() << endl;
  glui_ = GLUI_Master.create_glui("road_planner");
  glui_->add_statictext("Planner");

  // initialize values
  max_velocity_ = (float)dgc_ms2mph(planner_params_.max_speed_global);
  curvature_slowdown_factor_ = (float)planner_params_.curvature_slowdown_factor;
  const TrajectoryEvaluator::parameters& trj_params = awp_.chsm_planner_->traj_eval_->params();
  urgency_ = (float)trj_params.weight;

  // awp_.chsm_planner_ rollout
  planner_rollout_ = glui_->add_rollout("CHSM Planner Params", true);
  glui_->add_spinner_to_panel(planner_rollout_, "Max Velocity", GLUI_SPINNER_FLOAT,&max_velocity_, CB_UPDATE_PLANNER_PARAMS, gluiControlCBWrapper);
  glui_->add_spinner_to_panel(planner_rollout_, "Curvature Slowdown Factor", GLUI_SPINNER_FLOAT, &curvature_slowdown_factor_, CB_UPDATE_PLANNER_PARAMS, gluiControlCBWrapper);
  glui_->add_spinner_to_panel(planner_rollout_, "Urgency", GLUI_SPINNER_FLOAT, &urgency_, CB_UPDATE_TRAJECTORY_PARAMS, gluiControlCBWrapper);
  //glui_->add_spinner_to_panel(planner_rollout_, "surface_quality_slowdown_factor_", GLUI_SPINNER_FLOAT, &surface_quality_slowdown_factor_, CB_UPDATE_PLANNER_PARAMS, gluiControlCBWrapper);
//  glui_->add_checkbox_to_panel(planner_rollout_, "enable_recovery", &planner_params_. enable_recovery, CB_UPDATE_PLANNER_PARAMS, gluiControlCBWrapper);

  // display rollout
  display_rollout_ = glui_->add_rollout("Display", true);
  glui_->add_checkbox_to_panel(display_rollout_, "RNDF", &show_rndf_, CB_REFRESH, gluiControlCBWrapper);
  glui_->add_checkbox_to_panel(display_rollout_, "Imagery", &show_imagery_, CB_REFRESH, gluiControlCBWrapper);
//  glui_->add_checkbox_to_panel(display_rollout_, "pose history", &show_pose_history_, CB_REFRESH, gluiControlCBWrapper);
  glui_->add_checkbox_to_panel(display_rollout_, "Vehicle", &show_vehicle_, CB_REFRESH, gluiControlCBWrapper);
  glui_->add_checkbox_to_panel(display_rollout_, "Complete Center Line", &show_complete_center_line_, CB_REFRESH, gluiControlCBWrapper);
  glui_->add_checkbox_to_panel(display_rollout_, "Center Line", &show_center_line_, CB_REFRESH, gluiControlCBWrapper);
  glui_->add_checkbox_to_panel(display_rollout_, "Trajectories", &show_trajectories_, CB_REFRESH, gluiControlCBWrapper);
  glui_->add_checkbox_to_panel(display_rollout_, "Best Trajectory", &show_best_trajectory_, CB_REFRESH, gluiControlCBWrapper);
  glui_->add_checkbox_to_panel(display_rollout_, "Obstacle Destinations", &show_obstacle_destinations_, CB_REFRESH,gluiControlCBWrapper);
  glui_->add_checkbox_to_panel(display_rollout_, "Complete Graph", &show_complete_graph_, CB_REFRESH,gluiControlCBWrapper);
  glui_->add_checkbox_to_panel(display_rollout_, "Mission Graph", &show_mission_graph_, CB_REFRESH, gluiControlCBWrapper);
  glui_->add_checkbox_to_panel(display_rollout_, "Complete Mission Graph", &show_complete_mission_graph_, CB_REFRESH, gluiControlCBWrapper);
  glui_->add_checkbox_to_panel(display_rollout_, "Matched Edge", &show_matched_edge_, CB_REFRESH,gluiControlCBWrapper);
//  glui_->add_checkbox_to_panel(display_rollout_, "yellow dot", &show_yellow_dot_, CB_REFRESH, gluiControlCBWrapper);
  glui_->add_checkbox_to_panel(display_rollout_, "Dyn. Obstacles", &show_dynamic_objects_, CB_REFRESH, gluiControlCBWrapper);
  glui_->add_checkbox_to_panel(display_rollout_, "Dyn. Obstacle Prediction", &show_obstacle_predictions_, CB_REFRESH, gluiControlCBWrapper);
  glui_->add_checkbox_to_panel(display_rollout_, "Static Obstacle Map", &show_obstacle_map_, CB_REFRESH, gluiControlCBWrapper);
  glui_->add_checkbox_to_panel(display_rollout_, "Configuration Space", &show_configuration_space_, CB_REFRESH, gluiControlCBWrapper);

  //  glui_->add_checkbox_to_panel(display_rollout_, "pose accuracy", &show_pose_accuracy_, CB_REFRESH, gluiControlCBWrapper);
	glui_->add_checkbox_to_panel(display_rollout_, "Ego Distances", &show_ego_distances_, CB_REFRESH, gluiControlCBWrapper);
	glui_->add_checkbox_to_panel(display_rollout_, "Veh Distances", &show_veh_distances_, CB_REFRESH, gluiControlCBWrapper);
	glui_->add_checkbox_to_panel(display_rollout_, "Inters. Merge Point", &show_intersection_merging_point_, CB_REFRESH, gluiControlCBWrapper);

  glui_->add_button("Activate", CB_ACTIVATE, gluiControlCBWrapper);
  glui_->add_button("Pause", CB_PAUSE, gluiControlCBWrapper);
  glui_->add_button("Quit", CB_QUIT, gluiControlCBWrapper);

  // Link windows to GLUI, and register idle callback
  glui_->set_main_gfx_window(gui3D.window_id);
}

//-------------------------------------------------------------------------------------------
/**
\brief Initialize FBO for offscreen rendering
\param - width, height - size of offscreen render buffer
\return true: success, false: error
\ingroup display
*///-----------------------------------------------------------------------------------------
bool AWRoadPlannerGUI::initFrameBufferObject(const uint32_t fb_width, const uint32_t fb_height, GLuint& frame_buffer, GLuint& fb_texture) {
printf("%s\n", __FUNCTION__);
//Set up a FBO with one texture attachment
glGenFramebuffersEXT(1, &frame_buffer);
glGenTextures(1, &fb_texture);

glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, frame_buffer);
glPixelStorei(GL_PACK_ALIGNMENT, 1);
glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

glBindTexture(GL_TEXTURE_RECTANGLE_ARB, fb_texture);
glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGBA, fb_width, fb_height, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, NULL); // Needed?!?

glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, fb_texture, 0);

GLenum glerr = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
if (glerr != GL_FRAMEBUFFER_COMPLETE_EXT) {
  printf("Error %x: Cannot create frame buffer objects.\n", (unsigned int)glerr);
  return false;
  }

glViewport(0, 0, fb_width, fb_height);

glClearColor(0.0,0.0,0.0,0.0);
//  glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT|GL_ACCUM_BUFFER_BIT|GL_STENCIL_BUFFER_BIT);

glBindTexture(GL_TEXTURE_RECTANGLE_ARB, 0);
glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);

return true;
}



const float AWRoadPlannerGUI::cmap_rb1_red_[256] = {
   1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000,
   1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 0.988235, 0.964706, 0.941176, 0.917647, 0.894118, 0.870588, 0.847059, 0.823530, 0.800000, 0.776470, 0.752941, 0.729412, 0.705882, 0.682353, 0.658824, 0.635294, 0.611765, 0.588235, 0.564706, 0.541176, 0.517647,
   0.494118, 0.470588, 0.447059, 0.423529, 0.400000, 0.376470, 0.352941, 0.329412, 0.305882, 0.282353, 0.258823, 0.235294, 0.211765, 0.188235, 0.164706, 0.141176, 0.117647, 0.094118, 0.070588, 0.047059, 0.023529, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
   0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
   0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
   0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.023530, 0.047059, 0.070588, 0.094118, 0.117647, 0.141176, 0.164706, 0.188235, 0.211765, 0.235295, 0.258824, 0.282353, 0.305882, 0.329412, 0.352941, 0.376470, 0.400000, 0.423530, 0.447059, 0.470588, 0.494118,
   0.517647, 0.541176, 0.564706, 0.588235, 0.611765, 0.635294, 0.658824, 0.682353, 0.705882, 0.729412, 0.752941, 0.776470, 0.800000, 0.823530, 0.847059, 0.870588, 0.894118, 0.917647, 0.941176, 0.964706, 0.988236, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000,
   1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000,
};



const float AWRoadPlannerGUI::cmap_rb1_green_[256] = {
    0.000000, 0.023529, 0.047059, 0.070588, 0.094118, 0.117647, 0.141176, 0.164706, 0.188235, 0.211765, 0.235294, 0.258824, 0.282353, 0.305882, 0.329412, 0.352941, 0.376471, 0.400000, 0.423529, 0.447059, 0.470588, 0.494118, 0.517647, 0.541176, 0.564706, 0.588235, 0.611765, 0.635294, 0.658824, 0.682353, 0.705882, 0.729412,
    0.752941, 0.776471, 0.800000, 0.823529, 0.847059, 0.870588, 0.894118, 0.917647, 0.941177, 0.964706, 0.988235, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000,
    1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000,
    1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000,
    0.988235, 0.964706, 0.941176, 0.917647, 0.894118, 0.870588, 0.847059, 0.823529, 0.800000, 0.776470, 0.752941, 0.729412, 0.705882, 0.682353, 0.658823, 0.635294, 0.611765, 0.588235, 0.564706, 0.541176, 0.517647, 0.494117, 0.470588, 0.447059, 0.423529, 0.400000, 0.376470, 0.352941, 0.329412, 0.305882, 0.282353, 0.258823,
    0.235294, 0.211765, 0.188235, 0.164706, 0.141176, 0.117647, 0.094117, 0.070588, 0.047059, 0.023529, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
};

const float AWRoadPlannerGUI::cmap_rb1_blue_[256] = {
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.023529, 0.047059, 0.070588, 0.094118, 0.117647, 0.141177, 0.164706, 0.188235, 0.211765, 0.235294,
    0.258824, 0.282353, 0.305883, 0.329412, 0.352941, 0.376471, 0.400000, 0.423530, 0.447059, 0.470588, 0.494118, 0.517647, 0.541177, 0.564706, 0.588235, 0.611765, 0.635294, 0.658824, 0.682353, 0.705882, 0.729412, 0.752941, 0.776471, 0.800000, 0.823529, 0.847059, 0.870588, 0.894118, 0.917647, 0.941176, 0.964706, 0.988235,
    1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000,
    1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000,
    1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 0.988236, 0.964706, 0.941176, 0.917647, 0.894118, 0.870588, 0.847059, 0.823530, 0.800000, 0.776470, 0.752941,
    0.729412, 0.705882, 0.682353, 0.658824, 0.635294, 0.611765, 0.588235, 0.564706, 0.541176, 0.517647, 0.494118, 0.470588, 0.447059, 0.423530, 0.400000, 0.376470, 0.352941, 0.329412, 0.305882, 0.282353, 0.258824, 0.235294, 0.211765, 0.188235, 0.164706, 0.141176, 0.117647, 0.094118, 0.070588, 0.047059, 0.023530, 0.000000,
};

const float AWRoadPlannerGUI::cmap_rb2_red_[256] = {
   0.0104, 0.0208, 0.0313, 0.0417, 0.0521, 0.0625, 0.0729, 0.0833, 0.0938, 0.1042, 0.1146, 0.125, 0.1354, 0.1458, 0.1563, 0.1667, 0.1771, 0.1875, 0.1979, 0.2083, 0.2188, 0.2292, 0.2396, 0.25, 0.2604, 0.2708, 0.2813, 0.2917, 0.3021, 0.3125, 0.3229, 0.3333, 0.3438, 0.3542, 0.3646, 0.375, 0.3854, 0.3958, 0.4063, 0.4167, 0.4271, 0.4375, 0.4479, 0.4583, 0.4688, 0.4792, 0.4896, 0.5, 0.5104, 0.5208, 0.5313, 0.5417, 0.5521, 0.5625, 0.5729, 0.5833, 0.5938, 0.6042, 0.6146, 0.625, 0.6354, 0.6458, 0.6563, 0.6667, 0.6771, 0.6875, 0.6979, 0.7083, 0.7188, 0.7292, 0.7396, 0.75, 0.7604, 0.7708, 0.7813, 0.7917, 0.8021, 0.8125, 0.8229, 0.8333, 0.8438, 0.8542, 0.8646, 0.875, 0.8854, 0.8958, 0.9063, 0.9167, 0.9271, 0.9375, 0.9479, 0.9583, 0.9688, 0.9792, 0.9896, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
};

const float AWRoadPlannerGUI::cmap_rb2_green_[256] = {
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0104, 0.0208, 0.0313, 0.0417, 0.0521, 0.0625, 0.0729, 0.0833, 0.0938, 0.1042, 0.1146, 0.125, 0.1354, 0.1458, 0.1563, 0.1667, 0.1771, 0.1875, 0.1979, 0.2083, 0.2188, 0.2292, 0.2396, 0.25, 0.2604, 0.2708, 0.2813, 0.2917, 0.3021, 0.3125, 0.3229, 0.3333, 0.3438, 0.3542, 0.3646, 0.375, 0.3854, 0.3958, 0.4063, 0.4167, 0.4271, 0.4375, 0.4479, 0.4583, 0.4688, 0.4792, 0.4896, 0.5, 0.5104, 0.5208, 0.5313, 0.5417, 0.5521, 0.5625, 0.5729, 0.5833, 0.5938, 0.6042, 0.6146, 0.625, 0.6354, 0.6458, 0.6563, 0.6667, 0.6771, 0.6875, 0.6979, 0.7083, 0.7188, 0.7292, 0.7396, 0.75, 0.7604, 0.7708, 0.7813, 0.7917, 0.8021, 0.8125, 0.8229, 0.8333, 0.8438, 0.8542, 0.8646, 0.875, 0.8854, 0.8958, 0.9063, 0.9167, 0.9271, 0.9375, 0.9479, 0.9583, 0.9688, 0.9792, 0.9896, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
};

const float AWRoadPlannerGUI::cmap_rb2_blue_[256] = {
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0156, 0.0313, 0.0469, 0.0625, 0.0781, 0.0938, 0.1094, 0.125, 0.1406, 0.1563, 0.1719, 0.1875, 0.2031, 0.2188, 0.2344, 0.25, 0.2656, 0.2813, 0.2969, 0.3125, 0.3281, 0.3438, 0.3594, 0.375, 0.3906, 0.4063, 0.4219, 0.4375, 0.4531, 0.4688, 0.4844, 0.5, 0.5156, 0.5313, 0.5469, 0.5625, 0.5781, 0.5938, 0.6094, 0.625, 0.6406, 0.6563, 0.6719, 0.6875, 0.7031, 0.7188, 0.7344, 0.75, 0.7656, 0.7813, 0.7969, 0.8125, 0.8281, 0.8438, 0.8594, 0.875, 0.8906, 0.9063, 0.9219, 0.9375, 0.9531, 0.9688, 0.9844, 1
};

} // namespace vlr
