#include <cmath>

#include <passat_interface.h>

#include "aw_planner.h"

#include <lltransform.h>

  // for gui only
#include <passatmodel.h>
#include <gui3D.h>
#include <scaledTime.h>

#include "aw_planner_gui.h"

using namespace dgc;

vlr::AWRoadPlanner* awp=NULL;

namespace vlr {


    // member initialization
const double AWRoadPlanner::circle_demo_r_ = 35;
const double AWRoadPlanner::circle_demo_start_lat_ = 37.4275144;
const double AWRoadPlanner::circle_demo_start_lon_ = -122.0769586;
const std::string AWRoadPlanner::circle_demo_rndf_filename_ = "./circle_demo_rndf.txt";
const std::string AWRoadPlanner::circle_demo_mdf_filename_ = "./circle_demo_mdf.txt";

const double AWRoadPlanner::static_map_demo_start_lat_ = 37.4305720025051;    // start position
const double AWRoadPlanner::static_map_demo_start_lon_ = -122.1830539998572;
const double AWRoadPlanner::static_map_demo_start_yaw_ = 3;
const std::string AWRoadPlanner::static_map_demo_rndf_filename_ = "./static_map_demo_rndf.txt";
const std::string AWRoadPlanner::static_map_demo_mdf_filename_ = "./static_map_demo_mdf.txt";
const std::string AWRoadPlanner::static_map_demo_map_name_ = "./testmap.png";

void* planner_thread(void*) {
 awp->run();
 return NULL;
}

void AWRoadPlanner::applanixHandler(dgc::ApplanixPose* applanix_pose) {
  received_applanix_pose_ = true;
  updatePose(applanix_pose, &localize_pose_);
}

void AWRoadPlanner::localizePoseHandler(dgc::LocalizePose* localize_pose) {
  received_localize_pose_ = true;
  updatePose(&applanix_pose_, localize_pose);
}

void AWRoadPlanner::perceptionHandler(PerceptionObstacles* obstacles) {
//        printf("Got %u vehicles and %u statics\n", obstacles->num_dynamic_obstacles, obstacles->num_points);
    if (!received_applanix_pose_ || !received_localize_pose_) {
        return;
    }

    if(obstacles->num_points==0 && obstacles->num_dynamic_obstacles==0){
  //    std::cout << "Warning: empty perception message will be discarded.\n";
      return;
    }
//    obstacles->dynamic_obstacle[0].x = 572180.699800 - localize_pose_.x_offset;
//    obstacles->dynamic_obstacle[0].y = 4143020.147515 - localize_pose_.y_offset;
//    obstacles->dynamic_obstacle[0].direction = 4.107189;
    try {
      chsm_planner_->updateVehicles(obstacles->dynamic_obstacle, obstacles->num_dynamic_obstacles, obstacles->timestamp);
      chsm_planner_->updateStaticObstacleMap(obstacles->point, obstacles->num_points, obstacles->timestamp);
    }
    catch(Exception& e) {
      if(!demo_mode_){
        pthread_mutex_lock(&emergency_message_sent_mutex_);
        pthread_cond_wait(&emergency_message_sent_cv_, &emergency_message_sent_mutex_);
        pthread_mutex_unlock(&emergency_message_sent_mutex_);
      }
    quitPlanner();
    }
}

void AWRoadPlanner::trafficLightHandler(TrafficLightList* tll) {
  chsm_planner_->updateTrafficLightStates(tll);
  received_traffic_light_states_=true;
}

void AWRoadPlanner::stopZonesHandler() {
  received_stop_zones_ = true;
}

void AWRoadPlanner::estopHandler() {
  received_estop_ = true;
  std::string estop_cmd;
  if(estop_.estop_code == DGC_ESTOP_RUN) {estop_cmd= "DGC_ESTOP_RUN";}
  else if(estop_.estop_code == DGC_ESTOP_PAUSE) {estop_cmd= "DGC_ESTOP_PAUSE";}
//  else if(estop_.estop_code == DGC_ESTOP_DISABLED) {estop_cmd= "DGC_ESTOP_DISABLED";}

//  printf("received estop command %s\n", estop_cmd.c_str());
}

void AWRoadPlanner::vehicleStateHandler() {
  received_vehicle_state_=true;
//  pthread_mutex_lock(&received_vehicle_state_mutex_);
//  pthread_cond_signal(&received_vehicle_state_cv_);
//  pthread_mutex_unlock(&received_vehicle_state_mutex_);
}

void AWRoadPlanner::fsmRequestHandler() {
//  static PlannerFsmResponse fsm;
//  static int first = 1;
//  fsm.fsmdata=NULL;
//
//  if (first) {
//    strcpy(fsm.host, dgc_hostname());
//    first = 0;
//  }
//  fsm.timestamp = Time::current();
//  //fsm.fsmdata = chsm_planner_->dot_data();
//
//  if (fsm.fsmdata == NULL) return;
//
//  int err = ipc_->Publish(PlannerFsmResponseID, &fsm);
//  TestIpcExit(err, "Could not publish", PlannerFsmResponseID);
//  free(fsm.fsmdata);
}

void* gui_thread(void* ptr) {
  try {
    param_struct_p param = (param_struct_p)ptr;
    AWRoadPlannerGUI* gui = AWRoadPlannerGUI::instance(*awp, param->argc, param->argv);
    gui->run();
  } catch(...) {
    awp->quitPlanner();
  }
  return NULL;
}

AWRoadPlanner::AWRoadPlanner() : last_estop_state_(DGC_ESTOP_DISABLE), ipc_(NULL), pint_(NULL),
                                 show_gui_(true), received_applanix_pose_(false),
                                 received_localize_pose_(false), received_stop_zones_(false),
                                 received_estop_(false), received_traffic_light_states_(false),
                                 received_vehicle_state_(false), data_ready_to_publish_(false),
                                 run_planner_cycle_(true),
                                 wait_for_vehicle_(true), demo_mode_(false), show_static_map_demo_(false),
                                 show_fourway_stop1_demo_(false), show_circle_demo_(false), demo_(NULL),
                                 estop_run_requested_(false), estop_pause_requested_(false),
                                 caught_exception_(false), gui_thread_id_(0), quit_planner_(false) {
    // IPC initialization
  ipc_ = new IpcStandardInterface();

  pint_ = new ParamInterface(ipc_);

  pthread_attr_init(&def_thread_attr_);
  pthread_attr_setdetachstate(&def_thread_attr_, PTHREAD_CREATE_JOINABLE);

  pthread_mutex_init(&estop_mutex_, NULL);
  pthread_mutex_init(&stop_zones_mutex_, NULL);
  pthread_mutex_init(&traffic_light_mutex_, NULL);
  pthread_mutex_init(&vehicle_state_mutex_, NULL);

  pthread_mutex_init(&quit_planner_mutex_, NULL);
  pthread_cond_init(&quit_planner_cv_, NULL);

  pthread_mutex_init(&emergency_message_sent_mutex_, NULL);
  pthread_cond_init(&emergency_message_sent_cv_, NULL);

  pthread_mutex_init(&received_vehicle_state_mutex_, NULL);
  pthread_cond_init(&received_vehicle_state_cv_, NULL);

  memset(&pose_accuracy_, 0, sizeof(pose_accuracy_));

  // initialize trajectory
  trajectory_.num_points = 0;
  trajectory_.points = new TrajectoryPoint2D[MAX_TRAJECTORY_POINTS];
  strcpy(trajectory_.host, dgc_hostname());

    // intialize traffic light poses
  traffic_light_pose_list_.num_traffic_lights = 0;
  traffic_light_pose_list_.light_pose = new TrafficLightPose[MAX_REQUESTED_TRAFFIC_LIGHTS];

    // initialize estop command message
  estop_cmd_.estop_code = DGC_ESTOP_DISABLE;
  estop_cmd_.timestamp = Time::current();
  strcpy(estop_cmd_.host, dgc_hostname());
}

AWRoadPlanner::~AWRoadPlanner()
{

    // in case unhandled exception occured, notify other threads
  quitPlanner();

    // if we quit before gui was initialized properly we will get stuck in gui main loop
  while (gui_thread_id_ == 0 && show_gui_) {
    usleep(100000);
  }

  if(gui_thread_id_ > 0) {
    glutLeaveMainLoop();
    pthread_join(gui_thread_id_, NULL);
  }

  printf("gui thread joined.\n");

  if(planner_thread_id_ > 0) {
    run_planner_cycle_=false;
    pthread_join(planner_thread_id_, NULL);
  }

  printf("planner thread joined.\n");

  if(!demo_mode_) {ipc_->Disconnect();}
  if(chsm_planner_) {
    chsm_planner_->terminate(); // should not be necessary but doesn't hurt
    delete chsm_planner_;
  }
  if(ipc_) {delete ipc_;}

  pthread_attr_destroy(&def_thread_attr_);

  pthread_mutex_destroy(&estop_mutex_);
  pthread_mutex_destroy(&stop_zones_mutex_);
  pthread_mutex_destroy(&traffic_light_mutex_);
  pthread_mutex_destroy(&vehicle_state_mutex_);

  pthread_mutex_destroy(&quit_planner_mutex_);
  pthread_cond_destroy(&quit_planner_cv_);

  pthread_mutex_destroy(&emergency_message_sent_mutex_);
  pthread_cond_destroy(&emergency_message_sent_cv_);

  pthread_mutex_destroy(&received_vehicle_state_mutex_);
  pthread_cond_destroy(&received_vehicle_state_cv_);
}

void AWRoadPlanner::initialize(int argc, char** argv) {
  try {
    if (ipc_->ConnectLocked(argv[0]) < 0) {
      dgc_fatal_error("Could not connect to IPC network.");
    }

    IpcMessageID messages[] = { TrajectoryPoints2DID, PassatTurnSignalID, TrafficLightPoseListMsgID, EstopSoftstopID, HciAudioID };

    ipc_->DefineMessageArray(messages, sizeof(messages) / sizeof(*messages));

    // read parameters for on-road and off-road driving
    readParameters(*pint_, argc, argv);

    if (argc > 1) {
      for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "nogui") == 0) {
          show_gui_ = false;
        }
        else if (strcmp(argv[i], "nowaitforcar") == 0) {
          wait_for_vehicle_ = false;
        }
        else if (strcmp(argv[i], "circledemo") == 0) {
          demo_mode_ = true;
          show_circle_demo_ = true;
          rndf_filename_ = circle_demo_rndf_filename_;
          mdf_filename_ = circle_demo_mdf_filename_;
          demo_ = new CircleDemo(rndf_filename_, mdf_filename_, circle_demo_start_lat_, circle_demo_start_lon_,
              circle_demo_r_);
        }
        else if (strcmp(argv[i], "staticdemo") == 0) {
          demo_mode_ = true;
          show_static_map_demo_ = true;
          rndf_filename_ = static_map_demo_rndf_filename_;
          mdf_filename_ = static_map_demo_mdf_filename_;
          demo_ = new StaticMapDemo(rndf_filename_, mdf_filename_, static_map_demo_map_name_,
              static_map_demo_start_lat_, static_map_demo_start_lon_, static_map_demo_start_yaw_);
        }
      }
    }

    // don't wait for vehicle to shift when we run a simulation
    if (demo_mode_) {
      wait_for_vehicle_ = false;
    }

    //initialize planner core
    chsm_planner_ = new ChsmPlanner(rndf_filename_, mdf_filename_, static_obstacle_map_size_x_,
        static_obstacle_map_size_y_, static_obstacle_map_resolution_);

    // set everything demo related with dependencies on planner here
    if (demo_mode_) {
      std::string zone;
      if (show_circle_demo_) {
        static_cast<CircleDemo*> (demo_)->setFakeTrackerParams(chsm_planner_->traj_eval_->params().checked_horizon,
            chsm_planner_->traj_eval_->params().time_sample_res);
        latLongToUtm(circle_demo_start_lat_, circle_demo_start_lon_, &demo_start_point_.x, &demo_start_point_.y, zone);
      }
      else if (show_static_map_demo_) {
        latLongToUtm(static_map_demo_start_lat_, static_map_demo_start_lon_, &demo_start_point_.x,
            &demo_start_point_.y, zone);
      }

      demo_start_point_.t = Time::current();
      demo_->updatePoses(true, demo_start_point_, applanix_pose_, localize_pose_);
      applanixHandler(&applanix_pose_);
      localizePoseHandler(&localize_pose_);
      demo_->updateObstaclePredictions(Time::current(), chsm_planner_->predicted_obstacles_);
      perceptionHandler(const_cast<PerceptionObstacles*> (&demo_->getObstacleMessage()));
    }

    // if we are in demo mode we do not want messages
    if (!demo_mode_) {
      // initialize grid map
      PerceptionMapResetCommand(ipc_);

      //      // awp requests
      //    ipc_->Subscribe(PlannerFsmRequestID, this, &AWRoadPlanner::fsmRequestHandler, DGC_SUBSCRIBE_ALL);

      // poses
      ipc_->Subscribe(ApplanixPoseID, this, &AWRoadPlanner::applanixHandler, DGC_SUBSCRIBE_LATEST);
      ipc_->Subscribe(LocalizePoseID, this, &AWRoadPlanner::localizePoseHandler, DGC_SUBSCRIBE_LATEST);

      // perception output
      ipc_->Subscribe(PerceptionObstaclesID, this, &AWRoadPlanner::perceptionHandler, DGC_SUBSCRIBE_LATEST,
          &chsm_planner_->dyn_obstacles_mutex_);

      // estop
      ipc_->Subscribe(EstopStatusID, &estop_, this, &AWRoadPlanner::estopHandler, DGC_SUBSCRIBE_LATEST, &estop_mutex_);

      // passat status (to see if car is in right gear on activation)
      ipc_->Subscribe(PassatStateID, &vehicle_state_, this, &AWRoadPlanner::vehicleStateHandler, DGC_SUBSCRIBE_LATEST, &vehicle_state_mutex_);

      // traffic lights
      ipc_->Subscribe(TrafficLightListMsgID, this, &AWRoadPlanner::trafficLightHandler, DGC_SUBSCRIBE_LATEST);
    }

    if (show_gui_) {
      cmd_params_.argc = argc;
      cmd_params_.argv = argv;
      pthread_create(&gui_thread_id_, &def_thread_attr_, gui_thread, &cmd_params_);
    }
  }
  catch (Exception& e) {
    caught_exception_=true;
    caught_exception_text_=e.what();
    std::cout << e.what() << std::endl;
    if(!demo_mode_){
      pthread_mutex_lock(&emergency_message_sent_mutex_);
      pthread_cond_wait(&emergency_message_sent_cv_, &emergency_message_sent_mutex_);
      pthread_mutex_unlock(&emergency_message_sent_mutex_);
    }
    quitPlanner();
  }
}

void AWRoadPlanner::run() {
  try {
    double t1, t2, extra;
    bool init = true; // for demos

      if (!chsm_planner_->poseAvailable()) {
      std::cout << "Waiting for first Applanix and localize message...";
      while (!chsm_planner_->poseAvailable()) {
        if (!run_planner_cycle_) {return;}
        usleep(100000);
      }
      std::cout << "ok..position received.\n";
    }

    chsm_planner_->start();

    printf("CHSM planner started...\n");

    // awp loop
    while (run_planner_cycle_) {

      //    std::cout << "pose is " << pose_.x << ", " << pose_.y << " / " << lat_ << ", " << lon_ << " (yaw: " << pose_.yaw << ")\n";
      if (demo_mode_) {
        TrajectoryPoint2D current_trajectory_point;
        init = false;
        try {
          chsm_planner_->traj_eval_->getCurrentTrajectoryPoint(Time::current(), current_trajectory_point);
        }
        catch (vlr::Exception& e) {
          //        std::cout << e.what() << "\n";
          demo_start_point_.t = Time::current();
          current_trajectory_point = demo_start_point_;
          init = true;
        }

        demo_->updatePoses(init, current_trajectory_point, applanix_pose_, localize_pose_);

        applanixHandler(&applanix_pose_);
        localizePoseHandler(&localize_pose_);

        if (show_circle_demo_) {
          if (!init) {
            pthread_mutex_lock(&chsm_planner_->dyn_obstacles_mutex_);
            demo_->updateObstaclePredictions(current_trajectory_point.t, chsm_planner_->predicted_obstacles_);
            pthread_mutex_unlock(&chsm_planner_->dyn_obstacles_mutex_);

            perceptionHandler(const_cast<PerceptionObstacles*> (&demo_->getObstacleMessage()));

            pthread_mutex_unlock(&chsm_planner_->topology_mutex_);
            pthread_mutex_unlock(&chsm_planner_->dyn_obstacles_mutex_);
          }
        }
        else if (show_static_map_demo_) {
          pthread_mutex_lock(&chsm_planner_->dyn_obstacles_mutex_);
          demo_->updateObstaclePredictions(current_trajectory_point.t, chsm_planner_->predicted_obstacles_);
          pthread_mutex_unlock(&chsm_planner_->dyn_obstacles_mutex_);
          perceptionHandler(const_cast<PerceptionObstacles*> (&demo_->getObstacleMessage()));
        }
      }

      t1 = Time::current();

      // copy estop data
      //todo: is it allowed to call activate() if there was no actual state change?
      pthread_mutex_lock(&estop_mutex_);
      if (received_estop_) {
        if (estop_.estop_code == DGC_ESTOP_RUN && last_estop_state_ != DGC_ESTOP_RUN) {
          if (wait_for_vehicle_ && !demo_mode_) {
            pthread_mutex_unlock(&estop_mutex_);
            waitForVehicle();
            pthread_mutex_lock(&estop_mutex_);
          }
          chsm_planner_->activate();
        }
        else if (estop_.estop_code == DGC_ESTOP_PAUSE && last_estop_state_ != DGC_ESTOP_PAUSE) {
          chsm_planner_->pause();
        }
        last_estop_state_ = estop_.estop_code;
      }
      pthread_mutex_unlock(&estop_mutex_);

      if (demo_mode_ && estop_run_requested_) {
        chsm_planner_->activate();
        estop_run_requested_ = false;
      }
      else if (demo_mode_ && estop_pause_requested_) {
        chsm_planner_->pause();
        estop_pause_requested_ = false;
      }

      // do one planner iteration
 //     std::cout << "." << std::flush;
      pthread_mutex_lock(&chsm_planner_->dyn_obstacles_mutex_);
      pthread_mutex_lock(&chsm_planner_->topology_mutex_);
      chsm_planner_->process();
      pthread_mutex_unlock(&chsm_planner_->topology_mutex_);
      pthread_mutex_unlock(&chsm_planner_->dyn_obstacles_mutex_);

      // ipc hangs in demo mode..seems to be related to Subscribe command (that is not used for demo)
      if (!demo_mode_) {
        data_ready_to_publish_ = true;
      }

      if(chsm_planner_->emergencyStopInitiated()) {
        if(!demo_mode_){
          pthread_mutex_lock(&emergency_message_sent_mutex_);
          pthread_cond_wait(&emergency_message_sent_cv_, &emergency_message_sent_mutex_);
          pthread_mutex_unlock(&emergency_message_sent_mutex_);
        }
//        quitPlanner();
//        break;
      }

        // ... and loop
      t2 = Time::current();
      extra = 1.0 / planner_hz_ - (t2 - t1);
      if (extra > 0) {
        usleep((int) rint(extra * 1e6));
      }
      else if (extra < 0) {
        printf("TIMING ERROR - SYSTEM TOO SLOW: planner cycle time: %f\n", t2 - t1);
      }
    }
  }
  catch (Exception& e) {
    caught_exception_=true;
    caught_exception_text_=e.what();
    data_ready_to_publish_ = true;
    std::cout << e.what() << std::endl;
    if(!demo_mode_){
      pthread_mutex_lock(&emergency_message_sent_mutex_);
      pthread_cond_wait(&emergency_message_sent_cv_, &emergency_message_sent_mutex_);
      pthread_mutex_unlock(&emergency_message_sent_mutex_);
    }
    quitPlanner();
  }

  try {
    chsm_planner_->terminate();
  }
  catch(...) {
    // nothing left to do, just exit regularly
  }
}

void AWRoadPlanner::publishEstopRequest() {

  if (estop_run_requested_) {
    estop_run_requested_ = false;
    estop_cmd_.estop_code = DGC_ESTOP_RUN;
  }
  else if (estop_pause_requested_) {
    estop_pause_requested_ = false;
    estop_cmd_.estop_code = DGC_ESTOP_PAUSE;
  }
  else {
    return;
  }

  estop_cmd_.timestamp = Time::current();
  int err = ipc_->Publish(EstopSoftstopID, &estop_cmd_);

  if (err != IPC_OK) {
    if (err == IPC_Timeout) {
      printf("IPC Timeout occurred. Could not publish %s (error code %i)\n", EstopSoftstopID.name, IPC_errno);
    }
    else if (IPC_errno != IPC_No_Error) {
      printf("IPC ERROR occurred. Could not publish %s (error code %i)\n", EstopSoftstopID.name, IPC_errno);
    }
  }
}

void AWRoadPlanner::publish() {

  int turn_signal_state;

  publishEstopRequest();

  switch(chsm_planner_->vehiclecmd.turnsignal) {
    case TURN_SIGNAL_RIGHT:
      turn_signal_state = DGC_PASSAT_TURN_SIGNAL_RIGHT;
      break;

    case TURN_SIGNAL_LEFT:
      turn_signal_state = DGC_PASSAT_TURN_SIGNAL_LEFT;
      break;

    case TURN_SIGNAL_NONE:
      turn_signal_state = DGC_PASSAT_TURN_SIGNAL_NONE;
      break;

    default:
      turn_signal_state = DGC_PASSAT_TURN_SIGNAL_BOTH;
  }

  PassatTurnSignalCommand(ipc_, turn_signal_state);

  publishTrajectory(chsm_planner_->trajectory_points_);

  if(chsm_planner_->publish_traffic_lights_) {
    publishTrafficLightRequest();
  }

  if(chsm_planner_->emergencyStopInitiated()) {
    hci_message_string_ = "Emergency stop initiated.";
    publishEmergencyMessage();
    pthread_mutex_lock(&emergency_message_sent_mutex_);
    pthread_cond_signal(&emergency_message_sent_cv_);
    pthread_mutex_unlock(&emergency_message_sent_mutex_);
  }

  if(caught_exception_) {
    hci_message_string_ = caught_exception_text_;
    publishEmergencyMessage();
    pthread_mutex_lock(&emergency_message_sent_mutex_);
    pthread_cond_signal(&emergency_message_sent_cv_);
    pthread_mutex_unlock(&emergency_message_sent_mutex_);
  }

  // publish status messages
}

void AWRoadPlanner::publishEmergencyMessage() {

  hci_message_.msg=const_cast<char*>(hci_message_string_.c_str());

  int err = ipc_->Publish(HciAudioID, &hci_message_);

  if(err != IPC_OK) {
    if(err == IPC_Timeout) {
      printf("IPC Timeout occurred. Could not publish %s (error code %i)\n", HciAudioID.name, IPC_errno);
    }
    else if(IPC_errno != IPC_No_Error) {
      printf("IPC ERROR occurred. Could not publish %s (error code %i)\n", HciAudioID.name, IPC_errno);
    }
  }
}

void AWRoadPlanner::publishTrajectory(const std::vector<TrajectoryPoint2D>& trajectory_points) {

  if (trajectory_points.size() == 0) {
    return;
  }

  double offset_x, offset_y;
  chsm_planner_->currentLocalizeOffsets(offset_x, offset_y);

  pthread_mutex_lock(&chsm_planner_->trajectory_mutex_);
  trajectory_.num_points = std::min((int)trajectory_points.size(), MAX_TRAJECTORY_POINTS);

  for (unsigned int i = 0; i < (unsigned int) trajectory_.num_points; i++) {
    trajectory_.points[i] = trajectory_points[i];
//    recalcForFrontAxle(trajectory_points[i], trajectory_.points[i]);
    trajectory_.points[i].x += -offset_x;
    trajectory_.points[i].y += -offset_y;
  }

  int err = ipc_->Publish(TrajectoryPoints2DID, &trajectory_);

  pthread_mutex_unlock(&chsm_planner_->trajectory_mutex_);

  if(err != IPC_OK) {
    if(err == IPC_Timeout) {
      printf("IPC Timeout occurred. Could not publish %s (error code %i)\n", TrajectoryPoints2DID.name, IPC_errno);
    }
    else if(IPC_errno != IPC_No_Error) {
      printf("IPC ERROR occurred. Could not publish %s (error code %i)\n", TrajectoryPoints2DID.name, IPC_errno);
    }
  }
}

void AWRoadPlanner::publishTrafficLightRequest() {

  pthread_mutex_lock(&chsm_planner_->traffic_light_poses_mutex_);

  traffic_light_pose_list_.num_traffic_lights = std::min((int)chsm_planner_->traffic_light_poses_.size(), MAX_REQUESTED_TRAFFIC_LIGHTS);

  if(traffic_light_pose_list_.num_traffic_lights == 0) {
    std::cout << "Number of traffic lights to request is 0 (Cannot publish request).\n";
    pthread_mutex_unlock(&chsm_planner_->traffic_light_poses_mutex_);
    return;
  }

  if (traffic_light_pose_list_.num_traffic_lights != (int)chsm_planner_->traffic_light_poses_.size()) {
    std::cout << "Warning: Too many traffic lights requested (" << chsm_planner_->traffic_light_poses_.size()
              << ", only first " << MAX_REQUESTED_TRAFFIC_LIGHTS << " can be handled\n";
  }

  for (int i = 0; i < traffic_light_pose_list_.num_traffic_lights; i++) {
    traffic_light_pose_list_.light_pose[i] = chsm_planner_->traffic_light_poses_[i];
    printf("Requesting state for traffic light %s\n", traffic_light_pose_list_.light_pose[i].name);
  }

  pthread_mutex_unlock(&chsm_planner_->traffic_light_poses_mutex_);

  int err = ipc_->Publish(TrafficLightPoseListMsgID, &traffic_light_pose_list_);

  if(err != IPC_OK) {
    if(err == IPC_Timeout) {
      printf("IPC Timeout occurred. Could not publish %s (error code %i)\n", TrafficLightPoseListMsgID.name, IPC_errno);
    }
    else if(IPC_errno != IPC_No_Error) {
      printf("IPC ERROR occurred. Could not publish %s (error code %i)\n", TrafficLightPoseListMsgID.name, IPC_errno);
    }
  }
}

void AWRoadPlanner::updatePose(dgc::ApplanixPose* applanix_pose, dgc::LocalizePose* localize_pose) {
   // TODO: associate localize offset with correct applanix pose
  pthread_mutex_lock(&chsm_planner_->pose_mutex_);

  if(localize_pose != &localize_pose_) {
    localize_pose_ = *localize_pose;
  }

  if (!received_applanix_pose_) {
    pthread_mutex_unlock(&chsm_planner_->pose_mutex_);
    return;
  } // localize pose alone doesn't help..and anyway..shouldn't occur

  if(applanix_pose != &applanix_pose_) {
    applanix_pose_ = *applanix_pose;
  }
  double offset_x=0, offset_y=0;

  if (received_localize_pose_) {
//    printf("received xoff: %f, yoff: %f\n", localize_pose_.x_offset, localize_pose_.y_offset);
    offset_x = localize_pose_.x_offset;
    offset_y = localize_pose_.y_offset;
    utmToLatLong(applanix_pose_.smooth_x+offset_x, applanix_pose_.smooth_y+offset_y, localize_pose_.utmzone, &lat_, &lon_);
  }
  else { // that's the best we can do without localize pose
    lat_ = applanix_pose_.latitude;
    lon_ = applanix_pose_.longitude;
    char utm_zone[4];
    latLongToUtm(lat_, lon_, &offset_x, &offset_y, utm_zone);
    offset_x -= applanix_pose_.smooth_x;
    offset_y -= applanix_pose_.smooth_y;
  }

  pthread_mutex_unlock(&chsm_planner_->pose_mutex_);
    // the localize offset should vary more slowly compared to pose data
    // therefore use applanix pose timestamp until proper interpolation is in place
  if(chsm_planner_) {
//    static double t_old=Time::current(), t_old2=Time::current(), t_start=Time::current();
//    static uint32_t counter=0;
//    static uint32_t counter2=0;
//    double t=Time::current();
//    if(t-t_old >1) {
//    printf("count: %i freq: %f\n", counter, 1./(counter+.001));
//    counter=0;
//    t_old=t;
//    }
//    counter++;
// //   printf("applanix update: delta t = %f\n", t-t_old2);
//
//    if(t-t_old2 > 1./100) {counter2++;
////	printf("Warning: slow applanix update: delta t = %.4f, count: %u, freq: %.4f\n", t-t_old2, counter2, counter2/(t-t_start));
//}
//    t_old2=t;
////    if(t-t_old > 1./10) {printf("Warning: SUPER slow applanix update: delta t = %f\n", t-t_old);}
    chsm_planner_->updateRobot(applanix_pose_.timestamp, applanix_pose_.smooth_x, applanix_pose_.smooth_y, offset_x, offset_y,
                                applanix_pose_.yaw, applanix_pose_.speed, applanix_pose_.a_x, applanix_pose_.a_y, applanix_pose_.a_z, applanix_pose_.ar_yaw);
  }

  gui3D_forceRedraw();
}

void AWRoadPlanner::start() {
  pthread_create(&planner_thread_id_, &def_thread_attr_, planner_thread, NULL);
  if(!demo_mode_) {

  while (ipc_->Sleep(AW_PLANNER_IPC_TIMEOUT_MS) >=0 && !quit_planner_) //!= IPC_Error) // SleepUntilClear?!?
    {
    if(data_ready_to_publish_) {
        publish();
        data_ready_to_publish_ = false;
    }
  //  switch(sendIPCMessage)
  //    {
  //    case CBO_SEND_IPC_RESET_COMMAND:
  //      SendPlaybackCommand(ipc, DGC_PLAYBACK_COMMAND_RESET, 0, -1);
  //      sendIPCMessage=CBO_NO_IPC_COMMAND;
  //      break;
  //
  //    case CBO_SEND_IPC_REWIND_COMMAND:
  //      SendPlaybackCommand(ipc, DGC_PLAYBACK_COMMAND_REWIND, 10000, -1);
  //      sendIPCMessage=CBO_NO_IPC_COMMAND;
  //      break;
  //
  //    case CBO_SEND_IPC_PLAY_COMMAND:
  //      SendPlaybackCommand(ipc, DGC_PLAYBACK_COMMAND_PLAY, 0, -1);
  //      sendIPCMessage=CBO_NO_IPC_COMMAND;
  //      break;
  //
  //    case CBO_SEND_IPC_PAUSE_COMMAND:
  //      SendPlaybackCommand(ipc, DGC_PLAYBACK_COMMAND_STOP, 0, -1);
  //      sendIPCMessage=CBO_NO_IPC_COMMAND;
  //      break;
  //
  //    case CBO_SEND_IPC_FORWARD_COMMAND:
  //      SendPlaybackCommand(ipc, DGC_PLAYBACK_COMMAND_FORWARD, 10000, -1);
  //      sendIPCMessage=CBO_NO_IPC_COMMAND;
  //      break;
  //
  //    case CBO_SEND_IPC_STOP_COMMAND:
  //      SendPlaybackCommand(ipc, DGC_PLAYBACK_COMMAND_STOP, 0, -1);
  //      SendPlaybackCommand(ipc, DGC_PLAYBACK_COMMAND_RESET, 0, -1);
  //      sendIPCMessage=CBO_NO_IPC_COMMAND;
  //      break;
  //    }
    }
  }
  else {
    pthread_mutex_lock(&quit_planner_mutex_);
    pthread_cond_wait(&quit_planner_cv_, &quit_planner_mutex_);
    pthread_mutex_unlock(&quit_planner_mutex_);
  }

}

void AWRoadPlanner::readParameters(ParamInterface &pint, int argc, char** argv) {

  char* rndf=NULL, *mdf=NULL;

  Param param[] = {
    {"rndf", "rndf_file", DGC_PARAM_FILENAME, &rndf, 0, NULL},
    {"rndf", "mdf_file", DGC_PARAM_FILENAME, &mdf, 0, NULL},
    {"planner", "hz", DGC_PARAM_DOUBLE, &planner_hz_, 0, NULL},
    {"perception", "map_size_x", DGC_PARAM_DOUBLE, &static_obstacle_map_size_x_, 0, NULL},
    {"perception", "map_size_y", DGC_PARAM_DOUBLE, &static_obstacle_map_size_y_, 0, NULL},
    {"perception", "map_resolution", DGC_PARAM_DOUBLE, &static_obstacle_map_resolution_, 0, NULL},
  };

  pint.InstallParams(argc, argv, param, sizeof(param) / sizeof(param[0]));
  rndf_filename_ = rndf;
  mdf_filename_ = mdf;
  planner_hz_ = 1.0/AW_PLANNER_PUBLISH_CYCLE_TIME;
}

void AWRoadPlanner::waitForVehicle() {
  printf("%s\n",__FUNCTION__);
  if(!received_vehicle_state_) {usleep(10000);}
//  pthread_mutex_lock(&received_vehicle_state_mutex_);
//  pthread_cond_wait(&received_vehicle_state_cv_, &received_vehicle_state_mutex_);
//  pthread_mutex_unlock(&received_vehicle_state_mutex_);

  printf("Waiting for passat to respond with FORWARD_GO...\n");
//  pthread_mutex_lock(&vehicle_state_mutex_);
  int state = vehicle_state_.fsm_state;
//  pthread_mutex_unlock(&vehicle_state_mutex_);
  while(state != FORWARD_GO) {
 // pthread_mutex_lock(&vehicle_state_mutex_);
  state = vehicle_state_.fsm_state;
//  pthread_mutex_unlock(&vehicle_state_mutex_);
    switch(state) {
    case PAUSEFOR_STOP_VEHICLE:
       printf("Got state PAUSEFOR_STOP_VEHICLE\n");
       break;

    case PAUSEFOR_PRESHIFT_WAIT:
       printf("Got state PPAUSEFOR_PRESHIFT_WAIT\n");
       break;

    case PAUSEFOR_SHIFT_TO_PARK:
       printf("Got state PAUSEFOR_SHIFT_TO_PARK\n");
       break;

    case PAUSEFOR_WAIT_WITH_BRAKE:
       printf("Got state PAUSEFOR_WAIT_WITH_BRAKE\n");
       break;

    case PAUSEFOR_ENABLE_PARKING_BRAKE:
       printf("Got state PAUSEFOR_ENABLE_PARKING_BRAKE\n");
       break;

    case PAUSEFOR_WAIT:
       printf("Got state PAUSEFOR_WAIT\n");
       break;

    case PAUSEFOR_ENABLE_BRAKE:
       printf("Got state PAUSEFOR_ENABLE_BRAKE\n");
       break;

    case PAUSEFOR_DISABLE_PARKING_BRAKE:
       printf("Got state PAUSEFOR_DISABLE_PARKING_BRAKE\n");
       break;

    case PAUSEFOR_SHIFT_TO_DRIVE:
       printf("Got state PAUSEFOR_SHIFT_TO_DRIVE\n");
       break;

    case PAUSEFOR_WAIT_5SEC:
       printf("Got state PAUSEFOR_WAIT_5SEC\n");
       break;

    case FORWARD_STOP_VEHICLE:
       printf("Got state FORWARD_STOP_VEHICLE\n");
       break;

    case FORWARD_PRESHIFT_WAIT:
       printf("Got state FORWARD_PRESHIFT_WAIT\n");
       break;

    case FORWARD_SHIFT_TO_DRIVE:
       printf("Got state FORWARD_SHIFT_TO_DRIVE\n");
       break;

    case FORWARD_GO:
       printf("There we go :-D\n");
       break;
    default:
       printf("Got state %i\n", state);
    }
  usleep(100000);
  }
}

} // namespace vlr

void shutdown_handler(int x) {
  if (x == SIGINT) {
    fprintf(stderr, "Shutting down normally...\n");
    exit(0);
  }
  else if (x == SIGSEGV || x == SIGQUIT || x == SIGABRT || x == SIGBUS) {
    fprintf(stderr, "Shutting down (almost) normally (signal %i)...\n", x);
    exit(0);
  }
}

int main(int argc, char **argv) {
  signal(SIGINT, shutdown_handler);

  vlr::Time::scale(1);
  awp = new vlr::AWRoadPlanner;

  awp->initialize(argc, argv);
  awp->start();

  delete awp;
  return 0;
}
