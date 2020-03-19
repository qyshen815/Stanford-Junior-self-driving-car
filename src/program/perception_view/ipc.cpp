#include <IL/il.h>

#include <ipc_std_interface.h>
#include <traffic_light_messages.h>

#include "view.h"
#include "trafficlights.h"

extern double darpa_x_offset, darpa_y_offset;
extern double google_x_offset, google_y_offset;
extern uint32_t img_id;

using namespace vlr::rndf;

pthread_mutex_t traj2D_mutex =
PTHREAD_MUTEX_INITIALIZER;
vlr::TrajectoryPoints2D trajectory_points;

/************************************************************************
 *
 *   IPC HANDLER
 *
 ************************************************************************/

pthread_mutex_t pose_mutex;

void estop_status_handler(EstopStatus *msg) {
  estop.estop_code = msg->estop_code;
  estop.timestamp = applanix_current_pose()->timestamp;
}

void error_error_handler(ErrorString *status) {
  int i, n = MESSAGE_BUFFER_SIZE;

  pthread_mutex_lock(&message_buffer_mutex);
  for (i = n - 1; i > 0; i--) {
    strcpy(error_buffer[i].string, error_buffer[i - 1].string);
    error_buffer[i].timestamp = error_buffer[i - 1].timestamp;
  }
  strncpy(error_buffer[0].string, status->string, MAX_MESSAGE_LEN);
  if (dgc_get_time() - error_buffer[1].timestamp < 0.3) {
    error_buffer[0].timestamp = error_buffer[1].timestamp;
  }
  else {
    error_buffer[0].timestamp = dgc_get_time();
  }
  pthread_mutex_unlock(&message_buffer_mutex);
}

void error_status_handler(ErrorString *status) {
  int i, n = MESSAGE_BUFFER_SIZE;

  pthread_mutex_lock(&message_buffer_mutex);
  for (i = n - 1; i > 0; i--) {
    strcpy(message_buffer[i].string, message_buffer[i - 1].string);
    message_buffer[i].timestamp = message_buffer[i - 1].timestamp;
  }
  strncpy(message_buffer[0].string, status->string, MAX_MESSAGE_LEN);
  if (dgc_get_time() - error_buffer[1].timestamp < 0.3) {
    error_buffer[0].timestamp = error_buffer[1].timestamp;
  }
  else {
    message_buffer[0].timestamp = dgc_get_time();
  }
  pthread_mutex_unlock(&message_buffer_mutex);
}

void ghost_car_handler(void) {
  char utmzone[5] = "10S ";
  vlr::latLongToUtm(ghost_car.lat, ghost_car.lon, &ghost_car_x, &ghost_car_y, utmzone);
}

void simulator_handler(void) {
  int i;

  SimulatorVehiclePose *p;
  ApplanixPose pose;

  if (show_simulator && camera_lock) {
    p = &(simulator_groundtruth.vehicle[show_car_nr]);
    pose.smooth_x = p->x - loc_pose.x_offset;
    pose.smooth_y = p->y - loc_pose.y_offset;
    pose.smooth_z = 0.0;
    pose.yaw = p->theta;
    pose.pitch = 0.0;
    pose.roll = 0.0;
    gps_offset_x = 0.0;
    gps_offset_y = 0.0;
    global_x = p->x;
    global_y = p->y;
    applanix_history_add(&pose);

  }
  for (i = 0; i < simulator_groundtruth.num_vehicles; i++) {
    if (simulator_groundtruth.vehicle[i].forward_accel_warning) {
      sim_car_status[i].forward_accel_warning_time = dgc_get_time() + SIM_STATUS_WARNING_TIME;
    }
    if (simulator_groundtruth.vehicle[i].collision_warning) {
      sim_car_status[i].collision_warning_time = dgc_get_time() + SIM_STATUS_WARNING_TIME;
    }
    if (simulator_groundtruth.vehicle[i].lateral_accel_warning) {
      sim_car_status[i].lateral_accel_warning_time = dgc_get_time() + SIM_STATUS_WARNING_TIME;
    }
    if (simulator_groundtruth.vehicle[i].plan_warning) {
      sim_car_status[i].plan_warning_time = dgc_get_time() + SIM_STATUS_WARNING_TIME;
    }
  }
}

void simulator_tag_handler(SimulatorTag *tags) {
  int i, n;
  char info[20], name[20];
  for (i = 0; i < tags->num_vehicles; i++) {
    if (i < MAX_NUM_SIM_CARS) {
      n = sscanf(tags->tag[i], "{%[^}]}%[^\"]", info, name);
      if (n == 2) {
#ifdef SIMULATOR_FX
        sim_car_status[i].model_nr = dgc_gl_model_name_to_id(info);
#endif
        strncpy(sim_car_status[i].name, name, 20);
      }
      else {
        strncpy(sim_car_status[i].name, tags->tag[i], 20);
      }
    }
  }
}

void applanix_handler(ApplanixPose *pose) {
  static double last_time = 0.0;
  static int applanix_ctr = 0;
  double time, delta_s;

  pthread_mutex_lock(&pose_mutex);

   static double t_old=dgc_get_time(), t_old2=dgc_get_time(), t_start=dgc_get_time();
    static uint32_t counter=0;
    static uint32_t counter2=0;
    double t=dgc_get_time();
    if(t-t_old >1) {
    printf("count: %i freq: %f\n", counter, 1./(counter+.001));
    counter=0;
    t_old=t;
    }
    counter++;
 //   printf("applanix update: delta t = %f\n", t-t_old2);
    
    if(t-t_old2 > 1./100) {counter2++;
	printf("Warning: slow applanix update: delta t = %.4f, count: %u, freq: %.4f\n", t-t_old2, counter2, counter2/(t-t_start));
}
    t_old2=t;
//    if(t-t_old > 1./10) {printf("Warning: SUPER slow applanix update: delta t = %f\n", t-t_old);}

  time = dgc_get_time();
  delta_s = time - last_time;
  applanix_ctr++;
  if (delta_s > 1.0) {
    applanix_hz = applanix_ctr / delta_s;
    applanix_ctr = 0;
    last_time = time;
  }
  if (!show_simulator) {
    vlr::latLongToUtm(pose->latitude, pose->longitude, &applanix_x, &applanix_y, utmzone);
    gps_offset_x = loc_pose.x_offset - (applanix_x - pose->smooth_x);
    gps_offset_y = loc_pose.y_offset - (applanix_y - pose->smooth_y);
    applanix_history_add(pose);
    global_x = pose->smooth_x + loc_pose.x_offset;
    global_y = pose->smooth_y + loc_pose.y_offset;
    display_check_camera(pose);
  }
  pthread_mutex_unlock(&pose_mutex);
}

void planner_goal_handler(PlannerGoal *msg) {
  planner_goal.lat = msg->goal_lat;
  planner_goal.lon = msg->goal_lon;
  planner_goal.theta = msg->goal_theta;
  vlr::latLongToUtm(planner_goal.lat, planner_goal.lon, &planner_goal.utm_x, &planner_goal.utm_y, planner_goal.utm_zone);
  planner_goal.goal_set = 1;
  fprintf(stderr, "# INFO: received planner goal\n");
}

void ldlrs1_handler(void) {
  static double last_time = 0.0;
  static int firsttime = 1;
  double time;
  if (firsttime) {
    time = dgc_get_time();
    firsttime = 0;
  }
  else {
    time = dgc_get_time();
    ldlrs1_hz = dgc_avg_times(time - last_time, 1, 3);
  }
  last_time = time;
}

void ldlrs2_handler(void) {
  static double last_time = 0.0;
  static int firsttime = 1;
  double time;
  if (firsttime) {
    time = dgc_get_time();
    firsttime = 0;
  }
  else {
    time = dgc_get_time();
    ldlrs2_hz = dgc_avg_times(time - last_time, 1, 4);
  }
  last_time = time;
}

void perception_handler(void) {
  static double last_time = 0.0;
  static int firsttime = 1;
  double time;

  if (firsttime) {
    time = dgc_get_time();
    firsttime = 0;
  }
  else {
    time = dgc_get_time();
    perception_hz = dgc_avg_times(time - last_time, 1, 5);
  }
  last_time = time;
}

void lms1_handler(void) {
  static double last_time = 0.0;
  static int firsttime = 1;
  double time;
  if (firsttime) {
    time = dgc_get_time();
    firsttime = 0;
  }
  else {
    time = dgc_get_time();
    lms1_hz = dgc_avg_times(time - last_time, 1, 7);
  }
  last_time = time;
}

void lms3_handler(void) {
  static double last_time = 0.0;
  static int firsttime = 1;
  double time;
  if (firsttime) {
    time = dgc_get_time();
    firsttime = 0;
  }
  else {
    time = dgc_get_time();
    lms3_hz = dgc_avg_times(time - last_time, 1, 8);
  }
  last_time = time;
}

void can_handler(void) {
  static double last_time = 0.0;
  static int firsttime = 1;
  double time;

  time = dgc_get_time();
  if (firsttime) {
    firsttime = 0;
  }
  else {
    can_hz = dgc_avg_times(time - last_time, 1, 9);
  }
  last_time = time;
}

void map_reset(void) {
  int r, c;
  char *cell;
  for (r = 0; r < ipc_grid->rows; r++) {
    for (c = 0; c < ipc_grid->cols; c++) {
      cell = (char *) (ipc_grid->cell) + ipc_grid->bytes_per_cell * (r * ipc_grid->cols + c);
      *cell = PERCEPTION_MAP_OBSTACLE_FREE;
    }
  }

}

void perception_map_diff_handler(vlr::PerceptionMapDiff *msg) {
  int i;
  char *cell = NULL;
  if (use_ipc_grid) {
    pthread_mutex_lock(&map_mutex);
    if (msg->new_map) {
      map_reset();
    }
    dgc_grid_recenter_grid(ipc_grid, msg->center_x, msg->center_y);
    for (i = 0; i < msg->num_points; i++) {
      cell = (char *) dgc_grid_get_xy(ipc_grid, msg->grid[i].x, msg->grid[i].y);
      if (cell != NULL) {
        *cell = msg->grid[i].type;
      }
    }
    pthread_mutex_unlock(&map_mutex);
  }
}

void perception_map_handler(vlr::PerceptionMap *msg) {
  if (msg->rows != ipc_grid->rows || msg->cols != ipc_grid->cols || (short) (msg->resolution * 100)
      != (short) (ipc_grid->resolution * 100)) {
    fprintf(stderr, "# ERROR: received grid map does not match internal size.\n"
      "# ERROR:   received = %dx%d (%.2fm)\n"
      "# ERROR:   internal = %dx%d (%.2fm)\n", msg->rows, msg->cols, msg->resolution, ipc_grid->rows, ipc_grid->cols,
        ipc_grid->resolution);
    return;
  }

  fprintf(stderr, "# INFO: received grid map %dx%d (%.2fm)\n", msg->rows, msg->cols, msg->resolution);
  ipc_grid->map_r0 = msg->map_r0;
  ipc_grid->map_c0 = msg->map_c0;
  ipc_grid->array_r0 = msg->array_r0;
  ipc_grid->array_c0 = msg->array_c0;

  fprintf(stderr, "# INFO: update grid ... ");

  //    GetExceptionInfo(&exception);
  //    info = CloneImageInfo((ImageInfo *) NULL);
  //    im = BlobToImage(info, msg->data, msg->len, &exception);
  //    if (exception.severity != UndefinedException) CatchException(&exception);
  //    if (im == (Image *) NULL) {
  //      fprintf(stderr, "failed\n");
  //      fprintf(stderr, "# ERROR: can't extract image!\n");
  //      return;
  //    }
  //    if (msg->rows != (int) im->rows || msg->cols != (int) im->columns) {
  //      fprintf(stderr, "failed\n");
  //      fprintf(stderr, "# ERROR: image size (%dx%d) does not fit grid size (%dx%d)\n", (int) im->rows,
  //          (int) im->columns, msg->rows, msg->cols);
  //    }
  //    else {
  //      fprintf(stderr, "ok\n");
  //      ExportImagePixels(im, 0, 0, ipc_grid->cols, ipc_grid->rows, "I", CharPixel, ipc_grid->cell, &exception);
  //    }
  //    DestroyImage( im);
  //    DestroyImageInfo( info);
  //  }

  if (ilLoadL(IL_TYPE_UNKNOWN, (void*) msg->data, (ILuint) msg->len) == IL_FALSE) {
    return;
  }
  ilConvertImage(IL_LUMINANCE, IL_UNSIGNED_BYTE);
  memcpy(ipc_grid->cell, ilGetData(), ipc_grid->cols * ipc_grid->rows * sizeof(uint8_t));
}

void rndf_change_handler(void) {
  pthread_mutex_lock(&rndf_mutex);
  fprintf(stderr, "# INFO: delete old display lists\n");
  //  delete_rndf_display_list( rndf_dl );
  fprintf(stderr, "# INFO: delete old rndf\n");
  delete rn;
  rn = new RoadNetwork;
  fprintf(stderr, "# INFO: load new rndf: %s\n", rndf_filename);
  if (rn->loadRNDF(rndf_filename) == 0) {
    //    rndf->mark_yield_exits(); // TODO: check if necessary
    //    rndf->mark_merge_exits();
  }
  rndf_generate_display_lists = TRUE;
  pthread_mutex_unlock(&rndf_mutex);
}

void gls_handler(vlr::GlsOverlay *gls) {
  int i, n, found = 0;
  vlr::GlsOverlay *temp;
  static double last_time = 0.0;
  static int firsttime = 1;
  static int gls_data_size[MAX_GLS_SOURCES];
  static int gls_data_ctr[MAX_GLS_SOURCES];
  static int gls_ctr = 0;
  double time, delta_s;

  if (firsttime) {
    time = dgc_get_time();
    firsttime = 0;
    for (i = 0; i < MAX_GLS_SOURCES; i++) {
      gls_data_size[i] = 0;
      gls_data_ctr[i] = 0;
    }
  }
  else {
    time = dgc_get_time();
    delta_s = time - last_time;
    gls_ctr++;
    if (delta_s > 1.0) {
      gls_hz = gls_ctr / delta_s;
      gls_ctr = 0;
      for (i = 0; i < MAX_GLS_SOURCES; i++) {
        gls_size[i] = gls_data_size[i] / (1024.0 * delta_s);
        gls_s_hz[i] = gls_data_ctr[i] / delta_s;
        gls_data_size[i] = 0;
        gls_data_ctr[i] = 0;
      }
      last_time = time;
    }
  }
  pthread_mutex_lock(&gls_mutex);
  for (i = 0; i < (signed) gls_cache.size(); i++) {
    // is the name already in the cache?
    if (!strcmp(gls->name, gls_cache[i]->name)) {
      free(gls_cache[i]->byte);
      memcpy(gls_cache[i], gls, sizeof(vlr::GlsOverlay));
      gls_cache[i]->byte = (unsigned char *) calloc(gls->max_bytes, 1);
      dgc_test_alloc(gls_cache[i]->byte);
      memcpy(gls_cache[i]->byte, gls->byte, gls->num_bytes);
      gls_cache[i]->timestamp = dgc_get_time();
      gls_data_size[i] += gls->num_bytes;
      gls_data_ctr[i]++;
      found = 1;
      break;
    }
  }
  if (!found) {
    temp = vlr::gls_alloc("");
    memcpy(temp, gls, sizeof(vlr::GlsOverlay));
    temp->byte = (unsigned char *) calloc(gls->max_bytes, 1);
    dgc_test_alloc(temp->byte);
    temp->timestamp = dgc_get_time();
    memcpy(temp->byte, gls->byte, gls->num_bytes);
    n = (signed) gls_cache.size() - 1;
    if (n < MAX_GLS_SOURCES) {
      gls_data_size[n] += gls->num_bytes;
      gls_data_ctr[n]++;
    }
    gls_cache.push_back(temp);
  }
  pthread_mutex_unlock(&gls_mutex);

  display_redraw();
}

void trafficlight_handler(vlr::TrafficLightList* tls) {

  pthread_mutex_lock(&trafficlight_mutex);
  for (int32_t i = 0; i < tls->num_light_states; i++) {
    vlr::TrafficLightState& state = tls->light_state[i];
    std::string id = state.name;
    switch (state.state) {
      case 'r':
        trafficlight_state[id] = LIGHT_STATE_RED;
        break;
      case 'y':
        trafficlight_state[id] = LIGHT_STATE_YELLOW;
        break;
      case 'g':
        trafficlight_state[id] = LIGHT_STATE_GREEN;
        break;
      default:
        trafficlight_state[id] = LIGHT_STATE_UNKNOWN;
    }
    //  char tl_state;
    //  if(trafficlight_state[id]==LIGHT_STATE_GREEN) {tl_state='g';}
    //  else if(trafficlight_state[id]==LIGHT_STATE_YELLOW) {tl_state='y';}
    //  if(trafficlight_state[id]==LIGHT_STATE_RED) {tl_state='r';}
    //  else {tl_state='u';}
    //  printf("Updated traffic light %s (%c)\n", id.c_str(), tl_state);
  }
  pthread_mutex_unlock(&trafficlight_mutex);
}

/************************************************************************
 *
 *   IPC SUBSCRIBE
 *
 ************************************************************************/

void ipc_subscribe_simulator(int on) {
  static int subscribed = 0;
  static int callback1_id = -1;
  static int callback2_id = -1;

  if (on) {
    if (!subscribed) {
      callback1_id = ipc->Subscribe(SimulatorGroundTruthID, &simulator_groundtruth, &simulator_handler,
          DGC_SUBSCRIBE_LATEST, &simulator_mutex);
      callback2_id = ipc->Subscribe(SimulatorTagID, &simulator_tag_handler, DGC_SUBSCRIBE_LATEST, &simulator_mutex);

      /*      dgc_simulator_subscribe_groundtruth_message
       ( &simulator_groundtruth, (dgc_handler_t)simulator_handler,
       DGC_SUBSCRIBE_LATEST, &simulator_mutex );
       dgc_simulator_subscribe_tag_message
       ( &simulator_tags, (dgc_handler_t)simulator_tag_handler,
       DGC_SUBSCRIBE_LATEST, &simulator_mutex );*/
      subscribed = 1;
    }
  }
  else {
    if (subscribed) {
      ipc->Unsubscribe(callback1_id);
      ipc->Unsubscribe(callback2_id);
      subscribed = 0;
    }
  }
}

void ipc_subscribe_ghost_car(int on) {
  static int subscribed = 0;
  static int callback_id = -1;

  if (on) {
    if (!subscribed) {
      callback_id = ipc->Subscribe(GhostcarPoseID, &ghost_car, ghost_car_handler, DGC_SUBSCRIBE_LATEST,
          &ghost_car_mutex);
      subscribed = 1;
    }
  }
  else {
    if (subscribed) {
      ipc->Unsubscribe(callback_id);
      subscribed = 0;
    }
  }
}

void ipc_subscribe_radar(int on) {
  static int subscribed = 0;
  static int callback_id[NUM_RADARS];

  int i;

  if (on) {
    if (!subscribed) {
      callback_id[0] = ipc->Subscribe(RadarSensor1ID, &radar_lrr2[0], DGC_SUBSCRIBE_LATEST, &radar_mutex);
      callback_id[1] = ipc->Subscribe(RadarSensor2ID, &radar_lrr2[1], DGC_SUBSCRIBE_LATEST, &radar_mutex);
      callback_id[2] = ipc->Subscribe(RadarLRR3Sensor3ID, &radar_lrr3[0], DGC_SUBSCRIBE_LATEST, &radar_mutex);
      callback_id[3] = ipc->Subscribe(RadarSensor4ID, &radar_lrr2[2], DGC_SUBSCRIBE_LATEST, &radar_mutex);
      callback_id[4] = ipc->Subscribe(RadarSensor5ID, &radar_lrr2[3], DGC_SUBSCRIBE_LATEST, &radar_mutex);
      callback_id[5] = ipc->Subscribe(RadarLRR3Sensor6ID, &radar_lrr3[1], DGC_SUBSCRIBE_LATEST, &radar_mutex);
      subscribed = 1;
    }
  }
  else {
    if (subscribed) {
      for (i = 0; i < NUM_RADARS; i++)
        ipc->Unsubscribe(callback_id[i]);
      subscribed = 0;
    }
  }
}

void ipc_subscribe_can(int on) {
  static int subscribed = 0;
  static int callback_id = -1;

  if (on) {
    if (!subscribed) {
      callback_id = ipc->Subscribe(CanStatusID, &can, &can_handler);
      subscribed = 1;
    }
  }
  else {
    if (subscribed) {
      ipc->Unsubscribe(callback_id);
      subscribed = 0;
      can_hz = -10.0;
    }
  }
}

void ipc_subscribe_error(int on) {
  static int subscribed = 0;
  static int error_callback_id = -1;
  static int status_callback_id = -1;

  int i;
  if (on) {
    if (!subscribed) {
      for (i = 0; i < MESSAGE_BUFFER_SIZE; i++) {
        strcpy(message_buffer[i].string, "   ");
        message_buffer[i].timestamp = 0;
        strcpy(error_buffer[i].string, "   ");
        error_buffer[i].timestamp = 0;
      }
      subscribed = 1;
      status_callback_id = ipc->Subscribe(ErrorStatusID, error_status_handler, DGC_SUBSCRIBE_ALL);
      error_callback_id = ipc->Subscribe(ErrorStringID, error_error_handler, DGC_SUBSCRIBE_ALL);
    }
  }
  else {
    if (subscribed) {
      for (i = 0; i < MESSAGE_BUFFER_SIZE; i++) {
        free(message_buffer[i].string);
        free(error_buffer[i].string);
      }
      ipc->Unsubscribe(error_callback_id);
      ipc->Unsubscribe(status_callback_id);
      subscribed = 0;
    }
  }
}

void ipc_subscribe_estop(int on) {
  static int subscribed = 0;
  static int callback_id = -1;

  if (on) {
    if (!subscribed) {
      callback_id = ipc->Subscribe(EstopStatusID, estop_status_handler);
      estop.estop_code = 0;
      subscribed = 1;
    }
  }
  else {
    if (subscribed) {
      ipc->Unsubscribe(callback_id);
      subscribed = 0;
    }
  }
}

void ipc_subscribe_localize(int on) {
  static int subscribed = 0;
  static int callback_id = -1;

  if (on) {
    if (!subscribed) {
      callback_id = ipc->Subscribe(LocalizePoseID, &loc_pose);
      subscribed = 1;
    }
  }
  else {
    if (subscribed) {
      ipc->Unsubscribe(callback_id);
      subscribed = 0;
    }
  }
}

void ipc_subscribe_ldlrs1(int on) {
  static int subscribed = 0;
  static int callback_id = -1;

  if (on) {
    if (!subscribed) {
      callback_id = ipc->Subscribe(LdlrsLaser1ID, &ldlrs1, &ldlrs1_handler, DGC_SUBSCRIBE_LATEST, &ldlrs1_mutex);
      subscribed = 1;
    }
  }
  else {
    if (subscribed) {
      ipc->Unsubscribe(callback_id);
      subscribed = 0;
      ldlrs1_hz = -10.0;
    }
  }
}

void ipc_subscribe_ldlrs2(int on) {
  static int subscribed = 0;
  static int callback_id = -1;

  if (on) {
    if (!subscribed) {
      callback_id = ipc->Subscribe(LdlrsLaser2ID, &ldlrs2, &ldlrs2_handler, DGC_SUBSCRIBE_LATEST, &ldlrs2_mutex);
      subscribed = 1;
    }
  }
  else {
    if (subscribed) {
      ipc->Unsubscribe(callback_id);
      subscribed = 0;
      ldlrs2_hz = -10.0;
    }
  }
}

void ipc_subscribe_lms1(int on) {
  static int subscribed = 0;
  static int callback_id = -1;

  if (on) {
    if (!subscribed) {
      callback_id = ipc->Subscribe(LaserLaser1ID, &lms1, &lms1_handler, DGC_SUBSCRIBE_LATEST, &lms1_mutex);
      subscribed = 1;
    }
  }
  else {
    if (subscribed) {
      ipc->Unsubscribe(callback_id);
      subscribed = 0;
      lms1_hz = -10.0;
    }
  }
}

void ipc_subscribe_lms3(int on) {
  static int subscribed = 0;
  static int callback_id = -1;

  if (on) {
    if (!subscribed) {
      callback_id = ipc->Subscribe(LaserLaser3ID, &lms3, &lms3_handler, DGC_SUBSCRIBE_LATEST, &lms3_mutex);
      subscribed = 1;
    }
  }
  else {
    if (subscribed) {
      ipc->Unsubscribe(callback_id);
      subscribed = 0;
      lms3_hz = -10.0;
    }
  }
}

void ipc_subscribe_obstacles(int on) {
  static int subscribed = 0;
  static int callback_id = -1;

  if (on) {
    if (!subscribed) {
      callback_id = ipc->Subscribe(vlr::PerceptionObstaclesID, &obstacles, &perception_handler, DGC_SUBSCRIBE_LATEST,
          &obstacles_mutex);
      subscribed = 1;
    }
  }
  else {
    if (subscribed) {
      ipc->Unsubscribe(callback_id);
      subscribed = 0;
      perception_hz = -10.0;
    }
  }
}

void ipc_subscribe_map_diff(int on) {
  static int subscribed = 0;
  static int callback_id = -1;

  if (on) {
    if (!subscribed) {
      callback_id = ipc->Subscribe(vlr::PerceptionMapDiffID, &perception_map_diff_handler, DGC_SUBSCRIBE_LATEST, &map_mutex);
      fprintf(stderr, "subscribe map diff\n");
      vlr::PerceptionMapRequestCommand(ipc);
      subscribed = 1;
    }
  }
  else {
    if (subscribed) {
      ipc->Unsubscribe(callback_id);
      subscribed = 0;
    }
  }
}

void ipc_subscribe_map(int on) {
  static int subscribed = 0;
  static int callback_id = -1;

  if (on) {
    if (!subscribed) {
      callback_id = ipc->Subscribe(vlr::PerceptionMapID, &perception_map_handler);
      subscribed = 1;
    }
  }
  else {
    if (subscribed) {
      ipc->Unsubscribe(callback_id);
      subscribed = 0;
    }
  }
}

void ipc_subscribe_stop_zones(int on) {
  static int subscribed = 0;
  static int callback_id = -1;

  if (on) {
    if (!subscribed) {
      callback_id = ipc->Subscribe(vlr::PerceptionStopZonesID, &stop_zones, DGC_SUBSCRIBE_LATEST, &stop_zone_mutex);
      subscribed = 1;
    }
  }
  else {
    if (subscribed) {
      ipc->Unsubscribe(callback_id);
      subscribed = 0;
    }
  }
}

void ipc_subscribe_trafficlights(int on) {
  static int subscribed = 0;
  static int callback_id = -1;

  if (on) {
    if (!subscribed) {
      callback_id = ipc->Subscribe(vlr::TrafficLightListMsgID, &trafficlight_handler, DGC_SUBSCRIBE_LATEST);
      subscribed = 1;
    }
  }
  else {
    if (subscribed) {
      ipc->Unsubscribe(callback_id);
      subscribed = 0;
    }
  }
}

void ipc_subscribe_trajectory(int on) {
  static int subscribed = 0;
  static int callback_id = -1;
  static int callback_id2 = -1;

  if (on) {
    if (!subscribed) {
      callback_id = ipc->Subscribe(PlannerTrajectoryID, &planner_trajectory, DGC_SUBSCRIBE_LATEST, &traj_mutex);
      callback_id2 = ipc->Subscribe(vlr::TrajectoryPoints2DID, &trajectory_points, DGC_SUBSCRIBE_LATEST, &traj2D_mutex);
      subscribed = 1;
    }
  }
  else {
    if (subscribed) {
      ipc->Unsubscribe(callback_id);
      ipc->Unsubscribe(callback_id2);
      subscribed = 0;
    }
  }
}

void ipc_subscribe_goal(int on) {
  static int subscribed = 0;
  static int callback_id = -1;

  if (on) {
    if (!subscribed) {
      callback_id = ipc->Subscribe(PlannerMdfGoalID, &planner_mdf_goal, DGC_SUBSCRIBE_LATEST, &goal_mutex);
      subscribed = 1;
    }
  }
  else {
    if (subscribed) {
      ipc->Unsubscribe(callback_id);
      subscribed = 0;
    }
  }
}

void ipc_subscribe_planner_goal(int on) {
  static int subscribed = 0;
  static int callback_id = -1;

  if (on) {
    if (!subscribed) {
      callback_id = ipc->Subscribe(PlannerGoalID, &planner_goal_handler);
      subscribed = 1;
    }
  }
  else {
    if (subscribed) {
      ipc->Unsubscribe(callback_id);
      subscribed = 0;
    }
  }
}

void ipc_subscribe_applanix(int on) {
  static int subscribed = 0;
  static int callback_id = -1;

  if (on) {
    if (!subscribed) {
      callback_id = ipc->Subscribe(ApplanixPoseID, &applanix_handler, DGC_SUBSCRIBE_ALL);
      subscribed = 1;
    }
  }
  else {
    if (subscribed) {
      ipc->Unsubscribe(callback_id);
      subscribed = 0;
      applanix_hz = -10.0;
    }
  }
}

void ipc_subscribe_applanix_rms(int on) {
  static int subscribed = 0;
  static int callback_id = -1;

  if (on) {
    if (!subscribed) {
      callback_id = ipc->Subscribe(ApplanixRmsID, &applanix_rms);
      subscribed = 1;
    }
  }
  else {
    if (subscribed) {
      ipc->Unsubscribe(callback_id);
      subscribed = 0;
    }
  }
}

void ipc_subscribe_applanix_gps(int on) {
  static int subscribed = 0;
  static int callback_id = -1;

  if (on) {
    if (!subscribed) {
      callback_id = ipc->Subscribe(ApplanixGpsID, &applanix_gps);
      subscribed = 1;
    }
  }
  else {
    if (subscribed) {
      ipc->Unsubscribe(callback_id);
      subscribed = 0;
    }
  }
}

void ipc_subscribe_applanix_dmi(int on) {
  static int subscribed = 0;
  static int callback_id = -1;

  if (on) {
    if (!subscribed) {
      callback_id = ipc->Subscribe(ApplanixDmiID, &applanix_dmi);
      subscribed = 1;
    }
  }
  else {
    if (subscribed) {
      ipc->Unsubscribe(callback_id);
      subscribed = 0;
    }
  }
}

void ipc_subscribe_controller(int on) {
  static int subscribed = 0;
  static int callback_id = -1;

  if (on) {
    if (!subscribed) {
      callback_id = ipc->Subscribe(ControllerTargetID, &controller_target);
      subscribed = 1;
    }
  }
  else {
    if (subscribed) {
      ipc->Unsubscribe(callback_id);
      subscribed = 0;
    }
  }
}

void ipc_subscribe_passat_actuators(int on) {
  static int subscribed = 0;
  static int callback1_id = -1;
  static int callback2_id = -1;

  if (on) {
    if (!subscribed) {
      callback1_id = ipc->Subscribe(PassatActuatorID, &passat_act);
      callback2_id = ipc->Subscribe(PassatTurnSignalID, &passat_signal);
      subscribed = 1;
    }
  }
  else {
    if (subscribed) {
      ipc->Unsubscribe(callback1_id);
      ipc->Unsubscribe(callback2_id);
      subscribed = 0;
    }
  }
}

void ipc_subscribe_gls(int on) {
  static int subscribed = 0;
  static int callback_id = -1;

  if (on) {
    if (!subscribed) {
      callback_id = ipc->Subscribe(vlr::GlsOverlayID, &gls_handler, DGC_SUBSCRIBE_ALL, &gls_mutex);
      subscribed = 1;
    }
  }
  else {
    if (subscribed) {
      ipc->Unsubscribe(callback_id);
      subscribed = 0;
    }
  }
}

/************************************************************************
 *
 *   IPC PARAMETER SETTINGS
 *
 ************************************************************************/

void param_handler(void) {
  vlr::dgc_imagery_set_offset(DGC_IMAGERY_TYPE_DARPA, darpa_x_offset, darpa_y_offset);
  vlr::dgc_imagery_set_offset(DGC_IMAGERY_TYPE_GSAT, google_x_offset, google_y_offset);
}

void read_parameters(ParamInterface *pint, int argc, char **argv) {
  int i, n;
  char str[NAME_LENGTH];
  int mark_lrr2 = 0, mark_lrr3 = 0;

  Param params[] = {
  {
  "velodyne", "shm_key", DGC_PARAM_INT, &velo_shm_key, 0, NULL
  }, {
  "transform", "ldlrs_laser1", DGC_PARAM_TRANSFORM, &ldlrs1_offset, 1, NULL
  }, {
  "transform", "ldlrs_laser2", DGC_PARAM_TRANSFORM, &ldlrs2_offset, 1, NULL
  }, {
  "transform", "sick_laser1", DGC_PARAM_TRANSFORM, &lms1_offset, 1, NULL
  }, {
  "transform", "sick_laser3", DGC_PARAM_TRANSFORM, &lms3_offset, 1, NULL
  }, {
  "imagery", "root", DGC_PARAM_FILENAME, &imagery_root, 1, NULL
  }, {
  "rndf", "rndf_file", DGC_PARAM_FILENAME, &rndf_filename, 1, ParamCB(rndf_change_handler)
  },
  //    {"rndf", "rndf_file", DGC_PARAM_FILENAME, &rndf_filename, 0, NULL },
      {
      "perception", "map_resolution", DGC_PARAM_DOUBLE, &map_resolution, 0, NULL
      },
      {
      "perception", "map_size_x", DGC_PARAM_INT, &map_size_x, 0, NULL
      },
      {
      "perception", "map_size_y", DGC_PARAM_INT, &map_size_y, 0, NULL
      },
      {
      "perception", "map_size_y", DGC_PARAM_INT, &map_size_y, 0, NULL
      },
      {
      "perception", "stop_zone_dist_before_line", DGC_PARAM_DOUBLE, &stop_zones_dist_before_line, 1, NULL
      },
      {
      "perception", "stop_zone_dist_behind_line", DGC_PARAM_DOUBLE, &stop_zones_dist_behind_line, 1, NULL
      },
      {
      "perception", "stop_zone_width", DGC_PARAM_DOUBLE, &stop_zones_width, 1, NULL
      },
      {
      "perception", "stop_zone_side_shift", DGC_PARAM_DOUBLE, &stop_zones_side_shift, 1, NULL
      },
      {
      "imagery", "darpa_x_offset", DGC_PARAM_DOUBLE, &darpa_x_offset, 1, ParamCB(param_handler)
      },
      {
      "imagery", "darpa_y_offset", DGC_PARAM_DOUBLE, &darpa_y_offset, 1, ParamCB(param_handler)
      },
      {
      "imagery", "google_x_offset", DGC_PARAM_DOUBLE, &google_x_offset, 1, ParamCB(param_handler)
      },
      {
      "imagery", "google_y_offset", DGC_PARAM_DOUBLE, &google_y_offset, 1, ParamCB(param_handler)
      },
  };
  pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));

  vlr::dgc_imagery_set_offset(DGC_IMAGERY_TYPE_DARPA, darpa_x_offset, darpa_y_offset);
  vlr::dgc_imagery_set_offset(DGC_IMAGERY_TYPE_GSAT, google_x_offset, google_y_offset);

  for (i = 0; i < NUM_RADARS; i++) {
    snprintf(str, NAME_LENGTH, "radar%d", i + 1);
    if (i != RADAR3_INDEX && i != RADAR6_INDEX) {
      n = pint->GetTransform("transform", str, &(radar_lrr2_offset[mark_lrr2]));
      //fprintf(stderr, "i = %i, stored in radar_lrr2_offset[%i]\n", i, mark_lrr2);
      mark_lrr2++;
    }
    else {
      n = pint->GetTransform("transform", str, &(radar_lrr3_offset[mark_lrr3]));
      //fprintf(stderr, "i = %i, stored in radar_lrr3_offset[%i]\n", i, mark_lrr3);
      mark_lrr3++;
    }

    if (n < 0) {
      fprintf(stderr, "[31;1mParameter server contains no definition "
        "for %s_%s[0m\n\n", "transform", str);
      ::exit(0);
    }
  }
}

/************************************************************************
 *
 *   IPC INIT
 *
 ************************************************************************/

void viewer_ipc_init(int argc, char *argv[]) {
  int i;

  ipc = new IpcStandardInterface();
  pint = new ParamInterface(ipc);
  if (ipc->Connect(argv[0]) < 0) dgc_fatal_error("Could not connect to IPC network.");

  read_parameters(pint, argc, argv);

  planner_trajectory.num_waypoints = 0;

  applanix_history_init(APPLANIX_HISTORY_LENGTH);

  for (i = 0; i < MAX_GLS_SOURCES; i++) {
    gls_size[i] = 0.0;
    gls_enable[i] = 1;
    gls_s_hz[i] = -10.0;
  }
  for (i = 0; i < MAX_GLS_SOURCES + 2; i++) {
    gls_select[i] = (char *) malloc(sizeof(char) * MAX_GLS_NAME_LENGTH);
  }

  for (i = 0; i < MAX_SENSOR_INFO; i++) {
    sensor_info[i] = (char *) malloc(sizeof(char) * NAME_LENGTH);
  }

  ipc_subscribe_applanix(1);
  ApplanixPose pose = {
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0,
      0,
      0.0,
      0,
      0.0,
      ""
  };
  applanix_history_add(&pose);

  ipc_subscribe_applanix_rms(1);
  ipc_subscribe_applanix_gps(1);
  ipc_subscribe_applanix_dmi(1);
  ipc_subscribe_can(1);
  ipc_subscribe_estop(1);
  ipc_subscribe_error(1);
  ipc_subscribe_localize(1);
  ipc_subscribe_map(1);
  ipc_subscribe_map_diff(1);
  ipc_subscribe_stop_zones(1);
  ipc_subscribe_trafficlights(1);
  ipc_subscribe_obstacles(1);
  ipc_subscribe_passat_actuators(1);
  ipc_subscribe_controller(1);
  ipc_subscribe_simulator(1);
  ipc_subscribe_ghost_car(1);
  ipc_subscribe_trajectory(1);
  ipc_subscribe_goal(1);
  ipc_subscribe_planner_goal(1);
  ipc_check_subscription();
}

void ipc_check_subscription(void) {
  ipc_subscribe_ldlrs1(show_ldlrs1);
  ipc_subscribe_ldlrs2(show_ldlrs2);
  ipc_subscribe_lms1(show_lms1);
  ipc_subscribe_lms3(show_lms3);
  ipc_subscribe_radar(show_radar);
  ipc_subscribe_gls(show_gls);
  //  ipc_subscribe_map_diff( show_ipc_grid_map );
}
