#include <roadrunner.h>
#include <lltransform.h>
#include <ipc_std_interface.h>
#include <multicentral.h>
#include <passat_constants.h>
#include <applanix_interface.h>
#include <passat_interface.h>
#include <param_interface.h>
#include <can_interface.h>
#include <localize_interface.h>
#include <simulator_interface.h>
#include <perception_interface.h>
#include <radar_interface.h>
#include <estop_interface.h>
#include <ldlrs_interface.h>
#include <error_interface.h>
#include <planner_messages.h>
#include <aw_roadNetwork.h>
#include <iostream>
#include "vehicle.h"
#include "lasersim.h"
#include "stopzones.h"
#include "trafficlights.h"
#include "crosswalks.h"

using namespace vlr::rndf;
using namespace dgc;

namespace vlr {

typedef struct {
  double x, y, z;
  double roll, pitch, yaw;
} sensor_offset_t, *sensor_offset_p;

#define MAX_NUM_VEHICLES   40

int num_vehicles = 0;
vehicle_state *vehicle = NULL;
int bicycle_model, torque_mode;
char *obstacle_map_filename;

double laser_max_range, track_max_range, radar_max_range;

sensor_offset_t ldlrs1_offset;
sensor_offset_t radar1_offset, radar2_offset, radar3_offset, radar4_offset,
radar5_offset;

double ldlrs1_start_angle = -180.0;

char *rndf_filename = NULL;
int rndf_valid = 0;
//rndf_file *rndf;
RoadNetwork rn;

stop_zone_finder *sz_finder = NULL;
TrafficLightSimulator *lights = NULL;
CrosswalkSimulator *crosswalks = NULL;

#define MIN(x,y) (x < y ? x : y)
#define MAX(x,y) (x > y ? x : y)

typedef struct {
  int id1, id2;
} collision_t;

double *start_lat, *start_lon, *start_theta;

collision_t *collision;
int num_collisions = 0, max_collisions = 0;

int *no_plan, *high_lateral_accel, *high_forward_accel;
double *last_plan;

double *first_forward_accel_ts;
int *bad_forward_accel;

double *first_lateral_accel_ts;
int *bad_lateral_accel;

char **vehicle_name;

dgc::IpcInterface *ipc;
dgc::MultiCentral *mc;

//class my_data : public rndf_data {
//public:
//  int occupied;
//  my_data *clone(void);
//};
//
//#define MYDATA(x) (static_cast<my_data *>(x))
//
//my_data *my_data::clone(void)
//{
//  my_data *d = new my_data;
//  d->occupied = this->occupied;
//  return d;
//}

inline bool point_inside_poly(double *px, double *py, int N, double x, double y)
{
  double p1x, p1y, p2x, p2y;
  int counter = 0;
  int i;
  double xinters;

  p1x = px[0];
  p1y = py[0];
  for(i = 1; i <= N; i++) {
    p2x = px[i % N];
    p2y = py[i % N];
    if(y > MIN(p1y, p2y))
      if(y <= MAX(p1y, p2y))
        if(x <= MAX(p1x, p2x))
          if(p1y != p2y) {
            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x;
            if(p1x == p2x || x <= xinters)
              counter++;
          }
    p1x = p2x;
    p1y = p2y;
  }
  if(counter % 2 != 0)
    return true;
  return false;
}

int car_car_intersection(double x1, double y1, double theta1,
    double x2, double y2, double theta2, double w,
    double l)
{
  double xp1[4], yp1[4];
  double ctheta, stheta;
  double x, y, w2 = w / 2.0, l2 = l / 2.0;

  ctheta = cos(theta1);
  stheta = sin(theta1);

  xp1[0] = x1 + l2 * ctheta - w2 * stheta;
  yp1[0] = y1 + l2 * stheta + w2 * ctheta;
  xp1[1] = x1 + l2 * ctheta + w2 * stheta;
  yp1[1] = y1 + l2 * stheta - w2 * ctheta;
  xp1[2] = x1 - l2 * ctheta + w2 * stheta;
  yp1[2] = y1 - l2 * stheta - w2 * ctheta;
  xp1[3] = x1 - l2 * ctheta - w2 * stheta;
  yp1[3] = y1 - l2 * stheta + w2 * ctheta;

  ctheta = cos(theta2);
  stheta = sin(theta2);

  x = x2 + l2 * ctheta - w2 * stheta;
  y = y2 + l2 * stheta + w2 * ctheta;
  if(point_inside_poly(xp1, yp1, 4, x, y))
    return true;

  x = x2 + l2 * ctheta + w2 * stheta;
  y = y2 + l2 * stheta - w2 * ctheta;
  if(point_inside_poly(xp1, yp1, 4, x, y))
    return true;

  x = x2 - l2 * ctheta + w2 * stheta;
  y = y2 - l2 * stheta - w2 * ctheta;
  if(point_inside_poly(xp1, yp1, 4, x, y))
    return true;

  x = x2 - l2 * ctheta - w2 * stheta;
  y = y2 - l2 * stheta + w2 * ctheta;
  if(point_inside_poly(xp1, yp1, 4, x, y))
    return true;

  return false;
}

void compute_warnings(void)
{
  double current_time;
  int i, j;

  /* first collision warnings */
  num_collisions = 0;
  for(i = 0; i < num_vehicles; i++)
    for(j = i + 1; j < num_vehicles; j++)
      if(car_car_intersection(vehicle[i].x + vehicle[i].origin_x,
          vehicle[i].y + vehicle[i].origin_y,
          vehicle[i].yaw,
          vehicle[j].x + vehicle[j].origin_x,
          vehicle[j].y + vehicle[j].origin_y,
          vehicle[j].yaw,
          DGC_PASSAT_WIDTH, DGC_PASSAT_LENGTH)) {
        if(num_collisions == max_collisions) {
          max_collisions += 100;
          collision = (collision_t *)realloc(collision, max_collisions *
              sizeof(collision_t));
          dgc_test_alloc(collision);
        }
	/*
        fprintf(stderr, "%d - %d : %.2f %.2f %.2f    %.2f %.2f %.2f\n",
            i, j, vehicle[i].x, vehicle[i].y, vehicle[i].yaw,
            vehicle[j].x, vehicle[j].y, vehicle[j].yaw);
	*/
        collision[num_collisions].id1 = i;
        collision[num_collisions].id2 = j;
        num_collisions++;
      }

  /* forward accel */
  current_time = dgc_get_time();
  for(i = 0; i < num_vehicles; i++) {
    if(fabs(vehicle[i].actual_forward_accel) > 3.0) {
      if(!bad_forward_accel[i]) {
        if(i == 0)
	  /*
          fprintf(stderr, "actual forward accel = %f\n",
              vehicle[i].actual_forward_accel);
	  */

        first_forward_accel_ts[i] = current_time;
        bad_forward_accel[i] = 1;
      }
    }
    else
      bad_forward_accel[i] = 0;
  }

  /* lateral_accel */
  for(i = 0; i < num_vehicles; i++) {
    if(fabs(vehicle[i].lateral_accel) > 2.0) {
      if(!bad_lateral_accel[i]) {
        first_lateral_accel_ts[i] = current_time;
        bad_lateral_accel[i] = 1;
      }
    }
    else
      bad_lateral_accel[i] = 0;
  }

  for(i = 0; i < num_vehicles; i++)
    if(vehicle[i].paused)
      last_plan[i] = current_time;

}

void publish_collision_info(void)
{
  int i;

  for(i = 0; i < num_collisions; i++)
    SendErrorStatus(ipc, "COLLISION!: cars %d and %d", collision[i].id1 + 1,
        collision[i].id2 + 1);
}

void get_sensor_offset(dgc_transform_t t, sensor_offset_p sensor_offset)
{
  dgc_transform_get_rotation(t, &sensor_offset->roll,
      &sensor_offset->pitch, &sensor_offset->yaw);
  dgc_transform_get_translation(t, &sensor_offset->x,
      &sensor_offset->y, &sensor_offset->z);
}

void publish_start(void)
{
  static int first = 1;
  static dgc::SimulatorStart start;

  if(first) {
    strcpy(start.host, dgc_hostname());
    first = 0;
  }
  start.timestamp = dgc_get_time();
  int err = ipc->Publish(dgc::SimulatorStartID, &start);
  dgc::TestIpcExit(err, "Could not publish", dgc::SimulatorStartID);
}

void publish_ldlrs1(vehicle_state *vehicle)
{
  static int first = 1;
  static dgc::LdlrsLaser ldlrs;
  double laser_x, laser_y;

  if(first) {
    ldlrs.range = (float *)calloc(270 * 4, sizeof(float));
    dgc_test_alloc(ldlrs.range);
    strcpy(ldlrs.host, dgc_hostname());
    ldlrs.scan_count = 0;
    first = 0;
  }

  laser_x = vehicle->x + vehicle->origin_x +
      (ldlrs1_offset.x - vehicle->param.imu_to_cg_dist) * cos(vehicle->yaw) +
      ldlrs1_offset.y * cos(vehicle->yaw + M_PI / 2.0);
  laser_y = vehicle->y + vehicle->origin_y +
      (ldlrs1_offset.x - vehicle->param.imu_to_cg_dist) * sin(vehicle->yaw) +
      ldlrs1_offset.y * sin(vehicle->yaw + M_PI / 2.0);

  generate_ldlrs_laser_scan(&ldlrs, laser_x, laser_y, vehicle->yaw +
      ldlrs1_offset.yaw, dgc_d2r(ldlrs1_start_angle),
      laser_max_range);

  ldlrs.timestamp = dgc_get_time();
  int err = ipc->Publish(dgc::LdlrsLaser1ID, &ldlrs);
  dgc::TestIpcExit(err, "Could not publish", dgc::LdlrsLaser1ID);
}

  void generate_radar_message(dgc::RadarSensor *radar,
    double sensor_x, double sensor_y,
    double sensor_yaw, double max_range,
    int vehicle_num)
{
  double obstacle_x, obstacle_y, range, bearing, dx, dy;
  int i, mark;

  mark = 0;
  for(i = 0; i < num_vehicles; i++)
    if(i != vehicle_num) {

      obstacle_x = vehicle[i].x + vehicle[i].origin_x;
      obstacle_y = vehicle[i].y + vehicle[i].origin_y;

      range = hypot(obstacle_x - sensor_x, obstacle_y - sensor_y);
      bearing =
          dgc_normalize_theta(atan2(obstacle_y - sensor_y,
              obstacle_x - sensor_x) - sensor_yaw);

      if(range > max_range || fabs(bearing) > dgc_d2r(7.0))
        continue;

      dx = (obstacle_x - sensor_x) / range;
      dy = (obstacle_y - sensor_y) / range;

      radar->target[mark].id = i;
      radar->target[mark].measured = 1;
      radar->target[mark].historical = 0;
      radar->target[mark].distance = range * cos(bearing);
      radar->target[mark].lateral_offset = range * sin(bearing);
      radar->target[mark].lateral_offset_var = 1;
      radar->target[mark].relative_acceleration = 0;
      radar->target[mark].relative_velocity =
          (vehicle[i].v_x * cos(vehicle[i].yaw) -
              vehicle[i].v_y * sin(vehicle[i].yaw)) * dx +
              (vehicle[i].v_x * sin(vehicle[i].yaw) +
                  vehicle[i].v_y * cos(vehicle[i].yaw))* dy;
      mark++;
    }
  radar->num_targets = mark;
}

void publish_radar(int which_vehicle, int radar_num)
{
  static dgc::RadarSensor radar;
  static int first = 1;
  double radar_x, radar_y;
  sensor_offset_p offset;
  int err;

  if(radar_num == 1)
    offset = &radar1_offset;
  else if(radar_num == 2)
    offset = &radar2_offset;
  else if(radar_num == 3)
    offset = &radar3_offset;
  else if(radar_num == 4)
    offset = &radar4_offset;
  else
    offset = &radar5_offset;

  if(first) {
    radar.target = (dgc::RadarTarget *)calloc(64, sizeof(dgc::RadarTarget));
    dgc_test_alloc(radar.target);
    strcpy(radar.host, dgc_hostname());
    first = 0;
  }

  radar_x = vehicle[which_vehicle].x + vehicle[which_vehicle].origin_x +
      (offset->x - vehicle[which_vehicle].param.imu_to_cg_dist) *
      cos(vehicle[which_vehicle].yaw) +
      offset->y * cos(vehicle[which_vehicle].yaw + M_PI / 2.0);
  radar_y = vehicle[which_vehicle].y + vehicle[which_vehicle].origin_y +
      (offset->x - vehicle[which_vehicle].param.imu_to_cg_dist) *
      sin(vehicle[which_vehicle].yaw) +
      offset->y * sin(vehicle[which_vehicle].yaw + M_PI / 2.0);

  generate_radar_message(&radar, radar_x, radar_y, vehicle[which_vehicle].yaw +
      offset->yaw, radar_max_range, which_vehicle);
  radar.timestamp = dgc_get_time();

  if(radar_num == 1) {
    err = ipc->Publish(dgc::RadarSensor1ID, &radar);
    TestIpcExit(err, "Could not publish", dgc::RadarSensor1ID);
  }
  else if(radar_num == 2) {
    err = ipc->Publish(dgc::RadarSensor2ID, &radar);
    TestIpcExit(err, "Could not publish", dgc::RadarSensor2ID);
  }
  else if(radar_num == 3) {
    err = ipc->Publish(dgc::RadarSensor3ID, &radar);
    TestIpcExit(err, "Could not publish", dgc::RadarSensor3ID);
  }
  else if(radar_num == 4) {
    err = ipc->Publish(dgc::RadarSensor4ID, &radar);
    TestIpcExit(err, "Could not publish", dgc::RadarSensor4ID);
  }
  else if(radar_num == 5) {
    err = ipc->Publish(dgc::RadarSensor5ID, &radar);
    TestIpcExit(err, "Could not publish", dgc::RadarSensor5ID);
  }
}

void planner_state_handler(dgc::PlannerFsmState *state)
{
  int i, found = 0;
  void *c;

  c = ipc->GetContext();
  for (i = 0; i < mc->NumCentrals(); i++)
    if (mc->Connected(i) && c == mc->Context(i)) {
      found = 1;
      break;
    }
  if(found && state->state == 11)
    vehicle[i].set_position(start_lat[i], start_lon[i], start_theta[i]);
}

void passat_actuator_handler(dgc::PassatActuator *actuator)
{
  int i, found = 0;
  void *c;

  c = ipc->GetContext();
  for(i = 0; i < mc->NumCentrals(); i++)
    if (mc->Connected(i) && c == mc->Context(i)) {
      found = 1;
      break;
    }
  if (found) {
    if (vehicle[i].param.torque_mode) {
      vehicle[i].set_torque_controls(actuator->steering_torque,
				     actuator->throttle_fraction,
				     100*actuator->brake_pressure);
    } else {
      vehicle[i].set_controls(actuator->steering_angle, 
			      actuator->throttle_fraction,
			      100*actuator->brake_pressure);
    }

    vehicle[i].set_direction((actuator->direction == 
        dgc::DGC_PASSAT_DIRECTION_FORWARD) ? 1 : 0);
    last_plan[i] = dgc_get_time();
  }
}

void estop_status_handler(dgc::EstopStatus *estop)
{
  int i, found = 0;
  void *c;

  c = ipc->GetContext();
  for(i = 0; i < mc->NumCentrals(); i++) {
    if (mc->Connected(i) && c == mc->Context(i)) {
      found = 1;
      break;
    }
  }
  if (found) {
    vehicle[i].paused = (estop->estop_code != DGC_ESTOP_RUN);
  }
}

inline int point_inside_rectangle(double point_x, double point_y,
    double rect_x, double rect_y,
    double rect_theta, double rect_w,
    double rect_l)

{
  double perp_dist, par_dist, x_match, y_match;
  double ctheta, stheta;
  double x1, y1, x2, y2;

  /* compute center axis vector */
  ctheta = cos(rect_theta);
  stheta = sin(rect_theta);
  x1  = rect_x - (rect_w / 2.0) * stheta;
  y1  = rect_y + (rect_w / 2.0) * ctheta;
  x2  = rect_x - (-rect_w / 2.0) * stheta;
  y2  = rect_y + (-rect_w / 2.0) * ctheta;
  dgc_point_to_line_distance(point_x, point_y, x1, y1, x2, y2, &par_dist,
      &perp_dist, &x_match, &y_match);
  if(par_dist >= 0.0 && par_dist <= 1.0 && perp_dist <= rect_l / 2.0)
    return 1;
  return 0;
}

inline int car_partly_inside_rectangle(double car_x, double car_y,
    double car_theta,
    double car_w, double car_l,
    double rect_x, double rect_y,
    double rect_theta, double rect_w,
    double rect_l)
{
  double ctheta, stheta, x, y;

  ctheta = cos(car_theta);
  stheta = sin(car_theta);

  x = car_x + (car_l / 2.0) * ctheta - (car_w / 2.0) * stheta;
  y = car_y + (car_l / 2.0) * stheta + (car_w / 2.0) * ctheta;
  if(point_inside_rectangle(x, y, rect_x, rect_y, rect_theta,
      rect_w, rect_l))
    return 1;

  x = car_x + (car_l / 2.0) * ctheta - (-car_w / 2.0) * stheta;
  y = car_y + (car_l / 2.0) * stheta + (-car_w / 2.0) * ctheta;
  if(point_inside_rectangle(x, y, rect_x, rect_y, rect_theta,
      rect_w, rect_l))
    return 1;

  x = car_x + (-car_l / 2.0) * ctheta - (-car_w / 2.0) * stheta;
  y = car_y + (-car_l / 2.0) * stheta + (-car_w / 2.0) * ctheta;
  if(point_inside_rectangle(x, y, rect_x, rect_y, rect_theta,
      rect_w, rect_l))
    return 1;

  x = car_x + (-car_l / 2.0) * ctheta - (car_w / 2.0) * stheta;
  y = car_y + (-car_l / 2.0) * stheta + (car_w / 2.0) * ctheta;
  if(point_inside_rectangle(x, y, rect_x, rect_y, rect_theta,
      rect_w, rect_l))
    return 1;
  return 0;
}

void precompute_stop_zones(void)
{
//  rndf_waypoint *w;
//  int i, j, k;
//  double vehicle_x, vehicle_y;
//  zone_vector zones;
//  my_data *data;
//
//  /* clear the stop zones */
//  for(i = 0; i < rndf->num_segments(); i++)
//    for(j = 0; j < rndf->segment(i)->num_lanes(); j++)
//      for(k = 0; k < rndf->segment(i)->lane(j)->num_waypoints(); k++) {
//        w = rndf->segment(i)->lane(j)->waypoint(k);
//        MYDATA(w->data)->occupied = 0;
//      }
//
//  for(i = 0; i < num_vehicles; i++) {
//    vehicle_x = vehicle[i].x + vehicle[i].origin_x;
//    vehicle_y = vehicle[i].y + vehicle[i].origin_y;
//    sz_finder->closest_zones(&zones, vehicle_x, vehicle_y, 10.0);
//
//    for(j = 0; j < (int)zones.size(); j++) {
//      w = rndf->segment(zones[j].segment)->lane(zones[j].lane)->original_waypoint(zones[j].waypoint);
//      data = MYDATA(w->data);
//      if(!data->occupied &&
//          car_partly_inside_rectangle(vehicle[i].x, vehicle[i].y,
//              vehicle[i].yaw, DGC_PASSAT_WIDTH,
//              DGC_PASSAT_LENGTH,
//              zones[j].utm_x - vehicle[i].origin_x -
//              (zones[j].length / 2.0 - 1.0) *
//              cos(zones[j].heading),
//              zones[j].utm_y - vehicle[i].origin_y -
//              (zones[j].length / 2.0 - 1.0) *
//              sin(zones[j].heading),
//              zones[j].heading, zones[j].width,
//              zones[j].length)) {
//        if(vehicle[i].v_x>.2)
//          data->occupied = 2;
//        else
//          data->occupied = 1;
//      }
//    }
//  }
}

void publish_stop_zones(int which_vehicle)
{
//  static PerceptionStopZones msg;
//  static int first = 1;
//  double vehicle_x, vehicle_y;
//  zone_vector zones;
//  int i;
//  rndf_waypoint *w;
//
//  if(first) {
//    strcpy(msg.host, dgc_hostname());
//    first = 0;
//  }
//  msg.timestamp = dgc_get_time();
//
//  vehicle_x = vehicle[which_vehicle].x + vehicle[which_vehicle].origin_x;
//  vehicle_y = vehicle[which_vehicle].y + vehicle[which_vehicle].origin_y;
//
//  sz_finder->closest_zones(&zones, vehicle_x, vehicle_y, 100.0);
//  msg.num_zones = (signed)zones.size();
//  msg.zone = &(zones[0]);
//
//  for(i = 0; i < msg.num_zones; i++) {
//    msg.zone[i].state = ZONE_STATE_FREE;
//    w = rndf->segment(zones[i].segment)->lane(zones[i].lane)->original_waypoint(zones[i].waypoint);
//    if(MYDATA(w->data)->occupied == 1)
//      msg.zone[i].state = ZONE_STATE_OBSTACLE_STOPPED;
//    else if(MYDATA(w->data)->occupied == 2)
//      msg.zone[i].state = ZONE_STATE_OBSTACLE_MOVING;
//  }
//
//  int err = ipc->Publish(PerceptionStopZonesID, &msg);
//  TestIpcExit(err, "Could not publish", PerceptionStopZonesID);
}

void publish_traffic_lights()
{
  lights->update(dgc_get_time());
}

void fill_dirk_points(PerceptionObstaclePoint *point,
    int n, double x1, double y1,
    double x2, double y2, int id)
{
  int i;

  for(i = 0; i < n; i++) {
    point[i].x = x1 + (x2 - x1) * i / (double)n;
    point[i].y = y1 + (y2 - y1) * i / (double)n;
    point[i].z_min = -1.6;
    point[i].z_max = 0;
    point[i].type = id;
  }
}

void publish_map_diff(int which_vehicle)
{
  static PerceptionMapDiff diff;
  static char *host = NULL;

  if(host == NULL) {
    host = dgc_hostname();
    strcpy(diff.host, host);
  }

  diff.num_points = 0;
  diff.grid = NULL;
  diff.resolution = 0.15;
  diff.mapsize_x = 1000;
  diff.mapsize_y = 1000;
  diff.center_x = vehicle[which_vehicle].x;
  diff.center_y = vehicle[which_vehicle].y;
  diff.new_map = 0;

  int err = ipc->Publish(PerceptionMapDiffID, &diff);
  TestIpcExit(err, "Could not publish", PerceptionMapDiffID);
}

static PerceptionObstacles all_obstacles;

void precompute_fake_obstacles(void)
{
  static int first = 1;
  double obstacle_x, obstacle_y;
  double ctheta, stheta;
  int i, id, mark;

  if(first) {
    all_obstacles.point =
        (PerceptionObstaclePoint *)
        calloc(num_vehicles, sizeof(PerceptionObstaclePoint) * 40);
    dgc_test_alloc(all_obstacles.point);
    first = 0;
  }

  mark = 0;
  for(i = 0; i < num_vehicles; i++) {
    obstacle_x = vehicle[i].x;
    obstacle_y = vehicle[i].y;
    stheta = sin(vehicle[i].yaw);
    ctheta = cos(vehicle[i].yaw);

    if(hypot(vehicle[i].v_x, vehicle[i].v_y) > dgc_mph2ms(2.0))
      id = 1;
    else
      id = 0;

    fill_dirk_points(all_obstacles.point + mark, 10,
        obstacle_x + 0.5 * (DGC_PASSAT_LENGTH * ctheta -
            DGC_PASSAT_WIDTH * stheta),
            obstacle_y + 0.5 * (DGC_PASSAT_LENGTH * stheta +
                DGC_PASSAT_WIDTH * ctheta),
                obstacle_x + 0.5 * (DGC_PASSAT_LENGTH * ctheta +
                    DGC_PASSAT_WIDTH * stheta),
                    obstacle_y + 0.5 * (DGC_PASSAT_LENGTH * stheta -
                        DGC_PASSAT_WIDTH * ctheta), id);
    fill_dirk_points(all_obstacles.point + mark + 10, 10, 
        obstacle_x + 0.5 * (DGC_PASSAT_LENGTH * ctheta +
            DGC_PASSAT_WIDTH * stheta),
            obstacle_y + 0.5 * (DGC_PASSAT_LENGTH * stheta -
                DGC_PASSAT_WIDTH * ctheta),
                obstacle_x + 0.5 * (-DGC_PASSAT_LENGTH * ctheta +
                    DGC_PASSAT_WIDTH * stheta),
                    obstacle_y + 0.5 * (-DGC_PASSAT_LENGTH * stheta -
                        DGC_PASSAT_WIDTH * ctheta), id);
    fill_dirk_points(all_obstacles.point + mark + 20, 10, 
        obstacle_x + 0.5 * (-DGC_PASSAT_LENGTH * ctheta +
            DGC_PASSAT_WIDTH * stheta),
            obstacle_y + 0.5 * (-DGC_PASSAT_LENGTH * stheta -
                DGC_PASSAT_WIDTH * ctheta),
                obstacle_x + 0.5 * (-DGC_PASSAT_LENGTH * ctheta -
                    DGC_PASSAT_WIDTH * stheta),
                    obstacle_y + 0.5 * (-DGC_PASSAT_LENGTH * stheta +
                        DGC_PASSAT_WIDTH * ctheta), id);
    fill_dirk_points(all_obstacles.point + mark + 30, 10, 
        obstacle_x + 0.5 * (-DGC_PASSAT_LENGTH * ctheta -
            DGC_PASSAT_WIDTH * stheta),
            obstacle_y + 0.5 * (-DGC_PASSAT_LENGTH * stheta +
                DGC_PASSAT_WIDTH * ctheta),
                obstacle_x + 0.5 * (DGC_PASSAT_LENGTH * ctheta -
                    DGC_PASSAT_WIDTH * stheta),
                    obstacle_y + 0.5 * (DGC_PASSAT_LENGTH * stheta +
                        DGC_PASSAT_WIDTH * ctheta), id);
    mark += 40;
  }
}

void publish_dirk_obstacles(int which_vehicle)
{
  static PerceptionObstacles obstacles;
  static char *host = NULL;
  int i, j, mark;
  double applanix_x, applanix_y, obstacle_x, obstacle_y;
  double add_x, add_y, r;

  if(host == NULL) {
    host = dgc_hostname();
    strcpy(obstacles.host, host);
    obstacles.point =
        (PerceptionObstaclePoint *)
        malloc(sizeof(PerceptionObstaclePoint) * (num_vehicles * 40 + crosswalks->maxObstaclePoints()));
    obstacles.dynamic_obstacle =
            (PerceptionDynamicObstacle *)
            malloc(sizeof(PerceptionDynamicObstacle) * (num_vehicles + crosswalks->maxObstacles()));
    dgc_test_alloc(obstacles.point);
  }

  obstacles.timestamp = dgc_get_time();

  applanix_x = vehicle[which_vehicle].x + vehicle[which_vehicle].origin_x -
      vehicle[which_vehicle].param.imu_to_cg_dist *
      cos(vehicle[which_vehicle].yaw);
  applanix_y = vehicle[which_vehicle].y + vehicle[which_vehicle].origin_y -
      vehicle[which_vehicle].param.imu_to_cg_dist *
      sin(vehicle[which_vehicle].yaw);

  mark = 0;
  obstacles.num_points = 0;
  obstacles.num_dynamic_obstacles = 0;
  for(i = 0; i < num_vehicles; i++) {
    if(i != which_vehicle) {
      obstacle_x = vehicle[i].x + vehicle[i].origin_x;
      obstacle_y = vehicle[i].y + vehicle[i].origin_y;
      r = hypot(obstacle_x - applanix_x, obstacle_y - applanix_y);

      if(r < laser_max_range) {
        add_x =  vehicle[i].origin_x - applanix_x + vehicle[which_vehicle].x -
            vehicle[which_vehicle].param.imu_to_cg_dist *
            cos(vehicle[which_vehicle].yaw);
        add_y =  vehicle[i].origin_y - applanix_y + vehicle[which_vehicle].y -
            vehicle[which_vehicle].param.imu_to_cg_dist *
            sin(vehicle[which_vehicle].yaw);

        memcpy(obstacles.point + mark,
            all_obstacles.point + i * 40,
            40 * sizeof(PerceptionObstaclePoint));
        for(j = 0; j < 40; j++) {
          obstacles.point[mark + j].type = 0;
          obstacles.point[mark + j].x += add_x;
          obstacles.point[mark + j].y += add_y;
        }
        obstacles.num_points += 40;
        mark += 40;

        PerceptionDynamicObstacle &obstacle = obstacles.dynamic_obstacle[obstacles.num_dynamic_obstacles];
        obstacle.id = i;
        obstacle.confidence = 255;
        obstacle.x = obstacle_x;
        obstacle.y = obstacle_y;
        obstacle.direction = vehicle[i].yaw;
        obstacle.velocity = sqrt(vehicle[i].v_x*vehicle[i].v_x + vehicle[i].v_y*vehicle[i].v_y);
        obstacle.length = DGC_PASSAT_LENGTH;
        obstacle.width = DGC_PASSAT_WIDTH;
        obstacle.obstacleType = OBSTACLE_CAR;

        obstacles.num_dynamic_obstacles++;
      }
    }
  }

// TODO: completely remove static obstacles purposefully makred to be dynamic
//  if (crosswalks->getObstacles().num_points > 0) {
//    memcpy(obstacles.point + mark, crosswalks->getObstacles().point, crosswalks->getObstacles().num_points
//        * sizeof(PerceptionObstaclePoint));
//    obstacles.num_points += crosswalks->getObstacles().num_points;
//
//    for (j = 0; j < crosswalks->getObstacles().num_points; j++) {
//      obstacles.point[mark + j].x -= vehicle[which_vehicle].origin_x;
//      obstacles.point[mark + j].y -= vehicle[which_vehicle].origin_y;
//    }
//  }

  if (crosswalks->getObstacles().num_dynamic_obstacles > 0) {
    memcpy(obstacles.dynamic_obstacle + obstacles.num_dynamic_obstacles, crosswalks->getObstacles().dynamic_obstacle,
        crosswalks->getObstacles().num_dynamic_obstacles * sizeof(PerceptionDynamicObstacle));
    for (j = 0; j < crosswalks->getObstacles().num_dynamic_obstacles; j++) {
      obstacles.dynamic_obstacle[mark + j].x -= vehicle[which_vehicle].origin_x;
      obstacles.dynamic_obstacle[mark + j].y -= vehicle[which_vehicle].origin_y;
    }
    obstacles.num_dynamic_obstacles += crosswalks->getObstacles().num_dynamic_obstacles;
  }

 if(obstacles.num_dynamic_obstacles > 0 || obstacles.num_points > 0) {
   int err = ipc->Publish(PerceptionObstaclesID, &obstacles);
   TestIpcExit(err, "Could not publish", PerceptionObstaclesID);
 }
}

void publish_localize(vehicle_state *vehicle)
{
  static dgc::LocalizePose pose;
  static int first = 1;

  if(first) {
    strncpy(pose.host, dgc_hostname(), 10);
    first = 0;
  }
  pose.x_offset = vehicle->origin_x + vehicle->added_error_x;
  pose.y_offset = vehicle->origin_y + vehicle->added_error_y;
  pose.timestamp = dgc_get_time();
  int err = ipc->Publish(dgc::LocalizePoseID, &pose);
  dgc::TestIpcExit(err, "Could not publish", dgc::LocalizePoseID);
}

void publish_applanix(vehicle_state *vehicle)
{
  static dgc::ApplanixPose applanix_pose;
  static int first = 1;

  if(first) {
    strncpy(applanix_pose.host, dgc_hostname(), 10);
    first = 0;
  }

  vehicle->fill_applanix_message(&applanix_pose);

  int err = ipc->Publish(dgc::ApplanixPoseID, &applanix_pose);
  dgc::TestIpcExit(err, "Could not publish", dgc::ApplanixPoseID);
}

void publish_groundtruth(void)
{
  static int first = 1;
  static dgc::SimulatorGroundTruth truth;
  int i, j, mark = 0, found = 0;
  void *c;
  double current_time;

  if(first) {
    strcpy(truth.host, dgc_hostname());
    truth.vehicle =
        (SimulatorVehiclePose *)calloc(num_vehicles,
            sizeof(SimulatorVehiclePose));
    dgc_test_alloc(truth.vehicle);
    first = 0;
  }

  truth.num_vehicles = num_vehicles;
  for(j = 0; j < num_vehicles; j++) {
    truth.vehicle[mark].x = vehicle[j].x + vehicle[j].origin_x;
    truth.vehicle[mark].y = vehicle[j].y + vehicle[j].origin_y;
    truth.vehicle[mark].theta = vehicle[j].yaw;
    truth.vehicle[mark].alpha = vehicle[j].wheel_angle;
    truth.vehicle[mark].v = vehicle[j].v_x;
    truth.vehicle[mark].forward_accel = vehicle[j].actual_forward_accel;
    truth.vehicle[mark].lateral_accel = vehicle[j].lateral_accel;

    // fill us in later
    truth.vehicle[mark].plan_warning = 0;
    truth.vehicle[mark].collision_warning = 0;
    truth.vehicle[mark].forward_accel_warning = 0;
    truth.vehicle[mark].lateral_accel_warning = 0;
    mark++;
  }

  /* mark collisions */
  for(i = 0; i < num_collisions; i++) {
    truth.vehicle[collision[i].id1].collision_warning = 1;
    truth.vehicle[collision[i].id2].collision_warning = 1;
  }

  current_time = dgc_get_time();

  /* then plan warnings */
  for(i = 0; i < num_vehicles; i++)
    truth.vehicle[i].plan_warning = (current_time - last_plan[i] > 2.0);

  /* then acceleration warnings */
  for(i = 0; i < num_vehicles; i++) {
    truth.vehicle[i].forward_accel_warning =
        (bad_forward_accel[i] && current_time -
            first_forward_accel_ts[i] >= 0.25);
    truth.vehicle[i].lateral_accel_warning =
        (bad_lateral_accel[i] && current_time -
            first_lateral_accel_ts[i] >= 0.25);
  }


  for(i = 0; i < num_vehicles; i++) {
    if(truth.vehicle[i].forward_accel_warning)
      SendErrorStatus(ipc, "HARD BRAKE!: car %d", i + 1);
    if(truth.vehicle[i].lateral_accel_warning)
      SendErrorStatus(ipc, "TURN TOO FAST!: car %d", i + 1);
    if(truth.vehicle[i].plan_warning)
      SendErrorStatus(ipc, "NO PLAN: car %d", i + 1);
  }

  c = ipc->GetContext();
  for (i = 0; i < mc->NumCentrals(); i++)
    if (mc->Connected(i) && c == mc->Context(i)) {
      found = 1;
      break;
    }
  truth.our_vehicle_num = i;

  truth.timestamp = dgc_get_time();
  int err = ipc->Publish(dgc::SimulatorGroundTruthID, &truth);
  dgc::TestIpcExit(err, "Could not publish", dgc::SimulatorGroundTruthID);
}

void publish_tags(void)
{
  static int first = 1;
  static dgc::SimulatorTag tag;
  int i, numv = 0;

  if (num_vehicles>MAX_NUM_VEHICLES) {
    numv = MAX_NUM_VEHICLES;
  } else {
    numv = num_vehicles;
  }

  if(first) {
    strcpy(tag.host, dgc_hostname());
    tag.tag =
        (dgc_car_name *)calloc(numv, sizeof(dgc_car_name));
    dgc_test_alloc(tag.tag);
    first = 0;
  }

  tag.num_vehicles = numv;
  for(i = 0; i < numv; i++) {
    strncpy(tag.tag[i], vehicle_name[i], 20);
    tag.tag[i][19] = '\0';
  }

  tag.timestamp = dgc_get_time();
  int err = ipc->Publish(dgc::SimulatorTagID, &tag);
  dgc::TestIpcExit(err, "Could not publish", dgc::SimulatorTagID);
}

void publish_can(vehicle_state *vehicle)
{
  static CanStatus can;
  static int first = 1;

  if(first) {
    memset(&can, 0, sizeof(can));
    strncpy(can.host, dgc_hostname(), 10);
    first = 0;
  }
  vehicle->fill_can_message(&can);
  can.timestamp = dgc_get_time();
  int err = ipc->Publish(CanStatusID, &can);
  TestIpcExit(err, "Could not publish", CanStatusID);
}

void multisim_register_messages(void)
{
  IpcMessageID messages[] = {
      CanStatusID, PassatActuatorID, ApplanixPoseID, LocalizePoseID,
      LdlrsLaser1ID, LdlrsLaser2ID,
      RadarSensor1ID, RadarSensor2ID, RadarSensor3ID, RadarSensor4ID,
      RadarSensor5ID, PerceptionObstaclesID, PerceptionStopZonesID,
      PerceptionMapDiffID, SimulatorGroundTruthID, SimulatorTagID,
      SimulatorStartID, TrafficLightListMsgID
  };
  ipc->DefineMessageArray(messages, sizeof(messages) / sizeof(*messages));
}

void multisim_subscribe_messages(void)
{
  ipc->Subscribe(PassatActuatorID, &passat_actuator_handler);
  ipc->Subscribe(PlannerFsmStateID, &planner_state_handler);
  ipc->Subscribe(EstopStatusID, &estop_status_handler);
}

void get_start_state(ParamInterface *pint, int argc, char **argv,
    double *lat, double *lon, double *theta, double *vel,
    double *max_steering, double *max_throttle,
    double *max_brake, int *sim_laser, int *fake_perception)
{
  int   i;

  dgc_transform_t ldlrs1_transform;
  dgc_transform_t radar1_transform, radar2_transform, radar3_transform,
  radar4_transform, radar5_transform;

  Param params[] = {
      {"rndf", "rndf_file", DGC_PARAM_FILENAME, &rndf_filename, 0, NULL},
      {"sim", "vehicle_start_latitude", DGC_PARAM_DOUBLE, lat, 0, NULL},
      {"sim", "vehicle_start_longitude", DGC_PARAM_DOUBLE, lon, 0, NULL},
      {"sim", "vehicle_start_theta", DGC_PARAM_DOUBLE, theta, 0, NULL},
      {"sim", "vehicle_start_velocity", DGC_PARAM_DOUBLE, vel, 0, NULL},
      {"sim", "bicycle_model", DGC_PARAM_ONOFF, &bicycle_model, 0, NULL},
      {"vehiclesim", "torque_mode", DGC_PARAM_ONOFF, &torque_mode, 0, NULL},
      {"sim", "obstacle_map", DGC_PARAM_FILENAME, &obstacle_map_filename, 0, NULL},
      {"sim", "sim_laser", DGC_PARAM_ONOFF, sim_laser, 0, NULL},
      {"sim", "laser_max_range", DGC_PARAM_DOUBLE, &laser_max_range, 0, NULL},
      {"sim", "track_max_range", DGC_PARAM_DOUBLE, &track_max_range, 0, NULL},
      {"sim", "radar_max_range", DGC_PARAM_DOUBLE, &radar_max_range, 0, NULL},
      {"sim", "fake_perception", DGC_PARAM_ONOFF, fake_perception, 0, NULL},

      {"passat", "max_steering", DGC_PARAM_DOUBLE, max_steering, 0, NULL},
      {"passat", "max_throttle", DGC_PARAM_DOUBLE, max_throttle, 0, NULL},
      {"passat", "max_brake", DGC_PARAM_DOUBLE, max_brake, 0, NULL},
      {"transform", "ldlrs_laser1", DGC_PARAM_TRANSFORM, &ldlrs1_transform, 0, NULL},

      {"transform", "radar1", DGC_PARAM_TRANSFORM, &radar1_transform, 0, NULL},
      {"transform", "radar2", DGC_PARAM_TRANSFORM, &radar2_transform, 0, NULL},
      {"transform", "radar3", DGC_PARAM_TRANSFORM, &radar3_transform, 0, NULL},
      {"transform", "radar4", DGC_PARAM_TRANSFORM, &radar4_transform, 0, NULL},
      {"transform", "radar5", DGC_PARAM_TRANSFORM, &radar5_transform, 0, NULL},
  };
  pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));

  fprintf(stderr, "simlaser %d\n", *sim_laser);

  get_sensor_offset(ldlrs1_transform, &ldlrs1_offset);

  get_sensor_offset(radar1_transform, &radar1_offset);
  get_sensor_offset(radar2_transform, &radar2_offset);
  get_sensor_offset(radar3_transform, &radar3_offset);
  get_sensor_offset(radar4_transform, &radar4_offset);
  get_sensor_offset(radar5_transform, &radar5_offset);

  Param  vtag_params[MAX_NUM_VEHICLES];
  char temp[100];
  for (i=0; i<MAX_NUM_VEHICLES; i++) {
    vtag_params[i].module        = "sim";
    snprintf( temp, 50, "vehicle_name%02d", i );
    vtag_params[i].variable      = temp;
    vtag_params[i].type          = DGC_PARAM_STRING;
    vtag_params[i].user_variable = &(vehicle_name[i]);
    vtag_params[i].subscribe     = 0;
    vtag_params[i].cb            = NULL;
  }

  pint->AllowUnfoundVariables(true);
  pint->InstallParams(argc, argv, vtag_params, sizeof(vtag_params) /
      sizeof(vtag_params[0]));
}

//void add_waypoint_data(rndf_file *rndf)
//{
//  int i, j, k;
//  rndf_waypoint *w;
//
//  for(i = 0; i < rndf->num_segments(); i++)
//    for(j = 0; j < rndf->segment(i)->num_lanes(); j++)
//      for(k = 0; k < rndf->segment(i)->lane(j)->num_waypoints(); k++) {
//        w = rndf->segment(i)->lane(j)->waypoint(k);
//        my_data *data = new my_data;
//        w->data = data;
//      }
//
//  for(i = 0; i < rndf->num_zones(); i++) {
//    for(j = 0; j < rndf->zone(i)->num_perimeter_points(); j++) {
//      w = rndf->zone(i)->perimeter(j);
//      my_data *data = new my_data;
//      w->data = data;
//    }
//  }
//}

} // namespace vlr

using namespace vlr;

int main(int argc, char **argv)
{
  int i, j, iter = 0;
  double left, start_time, current_time, last_printout = 0;
  double vel;
  double max_steering, max_brake, max_throttle;
  int tenhz = 0;
  int load_map = 0;
  int *sim_laser;
  int *fake_perception;

  ParamInterface *pint;

  vehicle_name = (char **)malloc(MAX_NUM_VEHICLES * sizeof(char *));
  dgc_test_alloc(vehicle_name);
  for(i = 0; i < MAX_NUM_VEHICLES; i++) {
    vehicle_name[i] = (char *)malloc(20*sizeof(char));
    snprintf(vehicle_name[i], 20, "VEHICLE_%d", i + 1);
  }

  no_plan = (int *)calloc(MAX_NUM_VEHICLES, sizeof(int));
  dgc_test_alloc(no_plan);
  high_lateral_accel = (int *)calloc(MAX_NUM_VEHICLES, sizeof(int));
  dgc_test_alloc(high_lateral_accel);
  high_forward_accel = (int *)calloc(MAX_NUM_VEHICLES, sizeof(int));
  dgc_test_alloc(high_forward_accel);
  last_plan = (double *)calloc(MAX_NUM_VEHICLES, sizeof(double));
  dgc_test_alloc(last_plan);

  first_forward_accel_ts = (double *)calloc(MAX_NUM_VEHICLES, sizeof(double));
  dgc_test_alloc(first_forward_accel_ts);
  bad_forward_accel = (int *)calloc(MAX_NUM_VEHICLES, sizeof(int));
  dgc_test_alloc(bad_forward_accel);

  first_lateral_accel_ts = (double *)calloc(MAX_NUM_VEHICLES, sizeof(double));
  dgc_test_alloc(first_lateral_accel_ts);
  bad_lateral_accel = (int *)calloc(MAX_NUM_VEHICLES, sizeof(int));
  dgc_test_alloc(bad_lateral_accel);


  for(i = 0; i < MAX_NUM_VEHICLES; i++) {
    last_plan[i] = dgc_get_time();
    bad_forward_accel[i] = 0;
    bad_lateral_accel[i] = 0;
  }

  /* connect to all IPC servers */
  ipc = new IpcStandardInterface;
  mc = new MultiCentral(ipc, true);      // allow zero centrals
  if (mc->Connect(argc, argv, multisim_register_messages,
      multisim_subscribe_messages) < 0)
    dgc_fatal_error("Could not start multi-central connection.");

  usleep(100000);

  mc->StartMonitoringCentrals(NULL);

  pint = new ParamInterface(ipc);

  /* initialize the vehicles */
  num_vehicles = mc->NumCentrals();
  fprintf(stderr, "Simluation includes %d vehicles.\n", num_vehicles);
  vehicle = new vehicle_state[num_vehicles];
  sim_laser  = new int[num_vehicles];
  fake_perception  = new int[num_vehicles];
  start_lat = new double[num_vehicles];
  start_lon = new double[num_vehicles];
  start_theta = new double[num_vehicles];


  for(i = 0; i < num_vehicles; i++) {

    vehicle[i].set_passat_params();
    ipc->SetContext(mc->Context(i));
    get_start_state(pint, argc, argv, &start_lat[i], &start_lon[i], &start_theta[i],
        &vel, &max_steering,
        &max_throttle, &max_brake, &sim_laser[i],
        &fake_perception[i]);
    if(sim_laser[i])
      load_map = 1;
    vehicle[i].reset();
    vehicle[i].param.max_steering = max_steering;
    vehicle[i].param.max_throttle = max_throttle;
    vehicle[i].param.max_brake = max_brake;
    vehicle[i].set_position(start_lat[i], start_lon[i], start_theta[i]);
    vehicle[i].set_velocity(vel, 0);
    vehicle[i].param.bicycle_model = bicycle_model;
    vehicle[i].param.torque_mode = torque_mode;
  }


  /* load the RNDF file, if available */
//  rndf = new rndf_file;
  if(rn.loadRNDF(rndf_filename)) {rndf_valid = 1;}
  if(rndf_valid)
  {
    sz_finder = new stop_zone_finder(rn);
    lights = new TrafficLightSimulator(rn, ipc);
    crosswalks = new CrosswalkSimulator(rn, ipc);
  }
  usleep(100000);

//  add_waypoint_data(rndf);

  if(load_map)
    vlr::load_obstacle_map(obstacle_map_filename);

  /* handle IPC messages */
  for (i = 0; i < mc->NumCentrals(); i++)
    if (mc->Connected(i)) {
      ipc->SetContext(mc->Context(i));
      publish_start();
    }

  start_time = dgc_get_time();
  do {
    tenhz = !tenhz;

    /* update the vehicle positions */
    for(i = 0; i < num_vehicles; i++) {
      for(j = 0; j < 5; j++)
        vehicle[i].update(1.0 / 100.0);
    }

    if(load_map)
      vlr::fill_car_segments(num_vehicles, vehicle);

    if(tenhz) {
      precompute_stop_zones();
      precompute_fake_obstacles();
      crosswalks->update(dgc_get_time());
      compute_warnings();
    }

    mc->ReconnectCentrals();
    for (i = 0; i < mc->NumCentrals(); i++)
      if (mc->Connected(i)) {
        ipc->SetContext(mc->Context(i));

        publish_applanix(&vehicle[i]);
        publish_can(&vehicle[i]);
        publish_groundtruth();

        if(tenhz) {
          publish_collision_info();
          publish_localize(&vehicle[i]);

          if(fake_perception[i]) {
            publish_map_diff(i);
            publish_dirk_obstacles(i);
            publish_stop_zones(i);
            publish_traffic_lights();
          }

          else if(sim_laser[i]) {
            publish_ldlrs1(&vehicle[i]);
          }
          publish_radar(i, 1);
          publish_radar(i, 2);
          publish_radar(i, 3);
          publish_radar(i, 4);
          publish_radar(i, 5);
        }

        if(iter % 20 == 0)
          publish_tags();

        IPC_handleMessage(0);
        IPC_handleMessage(0);
        IPC_handleMessage(0);
      }

    iter++;
    current_time = dgc_get_time();
    left = (start_time + iter * 0.05) - current_time;
    if(left > 0)
      usleep((int)(left * 1e6));
    if(current_time - last_printout > 0.5 && tenhz) {
      //fprintf(stderr, "\rSIM:   %.1f%% idle      \n", left / 0.05 * 100.0);
      last_printout = current_time;
    }
  } while(1);

  delete lights;

  return 0;
}
