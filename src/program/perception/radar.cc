#include "perception.h"
#include "utils.h"


dgc_transform_t     radar_offset[NUM_RADARS];
double              radar_ts[NUM_RADARS] = {0,0,0,0,0,0};

using namespace dgc;
using namespace vlr;
using namespace std;
using namespace std::tr1;

/*void perception_add_radar(vector< shared_ptr<RadarObservation> >& obstacles, double timestamp) {
  ApplanixPose* current = applanix_current_pose();
  dgc_transform_t robot_to_smooth;
  dgc_transform_identity(robot_to_smooth);

  dgc_transform_rotate_x( robot_to_smooth, current->roll );
  dgc_transform_rotate_y( robot_to_smooth, current->pitch );
  dgc_transform_rotate_z( robot_to_smooth, current->yaw );
  dgc_transform_translate( robot_to_smooth, current->smooth_x, current->smooth_y, current->smooth_z );

  float applanix_velocity = applanix_current_pose()->speed;

  // front radar
  dgc_transform_t radar_to_smooth;
  dgc_transform_copy(radar_to_smooth, radar_offset[2]);
  dgc_transform_left_multiply(radar_to_smooth, robot_to_smooth);
  for (int i = 0; i < radar_lrr3[0].num_targets; i++) {
    RadarLRR3Target* target = &(radar_lrr3[0].target[i]);
    double x = target->long_distance;
    double y = target->lateral_distance;
    double z = 0.0;

    double x_vel = target->long_relative_velocity;
    double y_vel = target->lateral_relative_velocity;

    dgc_transform_point(&x, &y, &z, radar_to_smooth);
    dgc_transform_point(&x_vel, &y_vel, &z, radar_to_smooth);

    shared_ptr<RadarObservation> obstacle(new RadarObservation());
    obstacle->x = x;
    obstacle->y = y;
    obstacle->x_vel = x_vel;
    obstacle->y_vel = y_vel;
    obstacles.push_back(obstacle);
  }

  if (radar_lrr3[0].timestamp > radar_ts[2]) {
    radar_ts[2] = radar_lrr3[0].timestamp;
  }

}*/
