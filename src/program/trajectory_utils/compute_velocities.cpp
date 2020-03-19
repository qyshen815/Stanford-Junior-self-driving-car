#include <roadrunner.h>
#include <trajectory.h>
#include <kdtree.h>

#define   ACCEL_PARAMETER        1.0
#define   MAX_DECEL_PARAM        1.0
#define   MIN_WAYPOINT_DIST      3.0

dgc_trajectory_p trajectory = NULL;
dgc_kdtree_p trajectory_kdtree = NULL;

double velocity_cap;

double *curvature = NULL;
double *speed = NULL;

void compute_appropriate_velocities(dgc_trajectory_p trajectory)
{
  double d;
  double v, v1;
  int i, changed;

  curvature = (double *)calloc(trajectory->num_waypoints, sizeof(double));
  dgc_test_alloc(curvature);
  for(i = 0; i < trajectory->num_waypoints - 1; i++) {
    d = hypot(trajectory->waypoint[i].x - trajectory->waypoint[i + 1].x,
              trajectory->waypoint[i].y - trajectory->waypoint[i + 1].y);
    curvature[i] = dgc_normalize_theta(trajectory->waypoint[i + 1].theta - 
                                       trajectory->waypoint[i].theta) / d;
  }
  curvature[trajectory->num_waypoints - 1] = 0;

  speed = (double *)calloc(trajectory->num_waypoints, sizeof(double));
  dgc_test_alloc(speed);
  
  for(i = 0; i < trajectory->num_waypoints; i++) {
    speed[i] = velocity_cap;

    /* now check speed based on curvature */
    if(fabs(1 / curvature[i]) < 100.0) {
      v = sqrt(ACCEL_PARAMETER / fabs(curvature[i]));
      if(v < speed[i])
        speed[i] = v;
    }
  }

  /* now make sure we don't decelerate too fast */
  do {
    fprintf(stderr, "Velocity correction pass\n");
    changed = 0;
    for(i = trajectory->num_waypoints - 1; i > 0; i--) {
      d = hypot(trajectory->waypoint[i].x - trajectory->waypoint[i - 1].x,
                trajectory->waypoint[i].y - trajectory->waypoint[i - 1].y);
      v1 = sqrt(dgc_square(speed[i]) + MAX_DECEL_PARAM * d);
      if(speed[i - 1] > v1 + 0.01) {
        speed[i - 1] = v1;
        changed = 1;
      }
    }
  } while(changed);

  /* copy the speeds back into the trajectory */
  for(i = 0; i < trajectory->num_waypoints; i++)
    trajectory->waypoint[i].velocity = speed[i];
}

dgc_trajectory_p downsample_trajectory(dgc_trajectory_p raw_trajectory)
{
  int i, waypoint_count = 0;
  dgc_trajectory_p trajectory;
  double dv1 = 0, dv2, dtheta1, dtheta2;

  fprintf(stderr, "Initial number of waypoints %d\n", 
          raw_trajectory->num_waypoints);
  waypoint_count++;
  for(i = 1; i < raw_trajectory->num_waypoints - 1; i++) {
    dv1 = fabs(raw_trajectory->waypoint[i + 1].velocity -
               raw_trajectory->waypoint[i].velocity);
    dv2 = fabs(raw_trajectory->waypoint[i].velocity -
               raw_trajectory->waypoint[i - 1].velocity);
    dtheta1 = fabs(dgc_normalize_theta(raw_trajectory->waypoint[i + 1].theta -
                                       raw_trajectory->waypoint[i].theta));
    dtheta2 = fabs(dgc_normalize_theta(raw_trajectory->waypoint[i].theta -
                                       raw_trajectory->waypoint[i - 1].theta));

    if(dtheta1 > dgc_d2r(0.1) || dv1 > 0.01 || dv2 > 0.01)
      waypoint_count++;
  }
  waypoint_count++;
  fprintf(stderr, "Downsampled number of waypoints %d\n", waypoint_count);

  trajectory = dgc_trajectory_init(waypoint_count, raw_trajectory->looping);

  trajectory->waypoint[0] = raw_trajectory->waypoint[0];
  waypoint_count = 1;

  for(i = 1; i < raw_trajectory->num_waypoints - 1; i++) {
    dv1 = fabs(raw_trajectory->waypoint[i + 1].velocity -
               raw_trajectory->waypoint[i].velocity);
    dv2 = fabs(raw_trajectory->waypoint[i].velocity -
               raw_trajectory->waypoint[i - 1].velocity);
    dtheta1 = fabs(dgc_normalize_theta(raw_trajectory->waypoint[i + 1].theta -
                                       raw_trajectory->waypoint[i].theta));
    dtheta2 = fabs(dgc_normalize_theta(raw_trajectory->waypoint[i].theta -
                                       raw_trajectory->waypoint[i - 1].theta));

    if(dtheta1 > dgc_d2r(0.1) || dv1 > 0.01 || dv2 > 0.01) {
      trajectory->waypoint[waypoint_count] = raw_trajectory->waypoint[i];
      waypoint_count++;
    }
  }
  trajectory->waypoint[waypoint_count] = 
    raw_trajectory->waypoint[raw_trajectory->num_waypoints - 1];
  return trajectory;
}

dgc_trajectory_p upsample_trajectory(dgc_trajectory_p raw_trajectory)
{
  int i, j, new_waypoints, waypoint_count = 0;
  double d;
  dgc_trajectory_p trajectory;

  for(i = 0; i < raw_trajectory->num_waypoints - 1; i++) {
    d = hypot(raw_trajectory->waypoint[i + 1].x - 
              raw_trajectory->waypoint[i].x,
              raw_trajectory->waypoint[i + 1].y - 
              raw_trajectory->waypoint[i].y);
    waypoint_count++;
    if(d > MIN_WAYPOINT_DIST * 2)
      waypoint_count += (int)floor(d / MIN_WAYPOINT_DIST) - 1;
  }
  waypoint_count++;
  
  fprintf(stderr, "%d upsampled waypoints\n", waypoint_count);

  trajectory = dgc_trajectory_init(waypoint_count, raw_trajectory->looping);
  waypoint_count = 0;
  for(i = 0; i < raw_trajectory->num_waypoints - 1; i++) {
    d = hypot(raw_trajectory->waypoint[i + 1].x - 
              raw_trajectory->waypoint[i].x,
              raw_trajectory->waypoint[i + 1].y - 
              raw_trajectory->waypoint[i].y);
    
    /* move the waypoints 1 cm forward to avoid possibly being on segment
       boundary */
    trajectory->waypoint[waypoint_count].x = 
      raw_trajectory->waypoint[i].x + 0.01 / d * 
      (raw_trajectory->waypoint[i + 1].x -
       raw_trajectory->waypoint[i].x);
    trajectory->waypoint[waypoint_count].y = 
      raw_trajectory->waypoint[i].y + 0.01 / d *
      (raw_trajectory->waypoint[i + 1].y -
       raw_trajectory->waypoint[i].y);
    trajectory->waypoint[waypoint_count].theta = 
      raw_trajectory->waypoint[i].theta;
    waypoint_count++;

    if(d > MIN_WAYPOINT_DIST * 2) {
      new_waypoints = (int)floor(d / MIN_WAYPOINT_DIST) - 1;
      for(j = 1; j <= new_waypoints; j++) {
        trajectory->waypoint[waypoint_count].x = 
          raw_trajectory->waypoint[i].x + j / (double)(new_waypoints + 1) * 
          (raw_trajectory->waypoint[i + 1].x -
           raw_trajectory->waypoint[i].x);
        trajectory->waypoint[waypoint_count].y = 
          raw_trajectory->waypoint[i].y + j / (double)(new_waypoints + 1) * 
          (raw_trajectory->waypoint[i + 1].y -
           raw_trajectory->waypoint[i].y);
        trajectory->waypoint[waypoint_count].theta = 
          raw_trajectory->waypoint[i].theta;
        waypoint_count++;
      }
    }
  }
  trajectory->waypoint[waypoint_count] = 
    raw_trajectory->waypoint[raw_trajectory->num_waypoints - 1];
  waypoint_count++;
  return trajectory;
}

int main(int argc, char **argv)
{
  dgc_trajectory_p trajectory1;
  char filename[200];

  if(argc < 3)
    dgc_die("Error: not enough arguments.\n"
            "Usage: %s trajectory-file velocity_cap\n", argv[0]);

  /* read the trajectory file, if there is one */
  fprintf(stderr, "Reading in trajectory file %s.\n", argv[1]);
  trajectory1 = dgc_trajectory_read(argv[1]);
  fprintf(stderr, "Input trajectory has %d waypoints.\n", 
          trajectory1->num_waypoints);
  velocity_cap = dgc_mph2ms(atof(argv[2]));
  fprintf(stderr, "Velocity cap = %.2f mph\n", dgc_ms2mph(velocity_cap));

  compute_appropriate_velocities(trajectory1);
  trajectory = downsample_trajectory(trajectory1);
  
  fprintf(stderr, "Output trajectory has %d waypoints.\n", 
          trajectory->num_waypoints);
  
  strcpy(filename, argv[1]);
  dgc_trajectory_create_filename(argv[1], filename);
  
  dgc_trajectory_write(trajectory, filename);
  return 0;
}
