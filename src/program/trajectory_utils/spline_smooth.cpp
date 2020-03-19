#include <roadrunner.h>
#include <trajectory.h>

#define TURN_DIST 5.0

dgc_trajectory_p do_spline_interpolation(dgc_trajectory_p t_in, 
                                         double min_interp_dist)
{
  dgc_trajectory_p t_temp, t_out;
  int i, j, num_poses, max_poses, extra_points;
  double l, x1, y1, theta1, x2, y2, theta2, t;
  double k, ax, bx, cx, dx, ay, by, cy, dy;
  int num_waypoints, max_waypoints;
  double frac;

  /* prepare trajectory 3 for corner rounding */
  num_waypoints = 0;
  max_waypoints = 10000;

  t_temp = dgc_trajectory_init(max_waypoints, t_in->looping);
  t_temp->waypoint[0] = t_in->waypoint[0];
  num_waypoints++;

  for(i = 1; i < t_in->num_waypoints; i++) {
    l = hypot(t_in->waypoint[i].x - t_in->waypoint[i - 1].x,
              t_in->waypoint[i].y - t_in->waypoint[i - 1].y);

    frac = 0.45;
    if(frac * l > TURN_DIST)
      frac = TURN_DIST / l;

    if(num_waypoints == max_waypoints) {
      max_waypoints += 10000;
      dgc_trajectory_realloc(t_temp, max_waypoints);
    }
    t_temp->waypoint[num_waypoints].x = t_in->waypoint[i - 1].x +
      frac * (t_in->waypoint[i].x - t_in->waypoint[i - 1].x);
    t_temp->waypoint[num_waypoints].y = t_in->waypoint[i - 1].y +
      frac * (t_in->waypoint[i].y - t_in->waypoint[i - 1].y);
    t_temp->waypoint[num_waypoints].theta = t_in->waypoint[i - 1].theta;
    num_waypoints++;
    
    if(num_waypoints == max_waypoints) {
      max_waypoints += 10000;
      dgc_trajectory_realloc(t_temp, max_waypoints);
    }
    t_temp->waypoint[num_waypoints].x = t_in->waypoint[i - 1].x +
      (1 - frac) * (t_in->waypoint[i].x - t_in->waypoint[i - 1].x);
    t_temp->waypoint[num_waypoints].y = t_in->waypoint[i - 1].y +
      (1 - frac) * (t_in->waypoint[i].y - t_in->waypoint[i - 1].y);
    t_temp->waypoint[num_waypoints].theta = t_in->waypoint[i - 1].theta;
    num_waypoints++;
  }
  t_temp->waypoint[num_waypoints] = t_in->waypoint[t_in->num_waypoints - 1];
  num_waypoints++;
  dgc_trajectory_realloc(t_temp, num_waypoints);

  /* do cubic interpolation between waypoints */
  num_poses = 0;
  max_poses = 10000;

  t_out = dgc_trajectory_init(max_poses, t_in->looping);
  
  for(i = 0; i < t_temp->num_waypoints - 1; i++) {
    /* add previous waypoint */
    if(num_poses == max_poses) {
      max_poses += 10000;
      dgc_trajectory_realloc(t_out, max_poses);
    }
    t_out->waypoint[num_poses].x = t_temp->waypoint[i].x;
    t_out->waypoint[num_poses].y = t_temp->waypoint[i].y;
    num_poses++;

    l = hypot(t_temp->waypoint[i + 1].x - t_temp->waypoint[i].x,
              t_temp->waypoint[i + 1].y - t_temp->waypoint[i].y);
    extra_points = (int)floor(l / min_interp_dist);

    x1 = t_temp->waypoint[i].x;
    y1 = t_temp->waypoint[i].y;
    theta1 = t_temp->waypoint[i].theta;
    x2 = t_temp->waypoint[i + 1].x;
    y2 = t_temp->waypoint[i + 1].y;
    theta2 = t_temp->waypoint[i + 1].theta;
    
    k = l;

    ax = x1;
    ay = y1;
    bx = k * cos(theta1);
    by = k * sin(theta1);
    cx = 3 * (x2 - x1) - 2 * bx - k * cos(theta2);
    cy = 3 * (y2 - y1) - 2 * by - k * sin(theta2);
    dx = bx + k * cos(theta2) + 2 * (x1 - x2);
    dy = by + k * sin(theta2) + 2 * (y1 - y2);

    extra_points = (int)floor(l / min_interp_dist);
    for(j = 0; j < extra_points; j++) {
      if(num_poses == max_poses) {
        max_poses += 10000;
        dgc_trajectory_realloc(t_out, max_poses);
      }
      t = (j + 1) / (double)(extra_points + 1);
      t_out->waypoint[num_poses].x = ax + bx * t + cx * t * t +
        dx * t * t * t;
      t_out->waypoint[num_poses].y = ay + by * t + cy * t * t +
        dy * t * t * t;
      num_poses++;
    }
  }
  
  dgc_trajectory_free(t_temp);

  /* recompute thetas directly from poses */
  dgc_trajectory_realloc(t_out, num_poses);
  for(i = 0; i < t_out->num_waypoints - 1; i++)
    t_out->waypoint[i].theta = atan2(t_out->waypoint[i + 1].y - 
                                     t_out->waypoint[i].y,
                                     t_out->waypoint[i + 1].x - 
                                     t_out->waypoint[i].x);
  t_out->waypoint[t_out->num_waypoints - 1].theta =
    t_out->waypoint[t_out->num_waypoints - 2].theta;
  return t_out;
}

int main(int argc, char **argv)
{
  dgc_trajectory_p t1, t2, t3;
  char input_filename[256], output_filename[256];
  double waypoint_dist;

  if(argc < 3)
    dgc_die("Error: not enough arguments.\n"
            "Usage: %s trajectory-file waypoint-dist\n", argv[0]);
  waypoint_dist = atof(argv[2]);
  
  /* read the trajectory file, if there is one */
  fprintf(stderr, "Reading in trajectory file %s.\n", argv[1]);
  t1 = dgc_trajectory_read(argv[1]);
  fprintf(stderr, "Read %d waypoints.\n", t1->num_waypoints);

  t2 = do_spline_interpolation(t1, waypoint_dist);
  t3 = dgc_trajectory_decimate(t2, waypoint_dist);

  fprintf(stderr, "Output trajectory has %d waypoints.\n", t3->num_waypoints);

  strcpy(input_filename, argv[1]);
  strcpy(output_filename, argv[1]);
  dgc_trajectory_create_filename(input_filename, output_filename);
  fprintf(stderr, "Writing output trajectory %s\n", output_filename);
  dgc_trajectory_write(t3, output_filename);
  return 0;
}
