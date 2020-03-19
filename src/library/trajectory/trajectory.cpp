#include <roadrunner.h>
#include <trajectory.h>
#include <kdtree.h>

dgc_trajectory_p dgc_trajectory_init(int num_waypoints, int looping)
{
  dgc_trajectory_p trajectory;
  
  trajectory = (dgc_trajectory_p)calloc(1, sizeof(dgc_trajectory_t));
  dgc_test_alloc(trajectory);
  trajectory->num_waypoints = num_waypoints;
  trajectory->looping = looping;
  trajectory->waypoint = 
    (dgc_trajectory_waypoint_p)calloc(trajectory->num_waypoints, 
                                      sizeof(dgc_trajectory_waypoint_t));
  dgc_test_alloc(trajectory->waypoint);
  return trajectory;
}

void dgc_trajectory_realloc(dgc_trajectory_p trajectory, int num_waypoints)
{
  trajectory->num_waypoints = num_waypoints;
  trajectory->waypoint = 
    (dgc_trajectory_waypoint_p)realloc(trajectory->waypoint,
                                       trajectory->num_waypoints *
                                       sizeof(dgc_trajectory_waypoint_t));
  dgc_test_alloc(trajectory->waypoint);
}

void dgc_trajectory_free(dgc_trajectory_p trajectory)
{
  free(trajectory->waypoint);
  free(trajectory);
}

dgc_trajectory_p dgc_trajectory_read(char *filename)
{
  FILE *fp;
  dgc_trajectory_p trajectory;
  int i;

  fp = fopen(filename, "r");
  if(fp == NULL) {
    dgc_error("Error: could not open file %s for reading.\n", filename);
    return NULL;
  }
  trajectory = (dgc_trajectory_p)calloc(1, sizeof(dgc_trajectory_t));
  dgc_test_alloc(trajectory);
  if(fscanf(fp, "%d %d\n", &trajectory->num_waypoints, &trajectory->looping)
      != 2) {
    dgc_error("Error reading trajectories from file %s.", filename);
    fclose(fp);
    return NULL;
  }
  trajectory->waypoint = 
    (dgc_trajectory_waypoint_p)calloc(trajectory->num_waypoints, 
                                      sizeof(dgc_trajectory_waypoint_t));
  dgc_test_alloc(trajectory->waypoint);
  for(i = 0; i < trajectory->num_waypoints; i++)
    if(fscanf(fp, "%lf %lf %lf %lf %lf\n", &trajectory->waypoint[i].x,
           &trajectory->waypoint[i].y, &trajectory->waypoint[i].theta,
           &trajectory->waypoint[i].velocity,
           &trajectory->waypoint[i].yaw_rate) != 5) {
    dgc_error("Error reading trajectories from file %s.", filename);
    fclose(fp);
    return NULL;
  }
  fclose(fp);

  trajectory->total_distance = 0.0;
  trajectory->waypoint[0].cumulative_distance = 0;
  for(i = 1; i < trajectory->num_waypoints; i++) {
    trajectory->total_distance += 
      hypot(trajectory->waypoint[i].x - trajectory->waypoint[i - 1].x,
            trajectory->waypoint[i].y - trajectory->waypoint[i - 1].y);
    trajectory->waypoint[i].cumulative_distance =
      trajectory->total_distance;
  }
  
  for(i = 0; i < trajectory->num_waypoints; i++)
    trajectory->waypoint[i].z = -1e6;
  
  return trajectory;
}

void dgc_trajectory_write(dgc_trajectory_p trajectory, char *filename)
{
  FILE *fp;
  int i;

  fp = fopen(filename, "w");
  if(fp == NULL)
    dgc_die("Error: could not open file %s for writing.\n", filename);
  fprintf(fp, "%d %d\n", trajectory->num_waypoints, trajectory->looping);
  for(i = 0; i < trajectory->num_waypoints; i++)
    fprintf(fp, "%lf %lf %lf %lf %lf\n", trajectory->waypoint[i].x,
            trajectory->waypoint[i].y, trajectory->waypoint[i].theta,
            trajectory->waypoint[i].velocity, 
            trajectory->waypoint[i].yaw_rate);
  fclose(fp);
}

dgc_kdtree_p dgc_trajectory_to_kdtree(dgc_trajectory_p trajectory)
{
  double *x, *y;
  int i;
  dgc_kdtree_p kdtree;
  
  x = (double *)calloc(trajectory->num_waypoints, sizeof(double));
  dgc_test_alloc(x);
  y = (double *)calloc(trajectory->num_waypoints, sizeof(double));
  dgc_test_alloc(y);
  for(i = 0; i < trajectory->num_waypoints; i++) {
    x[i] = trajectory->waypoint[i].x;
    y[i] = trajectory->waypoint[i].y;
  }
  kdtree = dgc_kdtree_build_balanced(x, y, trajectory->num_waypoints);
  free(x);
  free(y);
  return kdtree;
}

dgc_trajectory_p dgc_trajectory_decimate(dgc_trajectory_p t_in, 
                                         double waypoint_dist)
{
  dgc_trajectory_p t_out;
  int i, last_i = -1, num_waypoints, max_waypoints;
  double d;

  num_waypoints = 0;
  max_waypoints = 10000;

  t_out = dgc_trajectory_init(max_waypoints, t_in->looping);
  t_out->waypoint[0] = t_in->waypoint[0];
  num_waypoints++;
  
  for(i = 1; i < t_in->num_waypoints; i++) {
    d = hypot(t_in->waypoint[i].x - t_out->waypoint[num_waypoints - 1].x,
              t_in->waypoint[i].y - t_out->waypoint[num_waypoints - 1].y);
    if(d > waypoint_dist) {
      if(num_waypoints == max_waypoints) {
        max_waypoints += 10000;
        dgc_trajectory_realloc(t_out, max_waypoints);
      }
      t_out->waypoint[num_waypoints] = t_in->waypoint[i];
      last_i = i;
      num_waypoints++;
    }
  }
  if(last_i != t_in->num_waypoints - 1) {
    t_out->waypoint[num_waypoints] = t_in->waypoint[t_in->num_waypoints - 1];
    num_waypoints++;
  }

  dgc_trajectory_realloc(t_out, num_waypoints);

  /* recompute thetas */
  for(i = 0; i < t_out->num_waypoints - 1; i++)
    t_out->waypoint[i].theta = atan2(t_out->waypoint[i + 1].y - 
                                     t_out->waypoint[i].y,
                                     t_out->waypoint[i + 1].x - 
                                     t_out->waypoint[i].x);
  t_out->waypoint[t_out->num_waypoints - 1].theta =
    t_out->waypoint[t_out->num_waypoints - 2].theta;

  return t_out;
}

void dgc_trajectory_create_filename(char *input_filename, 
                                    char *output_filename)
{
  int i = 0, next_num;

  if(strcmp(input_filename + strlen(input_filename) - 5, ".traj") == 0) {
    i = strlen(input_filename) - 6;
    while(i >= 0 && (input_filename[i] >= '0' && input_filename[i] <= '9'))
      i--;
    if(i < 0)
      i = strlen(input_filename) - 5;
    else
      i++;
  }
  else
    dgc_die("Error: Trajectory file must end in .traj\n");

  if(input_filename[i] == '.')
    next_num = 1;
  else {
    sscanf(input_filename + i, "%d.", &next_num);
    next_num++;
  }
  
  sprintf(output_filename + i, "%d.traj", next_num);
}

