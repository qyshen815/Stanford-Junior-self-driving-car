#include <roadrunner.h>
#include <rndf.h>
#include <vector>
#include "smoother.h"
#include "spline.h"

using std::vector;
using namespace dgc;

#define    WAYPOINT_DIST          0.2
#define    CURVATURE_DECEL        1.0
#define    MAX_LATERAL_ACCEL      3.0
#define    MAX_VELOCITY_MPH      70.0

void smooth_lane(rndf_lane *lane)
{
  double *x, *y, *std;
  smoother_p smoother;
  int i, j;
  char str[200];
  rndf_waypoint *w;
  dgc_trajectory_p t;
  double l = 0, current_std = 2;

  x = new double[lane->num_waypoints()];
  y = new double[lane->num_waypoints()];
  std = new double[lane->num_waypoints()];

  /* create the position standard deviation vector */
  current_std = 2.0;
  for(i = 0; i < lane->num_waypoints(); i++) {
    x[i] = lane->waypoint(i)->utm_x();
    y[i] = lane->waypoint(i)->utm_y();

    /* compute original link length */
    if(lane->waypoint(i)->original()) {
      j = i;
      l = 0;
      while(j < lane->num_waypoints() && (i == j || 
                                          !lane->waypoint(j)->original())) {
        l += lane->waypoint(j)->length();
        j++;
      }
      if(l != 0)
        current_std = 30.0 / l;
      if(current_std < 0.25)
        current_std = 0.25;
      current_std = 5;
    }
    std[i] = (lane->waypoint(i)->original()) ? 0.01 : current_std;
  }
  
  /* smooth the lane */
  smoother = smoother_init(x, y, std, lane->num_waypoints(), 1e-8, 1e-3);
  smooth_path(smoother, 1000, 1000, 50, 1e-3, 0, 0, str);
  t = smoother_trajectory(smoother);
  for(i = 0; i < t->num_waypoints; i++) {
    w = lane->waypoint(i);
    w->set_utm(t->waypoint[i].x, t->waypoint[i].y, w->utmzone());
  }
  dgc_trajectory_free(t);
  smoother_free(smoother);
  delete x;
  delete y;
  delete std;
}

inline double max_curvature_speed(double curvature, double lateral_accel)
{
  double v, v2;

  v = dgc_mph2ms(MAX_VELOCITY_MPH);
  if(fabs(1 / curvature) < 1000.0) {
    v2 = sqrt(lateral_accel / fabs(curvature));
    if(v2 < v)
      v = v2;
  }
  return v;
}

typedef struct {
  double x, y, theta, curvature, velocity;
} waypoint_t;

inline int max_previous_velocity(double x1, double y1, double *v1, 
                                 double x2, double y2, double v2)
{
  double d, new_v1;

  d = hypot(x2 - x1, y2 - y1);
  new_v1 = sqrt(dgc_square(v2) + CURVATURE_DECEL * d);
  if(*v1 > new_v1 + 0.01) {
    *v1 = new_v1;
    return 1;
  }
  return 0;
}

inline int max_next_velocity(double x1, double y1, double v1, 
                             double x2, double y2, double *v2)
{
  double d, new_v2;

  d = hypot(x2 - x1, y2 - y1);
  new_v2 = sqrt(dgc_square(v1) + CURVATURE_DECEL * d);
  if(*v2 > new_v2 + 0.01) {
    *v2 = new_v2;
    return 1;
  }
  return 0;
}

void smooth_velocities(vector <waypoint_t> *traj)
{
  int i, n, changed;

  n = (signed)(*traj).size();
  do {
    changed = 0;
    for(i = n - 1; i >= 0; i--)
      changed |= max_previous_velocity((*traj)[i].x,
                                       (*traj)[i].y,
                                       &(*traj)[i].velocity,
                                       (*traj)[(i + 1) % n].x,
                                       (*traj)[(i + 1) % n].y,
                                       (*traj)[(i + 1) % n].velocity);

    for(i = 0; i < n; i++)
      changed |= max_next_velocity((*traj)[i].x,
                                   (*traj)[i].y,
                                   (*traj)[i].velocity,
                                   (*traj)[(i + 1) % n].x,
                                   (*traj)[(i + 1) % n].y,
                                   &(*traj)[(i + 1) % n].velocity);
  } while(changed);
}

void write_simple_trajectory(rndf_file *rndf, char *filename)
{
  dgc_FILE *fp;
  rndf_lane *lane;
  vector <waypoint_t> temp_traj;
  waypoint_t temp_waypoint;
  rndf_waypoint *w;
  int i, k, n, l;
  cubic_spline spline;
  vlr::dgc_point2D_t p, p2;

  if(rndf->num_segments() == 0)
    dgc_die("Error: RNDF must have at least one segment.\n");
  if(rndf->segment(0)->num_lanes() == 0)
    dgc_die("Error: RNDF first segment must have at least one lane.\n");
  if(rndf->segment(0)->lane(0)->num_waypoints() < 2)
    dgc_die("Erorr: RNDF segment 0 lane 0 must have at least two waypoints\n");

  fp = dgc_fopen(filename, "w");
  if(fp == NULL)
    dgc_die("Error: could not open file %s for writing.\n", filename);

  lane = rndf->segment(0)->lane(0);

  /* smooth lane using CG */
  smooth_lane(lane);
  
  /* do spline interpolation to further increase resolution */
  for(k = 0; k < lane->num_waypoints(); k++) {
    w = lane->waypoint(k);
    
    /* print lane waypoints to file */
    if(k < lane->num_waypoints() - 1) {
      n = (int)floor(w->length() / WAYPOINT_DIST) - 1;
      if(n < 2)
	n = 2;
      spline.fit(w->utm_x(), w->utm_y(), w->heading(),
		 w->next()->utm_x(), w->next()->utm_y(), 
		 w->next()->heading());

      for(l = 0; l <= n; l++) {
	p = spline.eval(l / (double)(n + 1));
	temp_waypoint.x = p.x;
	temp_waypoint.y = p.y;
	p = spline.deriv_eval(l / (double)(n + 1));
	temp_waypoint.theta = atan2(p.y, p.x);
	p2 = spline.deriv2_eval(l / (double)(n + 1));
	temp_waypoint.curvature = (p.x * p2.y - p.y * p2.x) /
	  pow(p.x * p.x + p.y * p.y, 1.5);
	temp_traj.push_back(temp_waypoint);
      }
    }
    else {
      temp_waypoint.x = w->utm_x();
      temp_waypoint.y = w->utm_y();
      temp_waypoint.theta = w->heading();
      temp_waypoint.curvature = 0;
      temp_traj.push_back(temp_waypoint);
    }
  }

  for(i = 0; i < (signed)temp_traj.size(); i++)
    temp_traj[i].velocity = max_curvature_speed(temp_traj[i].curvature,
						MAX_LATERAL_ACCEL);

  smooth_velocities(&temp_traj);

  dgc_fprintf(fp, "%d 0\n", (signed)temp_traj.size());
  for(i = 0; i < (signed)temp_traj.size(); i++)
    dgc_fprintf(fp, "%f %f %f %f %f\n", 
	       temp_traj[i].x, temp_traj[i].y, temp_traj[i].theta,
	       temp_traj[i].velocity, 0);

  dgc_fclose(fp);

  fprintf(stderr, "Use starting position: lat %.6f lon %.6f theta %f\n", 
	  lane->waypoint(0)->lat(),
	  lane->waypoint(0)->lon(),
	  temp_traj[0].theta);
}

int main(int argc, char **argv)
{
  rndf_file *rndf;
  char outfilename[200];

  if(argc < 2)
    dgc_die("Error: not enough arguments.\n"
            "Usage: %s rndf-file\n", argv[0]);
  if(strcmp(argv[1] + strlen(argv[1]) - 4, ".txt") != 0)
    dgc_die("Error: RNDF or SRNDF files must end in .txt");

  rndf = new rndf_file;
  if(rndf->load(argv[1]) < 0)
    dgc_die("Error: could not read RNDF file %s\n", argv[1]);

  strcpy(outfilename, argv[1]);
  strcpy(outfilename + strlen(outfilename) - 4, ".traj");

  fprintf(stderr, "Upsampling to 2 m waypoint distance\n");
  rndf->upsample(2.0);

  write_simple_trajectory(rndf, outfilename);

  return 0;
}
