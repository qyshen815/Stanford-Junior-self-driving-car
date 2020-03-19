#include <roadrunner.h>
#include <rndf.h>
#include "smoother.h"
#include "spline.h"

using namespace dgc;
using namespace vlr;

#define WAYPOINT_DIST 0.2

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

void write_base_trajectory(rndf_file *rndf, char *filename)
{
  rndf_lane *lane;
  rndf_waypoint *w;
  int i, j, k, l, l2, n, wi, li, si;
  dgc_FILE *fp;
  cubic_spline spline;
  dgc_point2D_t p, p2;
  double last_x = 0, last_y = 0;

  fp = dgc_fopen(filename, "w");
  if(fp == NULL)
    dgc_die("Error: could not open file %s for writing.\n", filename);

  /* loop through every lane */
  for(i = 0; i < rndf->num_segments(); i++)
    for(j = 0; j < rndf->segment(i)->num_lanes(); j++) {
      fprintf(stderr, "Smoothing lane %d.%d\n", i + 1, j + 1);
      lane = rndf->segment(i)->lane(j);

      /* smooth lane using CG */
      smooth_lane(lane);

      /* do spline interpolation to further increase resolution */
      for(k = 0; k < lane->num_waypoints(); k++) {
        dgc_fprintf(fp, "%d %d %d\n", i + 1, j + 1, k + 1);
        w = lane->waypoint(k);

        /* print lane waypoints to file */
        if(k < lane->num_waypoints() - 1) {
          n = (int)floor(w->length() / WAYPOINT_DIST) - 1;
          if(n < 2)
            n = 2;
          spline.fit(w->utm_x(), w->utm_y(), w->heading(),
                     w->next()->utm_x(), w->next()->utm_y(), 
                     w->next()->heading());
          dgc_fprintf(fp, "%d\n", n + 1);
          for(l = 0; l <= n; l++) {
            p = spline.eval(l / (double)(n + 1));
            dgc_fprintf(fp, "%.4f %.4f ", p.x, p.y);
            p = spline.deriv_eval(l / (double)(n + 1));
            dgc_fprintf(fp, "%f ", atan2(p.y, p.x));
            p2 = spline.deriv2_eval(l / (double)(n + 1));
            dgc_fprintf(fp, "%f\n", 
                       (p.x * p2.y - p.y * p2.x) /
                       pow(p.x * p.x + p.y * p.y, 1.5));
          }
        }
        else {
          dgc_fprintf(fp, "1\n");
          dgc_fprintf(fp, "%.4f %.4f %f %f\n", w->utm_x(), w->utm_y(), 
                     w->heading(), 0);
        }

        /* print exit waypoints to file */
        dgc_fprintf(fp, "%d\n", w->num_exits());
        for(l = 0; l < w->num_exits(); l++) {
          /* print exit to waypoint */
          w->exit(l)->lookup_id(&si, &li, &wi);
          dgc_fprintf(fp, "%d %d %d\n", si + 1, li + 1, wi + 1); 

	  /* fit spline to exit */
	  spline.fit(w->utm_x(), w->utm_y(), w->heading(),
		     w->exit(l)->utm_x(), w->exit(l)->utm_y(), 
		     w->exit(l)->heading());
	  n = (int)floor(spline.length(0, 1) / WAYPOINT_DIST) - 1;
	  /* print exit waypoints to file */
	  dgc_fprintf(fp, "%d\n", n + 1);
	  for(l2 = 0; l2 <= n; l2++) {
	    p = spline.eval(l2 / (double)(n + 1));
	    dgc_fprintf(fp, "%.4f %.4f ", p.x, p.y);
	    last_x = p.x; last_y = p.y;
	    p = spline.deriv_eval(l2 / (double)(n + 1));
	    dgc_fprintf(fp, "%f ", atan2(p.y, p.x));
	    p2 = spline.deriv2_eval(l2 / (double)(n + 1));
	    dgc_fprintf(fp, "%f\n", 
		       (p.x * p2.y - p.y * p2.x) /
		       pow(p.x * p.x + p.y * p.y, 1.5));
	  }
        }
      }
    }
  dgc_fclose(fp);
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
  strcpy(outfilename + strlen(outfilename) - 4, ".bt");

  write_base_trajectory(rndf, outfilename);
  return 0;
}
