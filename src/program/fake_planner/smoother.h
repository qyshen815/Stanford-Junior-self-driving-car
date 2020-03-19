#ifndef DGC_SMOOTHER_H
#define DGC_SMOOTHER_H

#include <trajectory.h>

typedef double vec2[2];

typedef struct {
  int n;
  double origin_x, origin_y;
  double epsilon;
  vec2 rho;                  /* CG memory parameters */
  vec2 *x0;                  /* original x-y position of the points */
  vec2 *x;                   /* current x-y position of the points */
  double *var;
  vec2 *edg;                 /* edge vector */
  vec2 *tgt;                 /* normalized tangent vector */
  vec2 *ocl;                 /* left orthogonal complement */
  vec2 *ocr;                 /* right orthogonal complement */
  double *r;                 /* lateral offset limit */
  double *mag;               /* mean length of the segments */
  double *cdp;               /* cosine of angle change at each point */
  double *f;                 /* local value of potential */
  double f_total;            /* sum of all local potentials */
  vec2 *df_dx;               /* derivative of global potential wrt x */
  vec2 *sdx;                 /* search direction for CG */
  vec2 *cp;
  vec2 *xcopy;
  int iter_count;
} smoother_t, *smoother_p;

smoother_p smoother_init(double *x, double *y, double *std, int n,
                         double epsilon, double k_lateral);

void smoother_free(smoother_p s);

dgc_trajectory_p smoother_trajectory(smoother_p s);

void smoother_update_trajectory(smoother_p s, dgc_trajectory_p t);

int smooth_path(smoother_p s, int num_iter, int max_iter,
                int iter_for_cg_restart, double k_lateral, int use_pad, 
                double pad, char *finish_reason);

void find_potential(smoother_p s, double k_lateral);

#endif
