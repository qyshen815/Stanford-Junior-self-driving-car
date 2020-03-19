#include <roadrunner.h>
#include <trajectory.h>
#include "smoother.h"

smoother_p smoother_init(double *x, double *y, double *std, int n,
                         double epsilon, double k_lateral)
{
  smoother_p s;
  int i;
  
  s = (smoother_p)calloc(1, sizeof(smoother_t));
  dgc_test_alloc(s);

  s->n = n;
  s->epsilon = epsilon;

  /* allocate internal memory */
  s->x0 = (vec2 *)calloc(s->n, sizeof(vec2));
  dgc_test_alloc(s->x0);
  s->x = (vec2 *)calloc(s->n, sizeof(vec2));
  dgc_test_alloc(s->x);
  s->r = (double *)calloc(s->n, sizeof(double));
  dgc_test_alloc(s->r);
  s->var = (double *)calloc(s->n, sizeof(double));
  dgc_test_alloc(s->var);
  s->mag = (double *)calloc(s->n, sizeof(double));
  dgc_test_alloc(s->mag);
  s->edg = (vec2 *)calloc(s->n, sizeof(vec2));
  dgc_test_alloc(s->edg);
  s->tgt = (vec2 *)calloc(s->n, sizeof(vec2));
  dgc_test_alloc(s->tgt);
  s->cdp = (double *)calloc(s->n, sizeof(double));
  dgc_test_alloc(s->cdp);
  s->f = (double *)calloc(s->n, sizeof(double));
  dgc_test_alloc(s->f);
  s->df_dx = (vec2 *)calloc(s->n, sizeof(vec2));
  dgc_test_alloc(s->df_dx);
  s->ocl = (vec2 *)calloc(s->n, sizeof(vec2));
  dgc_test_alloc(s->ocl);
  s->ocr = (vec2 *)calloc(s->n, sizeof(vec2));
  dgc_test_alloc(s->ocr);
  s->sdx = (vec2 *)calloc(s->n, sizeof(vec2));
  dgc_test_alloc(s->ocr);
  s->cp = (vec2 *)calloc(s->n, sizeof(vec2));
  dgc_test_alloc(s->cp);
  s->xcopy = (vec2 *)calloc(s->n, sizeof(vec2));
  dgc_test_alloc(s->xcopy);

  /* compute centroid */
  s->origin_x = 0;
  s->origin_y = 0;
  for(i = 0; i < n; i++) {
    s->origin_x += x[i];
    s->origin_y += y[i];
  }
  s->origin_x /= (double)n;
  s->origin_y /= (double)n;

  /* make a copy of the orignal point locations */
  for(i = 0; i < s->n; i++) {
    s->x[i][0] = x[i] - s->origin_x;
    s->x[i][1] = y[i] - s->origin_y;
    s->var[i] = dgc_square(std[i]);
    s->r[i] = 10.0;
    s->x0[i][0] = s->x[i][0];
    s->x0[i][1] = s->x[i][1];
  }
  for(i = 0; i < s->n - 1; i++)
    s->mag[i] = hypot(s->x[i + 1][0] - s->x[i][0],
                      s->x[i + 1][1] - s->x[i][1]);
  find_potential(s, k_lateral);
  s->iter_count = 0;
  return s;
}

void smoother_free(smoother_p s)
{
  free(s->x0);
  free(s->x);
  free(s->r);
  free(s->mag);
  free(s->edg);
  free(s->tgt);
  free(s->cdp);
  free(s->f);
  free(s->df_dx);
  free(s->ocl);
  free(s->ocr);
  free(s->sdx);
  free(s);
}

dgc_trajectory_p smoother_trajectory(smoother_p s)
{
  dgc_trajectory_p t;
  int i;
  double d = 0;

  t = dgc_trajectory_init(s->n, 0);

  /* add offset */
  for(i = 0; i < t->num_waypoints; i++) {
    t->waypoint[i].x = s->x[i][0] + s->origin_x;
    t->waypoint[i].y = s->x[i][1] + s->origin_y;
  }

  /* compute orientations */
  for(i = 0; i < t->num_waypoints - 1; i++) {
    t->waypoint[i].theta = atan2(t->waypoint[i + 1].y - t->waypoint[i].y,
                                 t->waypoint[i + 1].x - t->waypoint[i].x);
  }
  t->waypoint[t->num_waypoints - 1].theta = 
    t->waypoint[t->num_waypoints - 2].theta;

  t->waypoint[0].cumulative_distance = 0;
  for(i = 1; i < t->num_waypoints; i++) {
    d += hypot(t->waypoint[i].y - t->waypoint[i - 1].y,
               t->waypoint[i].x - t->waypoint[i - 1].x);
    t->waypoint[i].cumulative_distance = d;
  }
  t->total_distance = d;
  return t;
}

void smoother_update_trajectory(smoother_p s, dgc_trajectory_p t)
{
  int i;
  double d = 0;

  /* add offset */
  for(i = 0; i < t->num_waypoints; i++) {
    t->waypoint[i].x = s->x[i][0] + s->origin_x;
    t->waypoint[i].y = s->x[i][1] + s->origin_y;
  }

  /* compute orientations */
  for(i = 0; i < t->num_waypoints - 1; i++) {
    t->waypoint[i].theta = atan2(t->waypoint[i + 1].y - t->waypoint[i].y,
                                 t->waypoint[i + 1].x - t->waypoint[i].x);
  }
  t->waypoint[t->num_waypoints - 1].theta = 
    t->waypoint[t->num_waypoints - 2].theta;

  t->waypoint[0].cumulative_distance = 0;
  for(i = 1; i < t->num_waypoints; i++) {
    d += hypot(t->waypoint[i].y - t->waypoint[i - 1].y,
               t->waypoint[i].x - t->waypoint[i - 1].x);
    t->waypoint[i].cumulative_distance = d;
  }
  t->total_distance = d;
}

void find_potential(smoother_p s, double k_lateral)
{
  int i, j;

  /* compute current edge and normalized tangent vectors */
  for(i = 0; i < s->n - 1; i++) {
    for(j = 0; j < 2; j++) 
      s->edg[i][j] = (s->x[i + 1][j] - s->x[i][j]);
    s->mag[i] = hypot(s->edg[i][0], s->edg[i][1]);
    for(j = 0; j < 2; j++) 
      s->tgt[i][j] = s->edg[i][j] / s->mag[i];
  }

  /* compute the left and right orthogonal complements */
  for(i = 1; i < s->n - 1; i++) {
    s->cdp[i] = s->tgt[i - 1][0] * s->tgt[i][0] + 
      s->tgt[i - 1][1] * s->tgt[i][1];
    for(j = 0; j < 2; j++) {
      s->ocl[i][j] = s->tgt[i - 1][j] - s->cdp[i] * s->tgt[i][j];
      s->ocr[i][j] = s->tgt[i][j] - s->cdp[i] * s->tgt[i - 1][j];
    }
  }

  /* calculate potentials */
  s->f_total = 0.0;
  for(i = 1; i < s->n - 1; i++) {
    s->f[i] = 
      (1.0 - s->cdp[i]) +                       /* curvature */
      k_lateral * (dgc_square(s->x[i][0] - s->x0[i][0]) + /* lateral offset */
                   dgc_square(s->x[i][1] - s->x0[i][1])) / s->var[i];
    s->f_total += s->f[i];
  }

  /* calculate the derivative of the potential */
  for(i = 2; i < s->n - 2; i++) 
    for(j = 0; j < 2; j++)
      s->df_dx[i][j] = -(s->ocl[i - 1][j] / s->mag[i - 1] +
                         s->ocr[i][j] / s->mag[i - 1] - 
                         s->ocl[i][j] / s->mag[i] -
                         s->ocr[i + 1][j] / s->mag[i])
        + 2 * k_lateral * (s->x[i][j] - s->x0[i][j]) / s->var[i];

}

void find_search_direction(smoother_p s, int iter_for_cg_restart)
{
  int i, j;
  double beta;

  s->rho[s->iter_count % 2] = 0.0;
  for(i = 2; i < s->n - 2; i++) 
    for(j = 0; j < 2; j++) 
      s->rho[s->iter_count % 2] += s->df_dx[i][j] * s->df_dx[i][j];

  if(s->iter_count % iter_for_cg_restart == 0) {
    for(i = 2; i < s->n - 2; i++) 
      for(j = 0; j < 2; j++) 
        s->sdx[i][j] = -s->df_dx[i][j];
  } 
  else {
    beta = s->rho[s->iter_count % 2] / s->rho[(s->iter_count - 1) % 2];
    for(i = 2; i < s->n - 2; i++) 
      for(j = 0; j < 2; j++) 
        s->sdx[i][j] = beta * s->sdx[i][j] - s->df_dx[i][j];
  }
}

int line_search(smoother_p s, double k_lateral, int use_pad, double pad)
{
  int i, j;
  double f0, f1, f2, a, b, step, magdev, maxdev;
  vec2 dev;

  magdev = 0;
  maxdev = 0;
  pad = pad;

  /* store the current global potential */
  f0 = s->f_total;

  /* move a little bit in search direction */
  for(i = 2; i < s->n - 2; i++) 
    for(j = 0; j < 2; j++) 
      s->x[i][j] += s->epsilon * s->sdx[i][j];
  
  /* recompute the potential and store the new global potential */
  find_potential(s, k_lateral);
  f1 = s->f_total;

  /* move a little bit more in search direction */
  for(i = 2; i < s->n - 2; i++) 
    for(j = 0; j < 2; j++) 
      s->x[i][j] += s->epsilon * s->sdx[i][j];
  
  /* recompute the potential and store the new global potential */
  find_potential(s, k_lateral);
  f2 = s->f_total;
  
  /* select optimal step size accordingly and move */
  a = (f2 - 2 * f1 + f0) / (2 * dgc_square(s->epsilon));
  b = (f2 - f0) / (2 * s->epsilon) - 2.0 * a * s->epsilon;
  step = -b / (2 * a);

  if(a == 0) 
    return 1;

  for(i = 2; i < s->n - 2; i++) {
    for(j = 0; j < 2; j++) {
      s->x[i][j] += (step - 2 * s->epsilon) * s->sdx[i][j];
      dev[j] = (s->x[i][j] - s->cp[i][j]);
    }

    if(use_pad) {
      magdev = hypot(dev[0], dev[1]);
      maxdev = s->r[i] - pad;
      if(maxdev < 0.5)
        maxdev = 0.5;
      if(magdev > maxdev)
        for(j = 0; j < 2; j++) 
          s->x[i][j] = s->cp[i][j] + maxdev * dev[j] / magdev;
    }
  }
  
  /* and set the paramter epsilon according to previous step size */
  s->epsilon = 1e-3 * step;
  return 0;
}

#define MIN(a,b) (((a)<(b))?(a):(b))

int smooth_path(smoother_p s, int num_iter, int max_iter,
                int iter_for_cg_restart, double k_lateral, int use_pad, 
                double pad, char *finish_reason)
{
  int iter, done = 0;
  int i;
  double best_f = 1e6;

  finish_reason[0] = '\0';
  for(iter = 0; iter < num_iter && s->iter_count < max_iter; iter++) {
    /* make backup copy of path */
    for(i = 0; i < s->n; i++) {
      s->xcopy[i][0] = s->x[i][0];
      s->xcopy[i][1] = s->x[i][1];
      best_f = s->f_total;
    }
      
    find_potential(s, k_lateral);
    find_search_direction(s, iter_for_cg_restart);
    done = line_search(s, k_lateral, use_pad, pad);
    if(done) {
      strcpy(finish_reason, "CONVERGED");
      break;
    }

    if(s->iter_count > 2 && s->f_total > best_f * 1.05) {
      for(i = 0; i < s->n; i++) {
        s->x[i][0] = s->xcopy[i][0];
        s->x[i][1] = s->xcopy[i][1];
      }
      s->f_total = best_f;
      strcpy(finish_reason, "METRIC INCREASING");
      done = 1;
      break;
    }
      
    s->iter_count++;
  }
  return done;
}

