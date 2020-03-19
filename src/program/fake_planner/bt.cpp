#include <roadrunner.h>
#include <rndf.h>
#include <mdf.h>
#include "planner_data.h"
#include "bt.h"

using namespace dgc;

inline double max_curvature_speed(double curvature, double lateral_accel)
{
  double v, v2;

  v = dgc_mph2ms(30.0);
  if(fabs(1 / curvature) < 1000.0) {
    v2 = sqrt(lateral_accel / fabs(curvature));
    if(v2 < v)
      v = v2;
  }
  return v;
}

void bt_assign_speed_limits(rndf_file *rndf, mdf_file *mdf, 
                            double max_speed_mph, double max_lateral_accel)
{
  int i, j, k, l, ei, id;
  rndf_waypoint *w;
  planner_data *data;
  double v;

  /* clear min and max speeds - set max to curvature speed */
  for(i = 0; i < rndf->num_segments(); i++) 
    for(j = 0; j < rndf->segment(i)->num_lanes(); j++)
      for(k = 0; k < rndf->segment(i)->lane(j)->num_waypoints(); k++) {
        w = rndf->segment(i)->lane(j)->waypoint(k);
        data = PDATA(w->data);
        for(l = 0; l < data->bt.num_waypoints(); l++) {
          data->bt.waypoint[l].min_v = 0;
          data->bt.waypoint[l].max_v = dgc_mph2ms(max_speed_mph);
          v = max_curvature_speed(data->bt.waypoint[l].true_k,
                                  max_lateral_accel);
          if(v < data->bt.waypoint[l].max_v)
            data->bt.waypoint[l].max_v = v;
        }
        for(ei = 0; ei < w->num_exits(); ei++) 
          for(l = 0; l < data->exit_bt[ei].num_waypoints(); l++) {
            data->exit_bt[ei].waypoint[l].min_v = 0;
            data->exit_bt[ei].waypoint[l].max_v = dgc_mph2ms(max_speed_mph);
            v = max_curvature_speed(data->exit_bt[ei].waypoint[l].true_k,
                                    max_lateral_accel);
            if(v < data->exit_bt[ei].waypoint[l].max_v)
              data->exit_bt[ei].waypoint[l].max_v = v;
          }
      }
          
  /* assign max and min velocities from MDF file */
  for(i = 0; i < mdf->num_speed_limits(); i++) {
    id = mdf->speed_limit(i)->id;
    if(id >= rndf->num_segments()) {
      fprintf(stderr, "Warning: skipping zone speed limit.\n");
      continue;
    }
    for(j = 0; j < rndf->segment(id)->num_lanes(); j++)
      for(k = 0; k < rndf->segment(id)->lane(j)->num_waypoints(); k++) {
        w = rndf->segment(id)->lane(j)->waypoint(k);
        data = PDATA(w->data);
        for(l = 0; l < data->bt.num_waypoints(); l++) {
          if(mdf->speed_limit(i)->max_speed != 0 &&
             mdf->speed_limit(i)->max_speed < data->bt.waypoint[l].max_v)
            data->bt.waypoint[l].max_v = mdf->speed_limit(i)->max_speed;
          if(mdf->speed_limit(i)->min_speed != 0)
            data->bt.waypoint[l].min_v = mdf->speed_limit(i)->min_speed;
          /* make sure that min velocity is <= max velocity */
          if(data->bt.waypoint[l].min_v > data->bt.waypoint[l].max_v)
            data->bt.waypoint[l].min_v = data->bt.waypoint[l].max_v;
        }
        for(ei = 0; ei < w->num_exits(); ei++) 
          for(l = 0; l < data->exit_bt[ei].num_waypoints(); l++) {
            if(mdf->speed_limit(i)->max_speed != 0 &&
               mdf->speed_limit(i)->max_speed < 
               data->exit_bt[ei].waypoint[l].max_v)
              data->exit_bt[ei].waypoint[l].max_v = 
                mdf->speed_limit(i)->max_speed;
            if(mdf->speed_limit(i)->min_speed != 0)
              data->exit_bt[ei].waypoint[l].min_v = 
                mdf->speed_limit(i)->min_speed;
            /* make sure that min velocity is <= max velocity */
            if(data->exit_bt[ei].waypoint[l].min_v > 
               data->exit_bt[ei].waypoint[l].max_v)
            data->exit_bt[ei].waypoint[l].min_v = 
              data->exit_bt[ei].waypoint[l].max_v;
          }
      }
  }
}

void bt_compute_lengths(rndf_file *rndf)
{
  int i, j, k, l, ei;
  rndf_waypoint *w;
  planner_data *data;
  double length;

  for(i = 0; i < rndf->num_segments(); i++) 
    for(j = 0; j < rndf->segment(i)->num_lanes(); j++)
      for(k = 0; k < rndf->segment(i)->lane(j)->num_waypoints(); k++) {
        w = rndf->segment(i)->lane(j)->waypoint(k);
        data = PDATA(w->data);
        
        length = 0;
        for(l = 1; l < data->bt.num_waypoints(); l++) 
          length += hypot(data->bt.waypoint[l].x - data->bt.waypoint[l - 1].x,
                          data->bt.waypoint[l].y - data->bt.waypoint[l - 1].y);
        data->bt.length = length;

        for(ei = 0; ei < w->num_exits(); ei++) {
          length = 0;
          for(l = 1; l < data->exit_bt[ei].num_waypoints(); l++) 
            length += hypot(data->exit_bt[ei].waypoint[l].x - 
                            data->exit_bt[ei].waypoint[l - 1].x,
                            data->exit_bt[ei].waypoint[l].y - 
                            data->exit_bt[ei].waypoint[l - 1].y);
          data->exit_bt[ei].length = length;
        }
      }
}

void add_base_trajectory(rndf_file *rndf, char *filename)
{
  dgc_FILE *fp;
  char line[1000], *err;
  int i, j, k, ei, num_exits, num_waypoints;
  int si, li, wi;
  rndf_waypoint *w;
  bt_waypoint wp;
  planner_data *data;
  int si2, li2, wi2;

  fp = dgc_fopen(filename, "r");
  if(fp == NULL)
    dgc_die("Error: could not open file %s for reading\n", filename);
  do {
    err = dgc_fgets(line, 1000, fp);
    if(err != NULL) {
      /* pick out the right waypoint */
      sscanf(line, "%d %d %d\n", &si, &li, &wi);
      si--; li--; wi--;
      w = rndf->segment(si)->lane(li)->waypoint(wi);
      if(w->data == NULL)
        dgc_die("Error: data class for RNDF node %d.%d.%d was NULL\n", si + 1,
                li + 1, wi + 1);
      data = PDATA(w->data);
      
      /* add the lane waypoints */
      dgc_fgets(line, 1000, fp);
      sscanf(line, "%d", &num_waypoints);
      for(i = 0; i < num_waypoints; i++) {
        dgc_fgets(line, 1000, fp);
        sscanf(line, "%lf %lf %f %f\n", &wp.x, &wp.y, &wp.theta, &wp.true_k);
        wp.is_stop = false;
        wp.max_v = 0;
        data->bt.waypoint.push_back(wp);
      }

      /* make sure we all agree on how many exits there should be */
      dgc_fgets(line, 1000, fp);
      sscanf(line, "%d", &num_exits);
      if(num_exits != w->num_exits())
        dgc_die("Error: incompatible RNDF and base trajectory\n");
      data->exit_bt.resize(w->num_exits());

      for(i = 0; i < num_exits; i++) {
        dgc_fgets(line, 1000, fp);
        sscanf(line, "%d %d %d", &si, &li, &wi);
        w->exit(i)->lookup_id(&si2, &li2, &wi2);
        if(si2 + 1 != si || li2 + 1 != li || wi2 + 1 != wi) {
          fprintf(stderr, "exit id %d.%d.%d\n", si, li, wi);
          fprintf(stderr, "actual id %d.%d.%d\n", si2 + 1, li2 + 1, wi2 + 1);
        }

        dgc_fgets(line, 1000, fp);
        sscanf(line, "%d", &num_waypoints);

        for(j = 0; j < num_waypoints; j++) {
          dgc_fgets(line, 1000, fp);
          sscanf(line, "%lf %lf %f %f\n", &wp.x, &wp.y, &wp.theta, 
                 &wp.true_k);
          wp.max_v = 0;
          wp.is_stop = false;
          data->exit_bt[i].waypoint.push_back(wp);
        }
      }
    }
  } while(err != NULL);
  dgc_fclose(fp);

  for(i = 0; i < rndf->num_segments(); i++)
    for(j = 0; j < rndf->segment(i)->num_lanes(); j++)
      for(k = 0; k < rndf->segment(i)->lane(j)->num_waypoints(); k++) {
        w = rndf->segment(i)->lane(j)->waypoint(k);
        if(w->stop()) {
          if(PDATA(w->data)->bt.num_waypoints() > 0)
            PDATA(w->data)->bt.waypoint[0].is_stop = true;
          for(ei = 0; ei < w->num_exits(); ei++)
            if(PDATA(w->data)->exit_bt[ei].num_waypoints() > 0)
              PDATA(w->data)->exit_bt[ei].waypoint[0].is_stop = true;
        }
      }

  //  bt_compute_lengths(rndf);
}


#ifdef blah
void bt_compute_curvature(rndf_file *rndf)
{
  int i, j, k, l, ei;
  double last_x = 0, last_y = 0, last_theta = 0;
  double first_x, first_y, first_theta;
  rndf_waypoint *w;
  planner_data *data;

  for(i = 0; i < rndf->num_segments(); i++)
    for(j = 0; j < rndf->segment(i)->num_lanes(); j++) {
      for(k = 0; k < rndf->segment(i)->lane(j)->num_waypoints(); k++) {
        w = rndf->segment(i)->lane(j)->waypoint(k);
        data = PDATA(w->data);

        /* compute curvature along lane */
        for(l = 0; l < data->bt.num_waypoints(); l++) {
          if(k != 0 || l != 0) {
            data->bt.waypoint[l].curvature = 
              dgc_normalize_theta(data->bt.waypoint[l].theta - last_theta) /
              hypot(data->bt.waypoint[l].x - last_x, 
                    data->bt.waypoint[l].y - last_y);
          }
          else
            data->bt.waypoint[l].curvature = 0;
          if(k < rndf->segment(i)->lane(j)->num_waypoints() - 1) {
            last_x = data->bt.waypoint[l].x;
            last_y = data->bt.waypoint[l].y;
            last_theta = data->bt.waypoint[l].theta;
          }
        }
        if(k == 0) {
          if(data->bt.num_waypoints() > 1)
            data->bt.waypoint[0].curvature = data->bt.waypoint[1].curvature;
        }

        first_x = last_x;
        first_y = last_y;
        first_theta = last_theta;

        /* compute curvature along exits */
        for(ei = 0; ei < w->num_exits(); ei++) {
          last_x = first_x;
          last_y = first_y;
          last_theta = first_theta;
          for(l = 0; l < data->exit_bt[ei].num_waypoints(); l++) {
            double d = hypot(data->exit_bt[ei].waypoint[l].x - last_x,
                      data->exit_bt[ei].waypoint[l].y - last_y);
            if(d < 0.05) {
              fprintf(stderr, "found d < 0.05 %f\n", d);
              fprintf(stderr, "at %d %d %d of %d l = %d\n", 
                      i + 1, j + 1, k + 1, 
                      rndf->segment(i)->lane(j)->num_waypoints(), l);
            }
            data->exit_bt[ei].waypoint[l].curvature = 
              dgc_normalize_theta(data->exit_bt[ei].waypoint[l].theta - 
                                  last_theta) /
              hypot(data->exit_bt[ei].waypoint[l].x - last_x,
                    data->exit_bt[ei].waypoint[l].y - last_y);
            last_x = data->exit_bt[ei].waypoint[l].x;
            last_y = data->exit_bt[ei].waypoint[l].y;
            last_theta = data->exit_bt[ei].waypoint[l].theta;
          }
        }

      }
    }

  /* assign velocities based on curvature */
  for(i = 0; i < rndf->num_segments(); i++)
    for(j = 0; j < rndf->segment(i)->num_lanes(); j++)
      for(k = 0; k < rndf->segment(i)->lane(j)->num_waypoints(); k++) {
        w = rndf->segment(i)->lane(j)->waypoint(k);
        data = PDATA(w->data);

        for(l = 0; l < data->bt.num_waypoints(); l++) 
          data->bt.waypoint[l].max_v =
            max_curvature_speed(data->bt.waypoint[l].curvature, 
                                ACCEL_PARAMETER);
        for(ei = 0; ei < w->num_exits(); ei++)
          for(l = 0; l < data->exit_bt[ei].num_waypoints(); l++)
            data->exit_bt[ei].waypoint[l].max_v =
              max_curvature_speed(data->exit_bt[ei].waypoint[l].curvature,
                                  ACCEL_PARAMETER);
      }
}
#endif


