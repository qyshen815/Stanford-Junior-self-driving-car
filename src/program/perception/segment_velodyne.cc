#include "perception.h"
#include <assert.h>

#define  NUM_BEAMS       64
#define  MAX_SEGMENTS_PER_BEAM 1024
using namespace std;

typedef struct dgc_perception_line_segment_t {
  int object_id;
  int num_noise_points;
  int is_ground;
  double last_range;
  double min_angle, max_angle;
  double mean_range, mean_z, mean_dist;
  vector<dgc_velodyne_point_p>  points;

  // for union-find
  dgc_perception_line_segment_t* parent;
  int rank;
} *dgc_perception_line_segment_p;

dgc_perception_line_segment_p segments[NUM_BEAMS];
int num_segments[NUM_BEAMS];

void perception_draw_segmentation() {
  static unsigned char colors[19][3] = {
    {255,0,0},
    {0,255,0},
    {0,0,255},
    {255,255,0},
    {0,255,255},
    {255,0,255},
    {239,230,0},
    {230,0,230},
    {0,230,230},
    {230,0,0},
    {128,51,128},
    {51,128,128},
    {255,51,51},
    {51,255,51},
    {51,51,255},
    {51,179,204},
    {128,255,51},
    {255,128,51},
    {51,128,255}
  };

  /* setup GLS header */
  gls->coordinates = GLS_LOCAL_COORDINATES;
  gls->origin_x = 0;
  gls->origin_y = 0;
  gls->origin_z = 0; //applanix_current_pose()->smooth_z;
  glsColor3f( gls, 1.0, 0.0, 0.0 );

  glsPushMatrix(gls);
//  glsScalef(gls, CM_TO_METER_FACTOR, CM_TO_METER_FACTOR, CM_TO_METER_FACTOR);
//  glsTranslatef(gls, , , CM_TO_METER_FACTOR);
  glsBegin(gls, GLS_POINTS);
  for (int l=0; l < NUM_BEAMS; l++) {
    for (int i = 0; i < lscan[l].num_points; i++) {
      if ( lscan[l].laser_point[i].valid) {
        if (lscan[l].laser_point[i].obstacle) //{
//
//        } else {
          glsColor3f( gls, 1.0, 0.0, 0.0 );
        else
          glsColor3f( gls, 0.7, 0.7, 0.7 );

          glsVertex3f(gls,
              lscan[l].laser_point[i].point->x * .01 + lscan[l].laser_point[i].scan->robot.x,
              lscan[l].laser_point[i].point->y * .01 + lscan[l].laser_point[i].scan->robot.y,
              lscan[l].laser_point[i].point->z * .01);
//        }
      }
    }
  }
  glsEnd(gls);

  double x1,y1,x2,y2;
  glsColor3f(gls, 0.7, 0.7, 0.7);
  glsLineWidth(gls, 1.0);
  glsBegin(gls, GLS_LINES);
  grid_rc_to_xy(grid, 0, 0, &x1, &y1);
  grid_rc_to_xy(grid, grid->rows, grid->cols, &x2, &y2);
  glsVertex3f(gls, x1, y1, 0);
  glsVertex3f(gls, x1, y2, 0);

  glsVertex3f(gls, x1, y2, 0);
  glsVertex3f(gls, x2, y2, 0);

  glsVertex3f(gls, x2, y2, 0);
  glsVertex3f(gls, x2, y1, 0);

  glsVertex3f(gls, x2, y1, 0);
  glsVertex3f(gls, x1, y1, 0);
  glsEnd(gls);

  glsPopMatrix(gls);
}


void
perception_segment_frame(ObstacleList* obstacles) {
  obstacles->obstacles.clear();

//  for (int l=0; l < NUM_BEAMS; l++) {
//    for (int i = 0; i < lscan[l].num_points; i++) {
//      if (lscan[l].laser_point[i].valid) {
//        lscan[l].laser_point[i].point->
//      }
//    }
//  }


  if (settings.segmentation_settings.gls_output) {
    perception_draw_segmentation();
  }

  // collect obstacles
//  perception_collect_obstacles(regions, obstacles);

//  printf("segmented obstacles: %d\n", obstacles->obstacles.size());
}

void
segment_velodyne( dgc_velodyne_data_p v ) {
  for (int i = 0; i < v->num_scans; i++) {
    segement_scan(&v->scans[i]);
  }
}

// the points come in order, but there are sometimes large sections missing
void segment_scan(dgc_velodyne_scan_p scan)
{
  dgc_perception_line_segment_p current_segment;

//  if(DEBUG) printf("scan_num_points: %d\n", scan_num_points);

  dgc_velodyne_point_p point = scan->p;
  int block = scan->block * 32;
  for (int i = 0; i < 32; i++) {
    int l = i + block;
    if (point->range < 200) continue;
    double delta_range = 1.0;
    double delta_angle = 0.01;
    int num_segment = num_segments[l];
    if (num_segment >= max_segments_per_laser) {
      printf("Warning: maxed out maximum segments to track!\n");
      continue;
    }

    current_segment = &segments[l][num_segment];
    int num_points = current_segment->points.size();

    if (num_points == 0) {
      current_segment->last_range = point->range;
      current_segment->min_angle = current_segment->max_angle = point->angle;
    }
    double delta_range = abs(point->range - current_segment->last_range);
    double delta_angle = abs(point->angle - current_segment->min_angle);

    // is this scan finished?
    if ( (delta_range > max_delta_range) ||
         (delta_angle > MAX_DELTA_ANGLE)) {
      if ( (num_points >= min_segment_points) &&
           (num_segments[l] < MAX_SEGMENTS_PER_BEAM)) {
        num_segments[l]++;
//        current_segment++;
        current_segment = &segments[l][num_segment+1];
      }
      clear_segment(current_segment);
    }
    // add current point to segment
    else if ((current_point->label != LABEL_NOISE) && (current_point->label != LABEL_OUTSIDE_RNDF)) {
      current_segment->points.push_back(current_point);
      current_segment->last_range = current_point->range;
    }
  }
}
