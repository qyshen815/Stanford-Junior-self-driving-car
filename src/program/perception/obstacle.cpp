/*
 *  Created on: Jul 24, 2009
 *      Author: duhadway
 */

#include "obstacle.h"
#include "utils.h"
#include <grid.h>
#include <iostream>

using namespace std;
using namespace Eigen;

//#define FILL_DEBUG
//#define FILL_DEBUG_2

void points_in_cell(dgc_perception_map_cell_p cell, std::vector<point3d_t>& points);

extern dgc_perception_map_cells_p       obstacles_s;

dgc_pose_t zero_pose = {0,0,0,0,0,0};

namespace dgc {

Obstacle::Obstacle(int id) :
  x_center_(0),
  y_center_(0),
  z_center_(0),
  lane_(NULL),
  rndfDist_(-1.0),
  matched_(false),
  id(id),
  pose(zero_pose),
  length(0.0),
  width(0.0),
  type(OBSTACLE_UNKNOWN),
  type_this_frame_(OBSTACLE_UNKNOWN),
  classified_this_frame_(false),
  response_(VectorXf::Zero(getClassNames().size()))
{
  robot_pose_when_observed_.x = -1;
  robot_pose_when_observed_.y = -1;
  robot_pose_when_observed_.z = -1;
}

Obstacle::Obstacle(const Obstacle& o) :
  x_center_(o.x_center_),
  y_center_(o.y_center_),
  z_center_(o.z_center_),
  lane_(o.lane_),
  rndfDist_(o.rndfDist_),
  matched_(o.matched_),
  id (o.id),
  pose(o.pose),
  length(o.length),
  width(o.width),
  type(o.type),
  type_this_frame_(o.type_this_frame_),
  classified_this_frame_(o.classified_this_frame_),
  robot_pose_when_observed_(o.robot_pose_when_observed_),
  response_(o.response_)
  // import not to copy points_ here
{


}

Obstacle::~Obstacle() {

}

void draw_line(dgc_grid_p grid, int x1, int y1, int x2, int y2, int value, bool* edge) {
  // bresenham
  int X1, Y1;
  int X2, Y2;
  int increment;
  int usingYindex;
  int deltaX, deltaY;
  int dTerm;
  int incrE, incrNE;
  int XIndex, YIndex;
  int flipped;

  usingYindex = 0;

  if(fabs((double)(y2 - y1) / (double)(x2 - x1)) > 1)
    (usingYindex)++;

  if(usingYindex) {
    Y1 = x1;
    X1 = y1;
    Y2 = x2;
    X2 = y2;
  }
  else {
    X1 = x1;
    Y1 = y1;
    X2 = x2;
    Y2 = y2;
  }

  if((x2 - x1) * (y2 - y1) < 0) {
    flipped = 1;
    Y1 = -Y1;
    Y2 = -Y2;
  }
  else
    flipped = 0;

  if(X2 > X1)
    increment = 1;
  else
    increment = -1;

  deltaX = X2-X1;
  deltaY = Y2-Y1;

  incrE = 2 * deltaY * increment;
  incrNE = 2 * (deltaY - deltaX) * increment;
  dTerm = (2 * deltaY - deltaX) * increment;

  XIndex = X1;
  YIndex = Y1;

  while (XIndex != X2) {

    XIndex += increment;
    if(dTerm < 0 || (increment < 0 && dTerm <= 0))
      dTerm += incrE;
    else {
      dTerm += incrNE;
      YIndex += increment;
    }

    int x, y;
    if(usingYindex) {
      y = XIndex;
      x = YIndex;
      if(flipped)
        x = -x;
    }
    else {
      x = XIndex;
      y = YIndex;
      if(flipped)
        y = -y;
    }

    dgc_perception_map_cell_p cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local(grid, x, y);
    if (cell) {
      cell->last_dynamic = value;
#ifdef FILL_DEBUG
      cell->min = 2.0; cell->max = 3.0;
      obstacles_s->cell[obstacles_s->num] = cell;
      if (obstacles_s->num<MAX_NUM_POINTS) {
        obstacles_s->num++;
      }
#endif
    } else {
      *edge = true;
    }
  }
}

// works for convex objects that don't cross grid boundaries (which can happen if grid is recentered)
void fast_fill_boundary(dgc_grid_p grid, int start_r, int start_c, unsigned short fill) {
  dgc_perception_map_cell_p cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local(grid, start_r, start_c);


  int bytes_per_cell = grid->bytes_per_cell;
  int bytes_per_row = bytes_per_cell * grid->cols;

  printf("(r0, c0): (%d %d)\n", grid->array_r0, grid->array_c0);

  char* last_dynamic = (char*)&cell->last_dynamic;
  char* right_dynamic = last_dynamic + bytes_per_cell;
  char* top = 0;
  char* bottom = 0;

  // fill left half of row
  while (*((unsigned short*)last_dynamic) < fill) {
    *((unsigned short*)last_dynamic) = fill;
    last_dynamic -= bytes_per_cell;

    if (!top) {
      char* tmp = last_dynamic + bytes_per_row;
      if (*(unsigned short*)tmp < fill)
        top = tmp;
    }

    if (!bottom) {
      char* tmp = last_dynamic - bytes_per_row;
      if (*(unsigned short*)tmp < fill)
        bottom = tmp;
    }

    if (top && bottom) {
      while (*((unsigned short*)last_dynamic) < fill) {
        *((unsigned short*)last_dynamic) = fill;
        last_dynamic -= bytes_per_cell;
      }
      break;
    }
  }

  // fill right half of row
  last_dynamic = right_dynamic;
  while (*((unsigned short*)last_dynamic) < fill) {
    *((unsigned short*)last_dynamic) = fill;
    last_dynamic += bytes_per_cell;

    if (!top) {
      char* tmp = last_dynamic + bytes_per_row;
      if (*(unsigned short*)tmp < fill)
        top = tmp;
    }

    if (!bottom) {
      char* tmp = last_dynamic - bytes_per_row;
      if (*(unsigned short*)tmp < fill)
        bottom = tmp;
    }

    if (top && bottom) {
      while (*((unsigned short*)last_dynamic) < fill) {
        *((unsigned short*)last_dynamic) = fill;
        last_dynamic += bytes_per_cell;
      }
      break;
    }
  }


  // fill top half
  while (top) {
    last_dynamic = top;
    right_dynamic = last_dynamic + bytes_per_cell;

    top = 0;

    // fill left half of row
    while (*((unsigned short*)last_dynamic) < fill) {
      *((unsigned short*)last_dynamic) = fill;

      if (!top) {
        char* tmp = last_dynamic + bytes_per_row;
        if (*(unsigned short*)tmp < fill)
          top = tmp;
        last_dynamic -= bytes_per_cell;
      } else {
        last_dynamic -= bytes_per_cell;
        while (*((unsigned short*)last_dynamic) < fill) {
          *((unsigned short*)last_dynamic) = fill;
          last_dynamic -= bytes_per_cell;
        }
        break;
      }
    }

    // fill right half of row
    last_dynamic = right_dynamic;
    while (*((unsigned short*)last_dynamic) < fill) {
      *((unsigned short*)last_dynamic) = fill;
      if (!top) {
        char* tmp = last_dynamic + bytes_per_row;
        if (*(unsigned short*)tmp < fill)
          top = tmp;
        last_dynamic += bytes_per_cell;
      } else {
        last_dynamic += bytes_per_cell;
        while (*((unsigned short*)last_dynamic) < fill) {
          *((unsigned short*)last_dynamic) = fill;
          last_dynamic += bytes_per_cell;
        }
        break;
      }
    }
  }

  // fill bottom half
  while (bottom) {
    last_dynamic = bottom;
    right_dynamic = last_dynamic + bytes_per_cell;

    bottom = 0;

    // fill left half of row
    while (*((unsigned short*)last_dynamic) < fill) {
      *((unsigned short*)last_dynamic) = fill;

      if (!bottom) {
        char* tmp = last_dynamic - bytes_per_row;
        if (*(unsigned short*)tmp < fill)
          bottom = tmp;
        last_dynamic -= bytes_per_cell;
      } else {
        last_dynamic -= bytes_per_cell;
        while (*((unsigned short*)last_dynamic) < fill) {
          *((unsigned short*)last_dynamic) = fill;
          last_dynamic -= bytes_per_cell;
        }
        break;
      }
    }

    // fill right half of row
    last_dynamic = right_dynamic;
    while (*((unsigned short*)last_dynamic) < fill) {
      *((unsigned short*)last_dynamic) = fill;
      if (!bottom) {
        char* tmp = last_dynamic - bytes_per_row;
        if (*(unsigned short*)tmp < fill)
          bottom = tmp;
        last_dynamic += bytes_per_cell;
      } else {
        last_dynamic += bytes_per_cell;
        while (*((unsigned short*)last_dynamic) < fill) {
          *((unsigned short*)last_dynamic) = fill;
          last_dynamic += bytes_per_cell;
        }
        break;
      }
    }
  }
}

void fill_boundary(dgc_grid_p grid, int start_r, int start_c, int fill) {
  // fill first row
#ifdef FILL_DEBUG_2
  int count = 0;
#endif
  int r = start_r;
  int c = start_c;
  int top_c;
  int bottom_c;
  bool found_top_c = false;
  bool found_bottom_c = false;

  dgc_perception_map_cell_p neighbor_cell;
  dgc_perception_map_cell_p cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local(grid, r, c);

  // fill left
  while ((cell) && (cell->last_dynamic < fill)) {
    cell->last_dynamic = fill;
#ifdef FILL_DEBUG_2
    cell->min = 2.0; cell->max = 3.0;
    obstacles_s->cell[obstacles_s->num] = cell;
    if (obstacles_s->num<MAX_NUM_POINTS) {
      obstacles_s->num++;
    }
    count++;
#endif

    if (!found_top_c) {
      neighbor_cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local(grid, r+1, c);
      if ((neighbor_cell) && (neighbor_cell->last_dynamic < fill)) {
        found_top_c = true;
        top_c = c;
      }
    }

    if (!found_bottom_c) {
      neighbor_cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local(grid, r-1, c);
      if ((neighbor_cell) && (neighbor_cell->last_dynamic < fill)) {
        found_bottom_c = true;
        bottom_c = c;
      }
    }

    c--;
    cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local(grid, r, c);
  }

  // fill right
  c = start_c + 1;
  cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local(grid, r, c);
  while ((cell) && (cell->last_dynamic < fill)) {
    cell->last_dynamic = fill;
#ifdef FILL_DEBUG_2
    cell->min = 2.0; cell->max = 3.0;
    obstacles_s->cell[obstacles_s->num] = cell;
    if (obstacles_s->num<MAX_NUM_POINTS) {
      obstacles_s->num++;
    }
    count++;
#endif

    if (!found_top_c) {
      neighbor_cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local(grid, r+1, c);
      if ((neighbor_cell) && (neighbor_cell->last_dynamic < fill)) {
        found_top_c = true;
        top_c = c;
      }
    }

    if (!found_bottom_c) {
      neighbor_cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local(grid, r-1, c);
      if ((neighbor_cell) && (neighbor_cell->last_dynamic < fill)) {
        found_bottom_c = true;
        bottom_c = c;
      }
    }

    c++;
    cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local(grid, r, c);
  }


  // fill up
  while (found_top_c) {
    found_top_c = false;
    c = top_c;
    int right_c = top_c + 1;
    r++;
    cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local(grid, r, c);

    // fill left
    while ((cell) && (cell->last_dynamic < fill)) {
      cell->last_dynamic = fill;
#ifdef FILL_DEBUG_2
      cell->min = 2.0; cell->max = 3.0;
      obstacles_s->cell[obstacles_s->num] = cell;
      if (obstacles_s->num<MAX_NUM_POINTS) {
        obstacles_s->num++;
      }
      count++;
#endif

      if (!found_top_c) {
        neighbor_cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local(grid, r+1, c);
        if ((neighbor_cell) && (neighbor_cell->last_dynamic < fill)) {
          found_top_c = true;
          top_c = c;
        }
      }

      c--;
      cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local(grid, r, c);
    }

    // fill right
    c = right_c;
    cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local(grid, r, c);
    while ((cell) && (cell->last_dynamic < fill)) {
      cell->last_dynamic = fill;
#ifdef FILL_DEBUG_2
      cell->min = 2.0; cell->max = 3.0;
      obstacles_s->cell[obstacles_s->num] = cell;
      if (obstacles_s->num<MAX_NUM_POINTS) {
        obstacles_s->num++;
      }
      count++;
#endif

      if (!found_top_c) {
        neighbor_cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local(grid, r+1, c);
        if ((neighbor_cell) && (neighbor_cell->last_dynamic < fill)) {
          found_top_c = true;
          top_c = c;
        }
      }

      c++;
      cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local(grid, r, c);
    }
  }

  // fill down
  r = start_r;
  while (found_bottom_c) {
    found_bottom_c = false;
    c = bottom_c;
    int right_c = bottom_c + 1;
    r--;
    cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local(grid, r, c);

    // fill left
    while ((cell) && (cell->last_dynamic < fill)) {
      cell->last_dynamic = fill;
#ifdef FILL_DEBUG_2
      cell->min = 2.0; cell->max = 3.0;
      obstacles_s->cell[obstacles_s->num] = cell;
      if (obstacles_s->num<MAX_NUM_POINTS) {
        obstacles_s->num++;
      }
      count++;
#endif

      if (!found_bottom_c) {
        neighbor_cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local(grid, r-1, c);
        if ((neighbor_cell) && (neighbor_cell->last_dynamic < fill)) {
          found_bottom_c = true;
          bottom_c = c;
        }
      }

      c--;
      cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local(grid, r, c);
    }

    // fill right
    c = right_c;
    cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local(grid, r, c);
    while ((cell) && (cell->last_dynamic < fill)) {
      cell->last_dynamic = fill;
#ifdef FILL_DEBUG_2
      cell->min = 2.0; cell->max = 3.0;
      obstacles_s->cell[obstacles_s->num] = cell;
      if (obstacles_s->num<MAX_NUM_POINTS) {
        obstacles_s->num++;
      }
      count++;
#endif

      if (!found_bottom_c) {
        neighbor_cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local(grid, r-1, c);
        if ((neighbor_cell) && (neighbor_cell->last_dynamic < fill)) {
          found_bottom_c = true;
          bottom_c = c;
        }
      }

      c++;
      cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local(grid, r, c);
    }
  }
}

void fill_boundary_unsafe(dgc_grid_p grid, int start_r, int start_c, int fill) {
  // fill first row
#ifdef FILL_DEBUG_2
  int count = 0;
#endif
  int r = start_r;
  int c = start_c;
  int top_c;
  int bottom_c;
  bool found_top_c = false;
  bool found_bottom_c = false;

  dgc_perception_map_cell_p neighbor_cell;
  dgc_perception_map_cell_p cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local_unsafe(grid, r, c);

  // fill left
  while (cell->last_dynamic < fill) {
    cell->last_dynamic = fill;
#ifdef FILL_DEBUG_2
    cell->min = 2.0; cell->max = 3.0;
    obstacles_s->cell[obstacles_s->num] = cell;
    if (obstacles_s->num<MAX_NUM_POINTS) {
      obstacles_s->num++;
    }
    count++;
#endif

    if (!found_top_c) {
      neighbor_cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local_unsafe(grid, r+1, c);
      if (neighbor_cell->last_dynamic < fill) {
        found_top_c = true;
        top_c = c;
      }
    }

    if (!found_bottom_c) {
      neighbor_cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local_unsafe(grid, r-1, c);
      if (neighbor_cell->last_dynamic < fill) {
        found_bottom_c = true;
        bottom_c = c;
      }
    }

    c--;
    cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local_unsafe(grid, r, c);
  }

  // fill right
  c = start_c + 1;
  cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local_unsafe(grid, r, c);
  while (cell->last_dynamic < fill) {
    cell->last_dynamic = fill;
#ifdef FILL_DEBUG_2
    cell->min = 2.0; cell->max = 3.0;
    obstacles_s->cell[obstacles_s->num] = cell;
    if (obstacles_s->num<MAX_NUM_POINTS) {
      obstacles_s->num++;
    }
    count++;
#endif

    if (!found_top_c) {
      neighbor_cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local_unsafe(grid, r+1, c);
      if (neighbor_cell->last_dynamic < fill) {
        found_top_c = true;
        top_c = c;
      }
    }

    if (!found_bottom_c) {
      neighbor_cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local_unsafe(grid, r-1, c);
      if (neighbor_cell->last_dynamic < fill) {
        found_bottom_c = true;
        bottom_c = c;
      }
    }

    c++;
    cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local_unsafe(grid, r, c);
  }


  // fill up
  while (found_top_c) {
    found_top_c = false;
    c = top_c;
    int right_c = top_c + 1;
    r++;
    cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local_unsafe(grid, r, c);

    // fill left
    while (cell->last_dynamic < fill) {
      cell->last_dynamic = fill;
#ifdef FILL_DEBUG_2
      cell->min = 2.0; cell->max = 3.0;
      obstacles_s->cell[obstacles_s->num] = cell;
      if (obstacles_s->num<MAX_NUM_POINTS) {
        obstacles_s->num++;
      }
      count++;
#endif

      if (!found_top_c) {
        neighbor_cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local_unsafe(grid, r+1, c);
        if (neighbor_cell->last_dynamic < fill) {
          found_top_c = true;
          top_c = c;
        }
      }

      c--;
      cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local_unsafe(grid, r, c);
    }

    // fill right
    c = right_c;
    cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local_unsafe(grid, r, c);
    while (cell->last_dynamic < fill) {
      cell->last_dynamic = fill;
#ifdef FILL_DEBUG_2
      cell->min = 2.0; cell->max = 3.0;
      obstacles_s->cell[obstacles_s->num] = cell;
      if (obstacles_s->num<MAX_NUM_POINTS) {
        obstacles_s->num++;
      }
      count++;
#endif

      if (!found_top_c) {
        neighbor_cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local_unsafe(grid, r+1, c);
        if (neighbor_cell->last_dynamic < fill) {
          found_top_c = true;
          top_c = c;
        }
      }

      c++;
      cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local_unsafe(grid, r, c);
    }
  }

  // fill down
  r = start_r;
  while (found_bottom_c) {
    found_bottom_c = false;
    c = bottom_c;
    int right_c = bottom_c + 1;
    r--;
    cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local_unsafe(grid, r, c);

    // fill left
    while (cell->last_dynamic < fill) {
      cell->last_dynamic = fill;
#ifdef FILL_DEBUG_2
      cell->min = 2.0; cell->max = 3.0;
      obstacles_s->cell[obstacles_s->num] = cell;
      if (obstacles_s->num<MAX_NUM_POINTS) {
        obstacles_s->num++;
      }
      count++;
#endif

      if (!found_bottom_c) {
        neighbor_cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local_unsafe(grid, r-1, c);
        if (neighbor_cell->last_dynamic < fill) {
          found_bottom_c = true;
          bottom_c = c;
        }
      }

      c--;
      cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local_unsafe(grid, r, c);
    }

    // fill right
    c = right_c;
    cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local_unsafe(grid, r, c);
    while (cell->last_dynamic < fill) {
      cell->last_dynamic = fill;
#ifdef FILL_DEBUG_2
      cell->min = 2.0; cell->max = 3.0;
      obstacles_s->cell[obstacles_s->num] = cell;
      if (obstacles_s->num<MAX_NUM_POINTS) {
        obstacles_s->num++;
      }
      count++;
#endif

      if (!found_bottom_c) {
        neighbor_cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local_unsafe(grid, r-1, c);
        if (neighbor_cell->last_dynamic < fill) {
          found_bottom_c = true;
          bottom_c = c;
        }
      }

      c++;
      cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local_unsafe(grid, r, c);
    }
  }
}

void Obstacle::markDynamic(dgc_grid_p grid, dgc_perception_map_cells_p obstacles_s, unsigned short counter, double velocity) {
  dgc_transform_t t;
  dgc_transform_identity(t);

  dgc_transform_rotate_z(t, pose.yaw);

  double buffer = 0.5;
  double lookahead = 0.0;
  if (velocity > 1.0) {
    lookahead = std::max(velocity, 10.0) * 0.5; // assume this is not going to hit anything in the next half second
  }

  double unused_z = 0;
  double w = (length/2.0) + buffer;
  double w1 = w +  lookahead;
  double l1 = (width/2.0)  + buffer;
  double w2 = w + lookahead;
  double l2 = -l1;
  double w3 = -w;
  double l3 = -l1;
  double w4 = -w;
  double l4 = l1;

  dgc_transform_point(&w1, &l1, &unused_z, t);
  dgc_transform_point(&w2, &l2, &unused_z, t);
  dgc_transform_point(&w3, &l3, &unused_z, t);
  dgc_transform_point(&w4, &l4, &unused_z, t);

  double x1 = pose.x + w1;
  double y1 = pose.y + l1;

  double x2 = pose.x + w2;
  double y2 = pose.y + l2;

  double x3 = pose.x + w3;
  double y3 = pose.y + l3;

  double x4 = pose.x + w4;
  double y4 = pose.y + l4;

  int r, c, r1, c1, r2, c2, r3, c3, r4, c4;
  dgc_grid_xy_to_rc_local(grid, pose.x, pose.y, &r, &c);
  dgc_grid_xy_to_rc_local(grid, x1, y1, &r1, &c1);
  dgc_grid_xy_to_rc_local(grid, x2, y2, &r2, &c2);
  dgc_grid_xy_to_rc_local(grid, x3, y3, &r3, &c3);
  dgc_grid_xy_to_rc_local(grid, x4, y4, &r4, &c4);

  bool edge = false;
  draw_line(grid, r1, c1, r2, c2, counter, &edge);
  draw_line(grid, r2, c2, r3, c3, counter, &edge);
  draw_line(grid, r3, c3, r4, c4, counter, &edge);
  draw_line(grid, r4, c4, r1, c1, counter, &edge);

  int min_r = r1;
  int min_c = c1;

  if (r2 < min_r) {
    min_r = r2;
    min_c = c2;
  }

  if (r3 < min_r) {
    min_r = r3;
    min_c = c3;
  }

  if (r4 < min_r) {
    min_r = r4;
    min_c = c4;
  }

  if (edge)
    fill_boundary(grid, r, c, counter);
  else
    fill_boundary_unsafe(grid, r, c, counter);
//  int max_r = std::max(r1, std::max(r2, std::max(r3,r4)));
//  fill_boundary(grid, min_r, max_r, min_c, counter, counter + 1);
//  fast_fill_boundary(grid, r, c, counter);
}

bool Obstacle::getCenterOfPoints(double *x, double *y)
{
  if (points_.size() == 0) {
    populatePoints();
  }

  if(x_center_ != 0.0 || y_center_ != 0.0) {
    *x = x_center_;
    *y = y_center_;
    return true;
  }

  *x = 0;
  *y = 0;
  int count = points_.size();
  if (count < 1)
    return false;

  for (int i=0; i<count; i++) {
    *x = *x + points_[i].x;
    *y = *y + points_[i].y;
  }

  *x = *x / (double)count;
  *y = *y / (double)count;
  x_center_ = *x;
  y_center_ = *y;
  return true;
}

bool Obstacle::getCenterOfPoints(double *x, double *y, double *z)
{
  if (points_.size() == 0) {
    populatePoints();
  }

  if(x_center_ != 0.0 || y_center_ != 0.0 || z_center_ != 0.0) {
    *x = x_center_;
    *y = y_center_;
    *z = z_center_;
  }

  *x = 0;
  *y = 0;
  *z = 0;
  int count = points_.size();
  if (count < 1)
    return false;

  for (int i=0; i<count; i++) {
    *x = *x + points_[i].x;
    *y = *y + points_[i].y;
    *z = *z + points_[i].z;
  }

  *x = *x / (double)count;
  *y = *y / (double)count;
  *z = *z / (double)count;
  x_center_ = *x;
  y_center_ = *y;
  z_center_ = *z;
  return true;
}

void Obstacle::merge(const Obstacle& o) {
  pose.x = (pose.x + o.pose.x) / 2.0;
  pose.y = (pose.y + o.pose.y) / 2.0;
  pose.z = (pose.z + o.pose.z) / 2.0;
  x_center_ = 0.0;
  y_center_ = 0.0;
  z_center_ = 0.0;

  if ((points_.size() > 0) && (o.points_.size() > 0)) {
    points_.insert(points_.end(), o.points_.begin(), o.points_.end());
  } else {
    points_.clear();
  }
}

void Obstacle::populatePoints() {
  points_.clear();
  point3d_t pt;
  pt.x = pose.x;
  pt.y = pose.y;
  pt.z = pose.z;
  points_.push_back(pt);
}

std::vector<point3d_t>& Obstacle::getPoints() {
  if (points_.size() == 0) {
    populatePoints();
  }

  return points_;
}
  
GridObstacle::GridObstacle(int id, dgc_grid_p grid) :
  Obstacle(id),
  grid_(grid) {
}

GridObstacle::GridObstacle (const GridObstacle& o) :
  Obstacle(o),
  grid_(o.grid_),
  cells_(o.cells_) {
}

GridObstacle::~GridObstacle() {

}

int GridObstacle::getSize() {
  return cells_.size();
}

void GridObstacle::clear() {
  cells_.clear();
}

//void GridObstacle::markDynamic(dgc_grid_p grid, dgc_perception_map_cells_p obstacles, unsigned short counter)  {
//  for (unsigned int i=0; i<cells_.size(); i++) {
//    cells_[i]->last_dynamic = counter;
//  }
//}

void GridObstacle::merge(const GridObstacle& o) {
  Obstacle::merge(o);
  cells_.insert(cells_.end(), o.cells_.begin(), o.cells_.end());
}

void GridObstacle::addCell(dgc_perception_map_cell_p cell) {
  cells_.push_back(cell);
}

std::vector<dgc_perception_map_cell_p>& GridObstacle::getCells() {
  return cells_;
}

void GridObstacle::populatePoints() {
  points_.clear();

  point3d_t pt;
  pt.z = 0;
  for (unsigned int i=0; i < cells_.size(); i++) {
    points_in_cell(cells_[i], points_);

//    cell_to_coord(grid_, cells_[i], &pt.x, &pt.y);
//    points_.push_back(pt);
  }
}

float GridObstacle::maxHeight() {
  float max_height = -1.0;

  for (unsigned int i=0; i < cells_.size(); i++) {
    max_height = std::max(max_height, cell_height(cells_[i]));
  }
  return max_height;
}


LaserObstacle::LaserObstacle(int id) : Obstacle(id) {

}

LaserObstacle::LaserObstacle (const LaserObstacle& o) :
    Obstacle(o),
    min_z_(1e9),
    max_z_(-1e9)
{
  points_ = o.points_;
}

LaserObstacle::~LaserObstacle() {

}

int LaserObstacle::getSize() {
  return points_.size();
}

void LaserObstacle::clear() {
  points_.clear();
}

void LaserObstacle::merge(const LaserObstacle& o) {
  Obstacle::merge(o);
}

//void LaserObstacle::markDynamic(dgc_grid_p grid, dgc_perception_map_cells_p obstacles, unsigned short counter) {
//  std::vector<point3d_t>& points = getPoints();
//  for (unsigned int i=0; i < points.size(); i++) {
//    dgc_perception_map_cell_p cell = (dgc_perception_map_cell_p)grid_get_xy(grid, points[i].x, points[i].y);
//    if (cell != NULL) {
//      cell->last_dynamic = counter;
//    }
//  }
//}

void LaserObstacle::populatePoints() {

}

void LaserObstacle::addPoint(laser_point_p pt) {
  point3d_t point;
  point.x = pt->point->x * CM_TO_METER_FACTOR + pt->scan->robot.x;
  point.y = pt->point->y * CM_TO_METER_FACTOR + pt->scan->robot.y;
  point.z = pt->point->z * CM_TO_METER_FACTOR + pt->scan->robot.z;
  point.intensity = pt->point->intensity;

  max_z_ = std::max(max_z_, point.z);
  min_z_ = std::min(min_z_, point.z);

  points_.push_back(point);
}

float LaserObstacle::maxHeight() {
  return (max_z_ - min_z_);
}

void LaserObstacle::reserve(int count) {
  points_.reserve(count);
}

RadarObservation::RadarObservation() {

}

RadarObservation::~RadarObservation() {

}


}

