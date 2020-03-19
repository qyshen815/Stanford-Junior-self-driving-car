/*
 *  Created on: Jul 24, 2009
 *      Author: duhadway
 */

#include "segment.h"
#include "utils.h"
#include "velodyne.h"
#include <tr1/memory>

using namespace std;
using namespace std::tr1;

namespace dgc {

Segmenter::Segmenter() {

}

Segmenter::~Segmenter() {

}

GridSegmenter::GridSegmenter() : grid_(NULL) {

}

GridSegmenter::~GridSegmenter() {

}

void GridSegmenter::clearRegions() {
  regions_->clear();
}
  
void GridSegmenter::findRegions(dgc_perception_map_cells_p obstacles) {
  clearRegions();

//  printf("min points: %d\n", settings.segmentation_settings.min_points);
  unsigned short region_id = 1;

  shared_ptr<GridObstacle>* current_region = new shared_ptr<GridObstacle>(new GridObstacle(region_id, grid_));

  // clear region labels
  for (int i=0; i < obstacles->num; i++) {
    obstacles->cell[i]->region = 0;
  }

  for (int i=0; i < obstacles->num; i++) {
    dgc_perception_map_cell_p cell = obstacles->cell[i];
    if ((cell->region == 0) /*&& (cell_eval(cell) != PERCEPTION_MAP_OBSTACLE_FREE) */) {
      cell->region = region_id;

//      current_region->get()->timestamp = timestamp_;
      current_region->get()->addCell(cell);
      labelNeighbors(cell, current_region->get());

      int size = current_region->get()->getSize();
      if (size > settings.segmentation_settings.min_points &&
          size < settings.segmentation_settings.max_points &&
          (*current_region)->getPoints().size() != 0) {
        (*current_region)->time_ = timestamp_;

	// -- Find a filled cell and get the robot pose for it.
	(*current_region)->robot_pose_when_observed_ = computeAverageRobotPoseForCells((*current_region)->cells_);
	assert(!isnan((*current_region)->robot_pose_when_observed_.x));
        regions_->push_back(*current_region);
        delete current_region;
        current_region = new shared_ptr<GridObstacle>(new GridObstacle(++region_id, grid_));
      } else {
        current_region->get()->clear();
        current_region->get()->id = ++region_id;
      }
    }
  }
  delete current_region;
}

void GridSegmenter::labelNeighbors(dgc_perception_map_cell_p cell, GridObstacle* current_region)
{
  short row, col;
  cell_to_coord(grid_, cell, &col, &row);
  static int d = settings.segmentation_settings.kernel_size / 2;

  for(int r=row-d;r<=row+d;++r) {
    if(r<0 || r>grid_->rows-1) continue;
    for(int c=col-d;c<=col+d;++c) {
      if(c<0 || c>grid_->cols-1) continue;

      dgc_perception_map_cell_p neighbor = (dgc_perception_map_cell_p)grid_get_rc(grid_, r, c);

      // check cell
      if( neighbor->hits > 0 && /*(cell_eval(neighbor) != PERCEPTION_MAP_OBSTACLE_FREE) &&*/ neighbor->region == 0) {
        neighbor->region = current_region->id;
        current_region->addCell(neighbor);
        labelNeighbors(neighbor, current_region);
      }
    }
  }
}


void GridSegmenter::segmentGrid(dgc_grid_p grid, dgc_perception_map_cells_p obstacles, vector< shared_ptr<Obstacle> >* regions, double timestamp) {
  grid_ = grid;
  regions_ = regions;
  timestamp_ = timestamp;

  findRegions(obstacles);
}

}
