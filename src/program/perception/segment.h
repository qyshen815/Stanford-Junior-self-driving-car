/*
 *  Created on: Jul 24, 2009
 *      Author: duhadway
 */

#ifndef SEGMENTER_H_
#define SEGMENTER_H_

#include <grid.h>

#include "perception.h"
#include "obstacle.h"

namespace dgc {

class Segmenter {
public:
  Segmenter();
  virtual ~Segmenter();
};

// a grid region
typedef struct {
  int id;
  int num_points;
  std::vector<dgc_perception_map_cell_p> cells;
} dgc_perception_map_region_t, *dgc_perception_map_region_p;

class GridSegmenter : public Segmenter {
private:
  std::vector<std::tr1::shared_ptr<dgc::Obstacle> >* regions_;
  dgc_grid_p grid_;
  double timestamp_;

  void clearRegions();
  void regionsToObstacles(std::vector<Obstacle*>& out);
  void labelNeighbors(dgc_perception_map_cell_p cell, GridObstacle* current_region);
  void findRegions(dgc_perception_map_cells_p obstacles);
public:
  GridSegmenter();
  virtual ~GridSegmenter();

  void segmentGrid(dgc_grid_p grid, dgc_perception_map_cells_p obstacles, std::vector< std::tr1::shared_ptr<dgc::Obstacle> >* regions, double timestamp);
};

}

#endif /* SEGMENTER_H_ */
