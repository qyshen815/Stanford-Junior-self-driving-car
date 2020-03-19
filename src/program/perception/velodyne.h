#ifndef PERCEPTION_VELODYNE_H
#define PERCEPTION_VELODYNE_H


#include "perception.h"
#include "utils.h"
#include "velodyne_rings.h"
#include <ext/hash_set>
#include <ext/hash_map>

dgc_pose_t computeAverageRobotPoseForCell(dgc_perception_map_cell_p cell);
dgc_pose_t computeAverageRobotPoseForCells(const std::vector<dgc_perception_map_cell_p>& cells);

#endif // PERCEPTION_VELODYNE_H
