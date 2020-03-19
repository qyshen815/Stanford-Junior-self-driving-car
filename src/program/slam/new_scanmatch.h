#ifndef DGC_NEW_SCANMATCH_H
#define DGC_NEW_SCANMATCH_H

#include <roadrunner.h>
#include <velo_support.h>
#include <ANN.h>
#include <vector>

struct LaserPoint {
  double x, y, z;
  float range;
};

class PointSet {
public:
  PointSet();
  ~PointSet();
  void BuildKDTree(void);
  void Clear(void) { point_.clear(); }
  int num_points(void) { return (int)point_.size(); }
  void AppendSpin(dgc::dgc_velodyne_spin *spin);
  void AlignScan(PointSet *scan, dgc_transform_t base_t, 
		 dgc_transform_t t, double max_match_dist);

private:
  std::vector <LaserPoint> point_;
  ANNpointArray kdtree_points_;
  ANNkd_tree *kdtree_;
  ANNpoint query_point_;
  ANNidxArray nn_index_;
  ANNdistArray nn_dist_;
};


#endif
