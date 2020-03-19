#include <roadrunner.h>
#include <velocore.h>
#include <velo_support.h>
#include <ANN.h>
#include "new_scanmatch.h"

using namespace dgc;

PointSet::PointSet() 
{
  kdtree_points_ = NULL;
  kdtree_ = NULL;
  query_point_ = annAllocPt(3);
  nn_index_ = new ANNidx[1];
  nn_dist_ = new ANNdist[1];
}

PointSet::~PointSet()
{
  delete query_point_;
  delete nn_index_;
  delete nn_dist_;

  if (kdtree_ != NULL)
    delete kdtree_;
  if (kdtree_points_ != NULL)
  annDeallocPts(kdtree_points_);
}

void PointSet::AppendSpin(dgc_velodyne_spin *spin)
{
  LaserPoint p;
  int i, j;
  
  for(i = 0; i < spin->num_scans; i++) 
    for(j = 0; j < VELO_BEAMS_IN_SCAN; j++) {
      if(spin->scans[i].p[j].range < 0.01)
	continue;

      p.x = spin->scans[i].p[j].x * 0.01 + spin->scans[i].robot.x;
      p.y = spin->scans[i].p[j].y * 0.01 + spin->scans[i].robot.y;
      p.z = spin->scans[i].p[j].z * 0.01 + spin->scans[i].robot.z;
      p.range = spin->scans[i].p[j].range * 0.01;
      point_.push_back(p);
    }
}

void PointSet::BuildKDTree(void)
{
  int i, n = num_points();

  if (kdtree_ != NULL) 
    delete kdtree_;
  if (kdtree_points_ != NULL)
    annDeallocPts(kdtree_points_);
  kdtree_points_ = annAllocPts(n, 3);
  for(i = 0; i < n; i++) {
    kdtree_points_[i][0] = point_[i].x;
    kdtree_points_[i][1] = point_[i].y;
    kdtree_points_[i][2] = point_[i].z;
  }
  kdtree_ = new ANNkd_tree(kdtree_points_, n, 3, 10);
}

void PointSet::AlignScan(PointSet *scan, dgc_transform_t base_t,
			 dgc_transform_t t, double max_match_dist)
{
  double max_d_sq = dgc_square(max_match_dist);
    double x, y, z, dx = 0, dy = 0, dz = 0;
    int i, num_matches = 0;

  /* find correpondences */
  for (i = 0; i < scan->num_points(); i++) {
    query_point_[0] = scan->point_[i].x;
    query_point_[1] = scan->point_[i].y;
    query_point_[2] = scan->point_[i].z;
    dgc_transform_point(&query_point_[0], &query_point_[1], 
			&query_point_[2], base_t);
    x = query_point_[0];
    y = query_point_[1];
    z = query_point_[2];
    dgc_transform_point(&query_point_[0], &query_point_[1], 
			&query_point_[2], t);
    kdtree_->annkSearch(query_point_, 1, nn_index_, nn_dist_, 0.0);
    if (nn_dist_[0] < max_d_sq) {
      dx += (point_[nn_index_[0]].x - x);
      dy += (point_[nn_index_[0]].y - y);
      dz += (point_[nn_index_[0]].z - z);
      num_matches++;
    }
  }

  dgc_transform_identity(t);
  t[0][3] = dx / (double)num_matches;
  t[1][3] = dy / (double)num_matches;
  t[2][3] = dz / (double)num_matches;
}


