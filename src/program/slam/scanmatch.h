#ifndef DGC_SCANMATCH_H
#define DGC_SCANMATCH_H

#include <roadrunner.h>
#include <transform.h>
#include <ANN.h>

typedef struct {
  double x, y, z;
  float range;
} point_t, *point_p;

typedef struct {
  int num_points, max_points;
  point_p point;
} pointset_t, *pointset_p;

typedef struct {
  ANNpointArray kdtree_dataPts;
  ANNkd_tree *kdtree;
  ANNpoint queryPt;
  ANNidxArray nn_idx;
  ANNdistArray nn_dist;
} kdtree_info_t, *kdtree_info_p;

kdtree_info_p 
build_kdtree(pointset_p p);

void
free_kdtree(kdtree_info_p *kinfo);

pointset_p
alloc_pointset(void);

void
free_pointset(pointset_p *p);

void
write_transformed_pointset_to_file(pointset_p p, dgc_transform_t t, 
				   char *filename);

void
write_pointset_to_file(pointset_p p, char *filename);

void
scan_match(pointset_p scan, pointset_p model, kdtree_info_p kinfo,
	   dgc_transform_t icp_shift, double max_match_dist);

#endif
