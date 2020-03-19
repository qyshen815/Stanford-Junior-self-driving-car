#include <roadrunner.h>
#include <rndf.h>
#include "rndf_lookup.h"

using namespace dgc;

RndfLookup::RndfLookup(rndf_file *rndf, double closeToNormalLane, 
		       double closeToLaneChange)
{
  this->closeToNormalLane = closeToNormalLane;
  this->closeToLaneChange = closeToLaneChange;
  this->rndf = rndf;
  maxRng=std::max(closeToNormalLane, closeToLaneChange);
  rndf_wp = NULL;
  rndf_heading = NULL;
  is_exit = NULL;
  build_rndf_kdtree();
  rndf_count = 0;
  true_rndf_count = 0;
}

RndfLookup::~RndfLookup()
{
  free(rndf_wp);
  free(rndf_heading);
  free(is_exit);
  dgc_kdtree_free(&rndf_kdtree);
}	

void RndfLookup::update_localize_offset(const Vec2 &newOffset)
{ 
  locOffset = newOffset;
}

bool RndfLookup::closeToRoad(const Vec2 &p, Vec2 *roadVec)
{
  double roadAng=0;
  bool ret=closeToRoad(p, &roadAng);
  roadVec->set(cos(roadAng), sin(roadAng));
  return ret;
}

bool RndfLookup::closeToRoad(const Vec2 &p, double *road_angle)
{
  Vec2 pg=p;
  pg.add(locOffset);
  
/*  rndf_count++;
  true_rndf_count++;
  dgc_kdtree_nearest_neighbor(rndf_kdtree, pg.v[0], pg.v[1], &which, &dist);
*/
  dgc_dlist_node_p lst = dgc_kdtree_range_search(rndf_kdtree, pg.v[0], pg.v[1], maxRng);
  dgc_dlist_node_p lst2=lst;
  bool isClose=false;
  double dist=1000;
  int which=-1;
  while (lst2) {
  	if (lst2->distance<dist) {
  		dist=lst2->distance;
  		which=lst2->id;
  	}
  	if ((is_exit[lst2->id] && lst2->distance < closeToLaneChange) 
  		|| lst2->distance < closeToNormalLane) {
  		isClose=true;
  	}
  	lst2=lst2->next;
  }
  if (which >=0 && road_angle!=NULL) {
  	*road_angle = rndf_heading[which];
  }
  dgc_dlist_free(&lst);
  return isClose;
}

void RndfLookup::resize_wp_memory(int num_waypoints)
{
  if(num_waypoints > max_waypoints) {
    max_waypoints = num_waypoints + 10000;
    rndf_wp = (rndf_waypoint **)realloc(rndf_wp, max_waypoints *
					sizeof(rndf_waypoint *));
    dgc_test_alloc(rndf_wp);
    rndf_x = (double *)realloc(rndf_x, max_waypoints * sizeof(double));
    dgc_test_alloc(rndf_x);
    rndf_y = (double *)realloc(rndf_y, max_waypoints * sizeof(double));
    dgc_test_alloc(rndf_y);
    rndf_heading = (double *)realloc(rndf_heading, max_waypoints * sizeof(double));
    dgc_test_alloc(rndf_heading);
    is_exit = (int *)realloc(is_exit, max_waypoints * sizeof(int));
    dgc_test_alloc(is_exit);
  }
}

void RndfLookup::build_rndf_kdtree()
{
  int num_waypoints = 0;
  int i, j, k, l, l2, n;
  rndf_waypoint *w;
  double alpha;

  rndf_x = NULL; 
  rndf_y = NULL;
  rndf_wp = NULL;
  is_exit = NULL;
  max_waypoints = 0;
  for(i = 0; i < rndf->num_segments(); i++)
    for(j = 0; j < rndf->segment(i)->num_lanes(); j++)
      for(k = 0; k < rndf->segment(i)->lane(j)->num_waypoints(); k++) {
	w = rndf->segment(i)->lane(j)->waypoint(k);
	if(w->next() != NULL) {
	  n = (int)floor(w->length() / 1.0);
	  for(l = 0; l <= n; l++) {
	    resize_wp_memory(num_waypoints + 1);
	    alpha = l / (double)(n + 1);
	    rndf_wp[num_waypoints] = w;
	    rndf_x[num_waypoints] = w->utm_x() + 
	      alpha * (w->next()->utm_x() - w->utm_x());
	    rndf_y[num_waypoints] = w->utm_y() + 
	      alpha * (w->next()->utm_y() - w->utm_y());
	    is_exit[num_waypoints] = 0;
	    rndf_heading[num_waypoints] = w->heading();
	    num_waypoints++;
	  }
	}
	else {
	  resize_wp_memory(num_waypoints + 1);
	  rndf_wp[num_waypoints] = w;
	  rndf_x[num_waypoints] = w->utm_x();
	  rndf_y[num_waypoints] = w->utm_y();
	  rndf_heading[num_waypoints] = w->heading();
      is_exit[num_waypoints] = 0;
	  num_waypoints++;
	}
	for(l = 0; l < w->num_exits(); l++) {
		rndf_waypoint *w2=w->exit(l);
	    if(!w2) continue;
	    n = (int)floor(w->exit_length(l) / 1.0);
	    double ang=atan2(w2->utm_y() - w->utm_y(), 
	      			w2->utm_x() - w->utm_x());
	    for(l2 = 0; l2 <= n; l2++) {
	      resize_wp_memory(num_waypoints + 1);
	      alpha = l2 / (double)(n + 1);
	      rndf_wp[num_waypoints] = w;
	      rndf_x[num_waypoints] = w->utm_x() + 
			alpha * (w2->utm_x() - w->utm_x());
	      rndf_y[num_waypoints] = w->utm_y() + 
			alpha * (w2->utm_y() - w->utm_y());
	      is_exit[num_waypoints] = 1;
	      rndf_heading[num_waypoints] = ang;
	      num_waypoints++;
	    }
	}
  }
  rndf_kdtree = dgc_kdtree_build_balanced(rndf_x, rndf_y, num_waypoints);
  free(rndf_x);
  free(rndf_y);
}

