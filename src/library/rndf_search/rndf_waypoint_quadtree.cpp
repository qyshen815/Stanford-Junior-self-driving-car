/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * file .......: rndf_waypoint_quadtree.cpp
 * authors ....: Benjamin Pitzer
 * organization: Stanford University
 * creation ...: 02/15/2007
 * revisions ..: $Id: rndf_waypoint_quadtree.cpp,v 1.1 2007-04-16 04:51:22 pitzer Exp $
---------------------------------------------------------------------*/
#define DEBUG_LEVEL 0

//== INCLUDES =================================================================
#include "geometry_2d.h"
#include "rndf_waypoint_quadtree.h"

//== NAMESPACES ===============================================================
using namespace std;

namespace dgc {

//== IMPLEMENTATION ==========================================================
rndf_waypoint_quadtree::
rndf_waypoint_quadtree(
  rndf_file*            rndf, 
  double                min_radius, 
  int                   min_elements) 
  :  m_elements(NULL), m_tile_id(1)
{
  sla::Vec2d bb_min(MAXDOUBLE);
  sla::Vec2d bb_max(-MAXDOUBLE);
  int i, j, k, n;
  element_p element;  
  int num_segments,num_lanes,num_waypoints;
  
  m_elements = new element_list_t();
  index_list_t indices;
  
  // calculate bounding box and create elements
  n = 0;
  num_segments = rndf->num_segments();
  for(i = 0; i < num_segments; i++) {
    num_lanes = rndf->segment(i)->num_lanes();
    for(j = 0; j < num_lanes; j++) {
      num_waypoints = rndf->segment(i)->lane(j)->num_waypoints();
      for(k = 0; k < num_waypoints; k++) {
        // calculate bounding box
        bb_min.x[0] = min(bb_min.x[0],rndf->segment(i)->lane(j)->waypoint(k)->utm_x());
        bb_max.x[0] = max(bb_max.x[0],rndf->segment(i)->lane(j)->waypoint(k)->utm_x());
        bb_min.x[1] = min(bb_min.x[1],rndf->segment(i)->lane(j)->waypoint(k)->utm_y());
        bb_max.x[1] = max(bb_max.x[1],rndf->segment(i)->lane(j)->waypoint(k)->utm_y());
        
        printf("%d %f %f\n",n,rndf->segment(i)->lane(j)->waypoint(k)->utm_x(),rndf->segment(i)->lane(j)->waypoint(k)->utm_y());
        
        // create elements
        element = (element_p)malloc(sizeof(element_t));
        element->w = rndf->segment(i)->lane(j)->waypoint(k);
        element->pos.set(element->w->utm_x(),element->w->utm_y());
        m_elements->push_back(element);
        indices.push_back(n);
        n++;  
      } // k<num_waypoints-1
    } // j
  } // i
      
  // set members 
  m_center = bb_min+(bb_max-bb_min)/2.0;
  m_radius = (bb_max-bb_min).max();
  m_parent_index = 0;
  m_parent_tree = NULL;
 
  // initialize quadtree
  initialize(indices,min_radius,min_elements);
}

rndf_waypoint_quadtree::
rndf_waypoint_quadtree(
  rndf_file*            rndf, 
  sla::Vec2d&           center,  
  double                radius, 
  double                min_radius, 
  int                   min_elements)
  : m_elements(NULL), m_center(center), m_radius(radius), m_tile_id(1)
{
  int i, j, k, n;
  element_p element;  
  int num_segments,num_lanes,num_waypoints;
  
  m_elements = new element_list_t();
  index_list_t indices;
  
  // calculate bounding box and create elements
  n = 0;
  num_segments = rndf->num_segments();
  for(i = 0; i < num_segments; i++) {
    num_lanes = rndf->segment(i)->num_lanes();
    for(j = 0; j < num_lanes; j++) {
      num_waypoints = rndf->segment(i)->lane(j)->num_waypoints();
      for(k = 0; k < num_waypoints; k++) {
        // create elements
        element = (element_p)malloc(sizeof(element_t));
        element->w = rndf->segment(i)->lane(j)->waypoint(k);
        element->pos.set(element->w->utm_x(),element->w->utm_y());
        m_elements->push_back(element);
        indices.push_back(n);
        n++;  
      } // k
    } // j
  } // i
      
  // set members 
  m_parent_index = 0;
  m_parent_tree = NULL;
  
  // initialize quadtree
  initialize(indices,min_radius,min_elements);
}

rndf_waypoint_quadtree::
rndf_waypoint_quadtree(
    const element_list_p  elements,
    const index_list_t&   indices,
    sla::Vec2d&           center, 
    double                radius, 
    double                min_radius, 
    int                   min_elements, 
    int                   parent_index, 
    rndf_waypoint_quadtree*   parent_tree)
  : m_elements(elements), m_center(center), m_radius(radius),
    m_parent_index(parent_index), m_parent_tree(parent_tree)
{
  if(m_parent_tree)
    m_tile_id = m_parent_tree->tile_id()*10+parent_index;
  else
    m_tile_id=1;
    
  initialize(indices,min_radius,min_elements);
}

rndf_waypoint_quadtree::
~rndf_waypoint_quadtree()
{
  element_p element;
  if (m_is_leaf) {
    element_list_t::const_iterator it, it_end;
    for(it=m_elements->begin(), it_end=m_elements->end(); it!=it_end; ++it) {
      element = *it;
      if(element) {
        free(element);
        element=NULL;
      }
    }
    m_elements->clear();
    return;
  }
  for (int i=0; i<4; i++) {
    if (m_child[i]) 
      delete m_child[i];
  }
}

void rndf_waypoint_quadtree::
initialize(const index_list_t& indices, double min_radius, int min_elements)
{
  bool subdivided=false;
  double center_x = m_center(0);
  double center_y = m_center(1);
  element_p element;
  m_indices.clear();
  m_is_leaf = false;

  // check whether we need to subdivide more
  if ((m_radius < 2.0*min_radius) || indices.size()==1) {
    m_is_leaf = true;
    m_indices = indices;
    m_child[0] = m_child[1] = m_child[2] = m_child[3] = NULL;
    return;
  }

  // if so, try for triangle and each child whether they intersect
  double half_radius = m_radius*.5;
  sla::Vec2d child_center;

  index_list_t xp, xm; // x plus, x minus
  index_list_t yp, ym; // y plus, y minus
  xp.reserve(indices.size());
  xm.reserve(indices.size());
  
  // split about x axis
  index_list_t::const_iterator it, it_end;
  for(it=indices.begin(), it_end=indices.end(); it!=it_end; ++it)
  {
    element = m_elements->at(*it);
    if (element->pos(0) >= center_x)
      xp.push_back(*it); 
    else 
      xm.push_back(*it);   
  }
 
  printf("xp %d xm %d\n",xp.size(),xm.size());
  
  // split about y axis
  index_list_t &xv = xm;
  for (int i=0; i<2; i++) {
    if (i) xv = xp;
    else   xv = xm;
    yp.clear(); yp.reserve(xv.size());
    ym.clear(); ym.reserve(xv.size());

    for(it=xv.begin(), it_end=xv.end(); it!=it_end; ++it) 
    {
      element = m_elements->at(*it);
      if (element->pos(1) >= center_y)
        yp.push_back(*it);
      else
        ym.push_back(*it);  
    }

    // check the unknowns
    child_center(0) = m_center(0) + (i ? half_radius : -half_radius);
    child_center(1) = m_center(1) - half_radius;
   
    // create the children, if needed
    int k = i;
    if (ym.size()>0) {    
      m_child[k] = new rndf_waypoint_quadtree(m_elements, ym, child_center, half_radius, 
                                              min_radius, min_elements, 
                                              k, this);
      subdivided = true;
    } else {
      m_child[k] = NULL;
    }
    
    // create the children, if needed
    child_center(1) = m_center(1) + half_radius;
    k += 2;
   
    if (yp.size()>0) {
      m_child[k] = new rndf_waypoint_quadtree(m_elements, yp, child_center, half_radius, 
                                              min_radius, min_elements, 
                                              k, this);
      subdivided = true;
    } else {
      m_child[k] = NULL;
    }
  }

  // no subdivision made
  if(!subdivided)
  {
    m_indices = indices;
    m_is_leaf = true;
    m_child[0] = m_child[1] = m_child[2] = m_child[3] = NULL;
  }
}

bool rndf_waypoint_quadtree::
element_inside_quad(const sla::Vec2d& center, double radius, element_p element)
{
  bool outside = point_outside_of_rect(center, radius*2.0, element->pos);
  return (!outside);
}

bool rndf_waypoint_quadtree::
closer_to_element(const sla::Vec2d& p, element_p element, 
                  double& distance, sla::Vec2d& cp) 
{
  double nd = dist2(p,element->pos);
  if (nd < distance) { cp = element->pos; distance = nd; return true; }
  return false;
}

bool rndf_waypoint_quadtree::
search_nearest_neighbor(const sla::Vec2d& p, element_p& element, double& distance, sla::Vec2d& closest_point)
{
  // is this a leaf node?
  if (m_is_leaf) {
    // look for a new closest distance
    index_list_t::const_iterator it, it_end;
    for(it=m_indices.begin(), it_end=m_indices.end(); it!=it_end; ++it) 
    {
      // check if distance to this element is smaller
      if(closer_to_element(p,m_elements->at(*it),distance,closest_point))
        element = m_elements->at(*it);
    }
    return circle_within_bounds(p, sqrt(distance), m_center, m_radius);
  }

  // which child contains p?
  int iChild = 2*(m_center(1)<p(1)) + (m_center(0)<p(0));
  // check that child first
  if (m_child[iChild] && m_child[iChild]->search_nearest_neighbor(p, element, distance, closest_point)) return true;

  // now see if the other children need to be checked
  for (int i=0; i<4; i++) {
    if (i==iChild) continue;
    if (m_child[i]) {
      if(bounds_overlap_circle(p,sqrt(distance),m_child[i]->m_center, m_child[i]->m_radius)) {
        if (m_child[i]->search_nearest_neighbor(p, element, distance, closest_point)) return true;
      }
    }
  }
  return circle_within_bounds(p, sqrt(distance), m_center, m_radius);
}

void rndf_waypoint_quadtree::
get_tiles(std::vector<sla::Vec2d> &points, std::vector<int> &edges, std::vector<unsigned long long> &tile_ids) 
{
  if (m_is_leaf) {
    int s = points.size();
    points.push_back(m_center + sla::Vec2d(-m_radius,-m_radius));//0
    points.push_back(m_center + sla::Vec2d( m_radius,-m_radius));//1
    points.push_back(m_center + sla::Vec2d( m_radius, m_radius));//2
    points.push_back(m_center + sla::Vec2d(-m_radius, m_radius));//3
    
    edges.push_back(s+0);edges.push_back(s+1);
    edges.push_back(s+1);edges.push_back(s+2);
    edges.push_back(s+2);edges.push_back(s+3);
    edges.push_back(s+3);edges.push_back(s+0);
    
    tile_ids.push_back(m_tile_id);
    return;
  }
  for (int i=0; i<4; i++) {
    if (m_child[i]) 
      m_child[i]->get_tiles(points,edges,tile_ids);
  }
}

void rndf_waypoint_quadtree::
get_elements(element_list_t& elements) 
{
  if (m_is_leaf) {
    element_list_t::const_iterator it, it_end;
    for(it=m_elements->begin(), it_end=m_elements->end(); it!=it_end; ++it)
      elements.push_back(*it);
    return;
  }
  for (int i=0; i<4; i++) {
    if (m_child[i]) 
      m_child[i]->get_elements(elements);
  }
}

  
//=============================================================================
} // namespace dgc
//=============================================================================
