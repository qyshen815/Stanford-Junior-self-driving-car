/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * file .......: rndf_waypoint_quadtree.h
 * authors ....: Benjamin Pitzer
 * organization: Stanford University
 * creation ...: 02/15/2007
 * revisions ..: $Id: rndf_waypoint_quadtree.h,v 1.1 2007-04-16 04:51:22 pitzer Exp $
---------------------------------------------------------------------*/
#ifndef RNDF_WAYPOINT_QUADTREE_H
#define RNDF_WAYPOINT_QUADTREE_H

//== INCLUDES =================================================================
#include <vector>
#include <stack>
#include <sstream>
#include <limits>
#include <vec2.h>
#include <rndf.h>
#include "rndf_types.h"

//== NAMESPACES ===============================================================
namespace dgc {
                                                     
//== CLASS DEFINITION =========================================================
class rndf_waypoint_quadtree 
{
public: 
  typedef struct {
    rndf_waypoint_p w;
    sla::Vec2d& pos;
  } element_t, *element_p;
  typedef std::vector<element_p> element_list_t,*element_list_p;
  typedef std::vector<int> index_list_t,*index_list_p;                                                    
                                                       
  rndf_waypoint_quadtree(
    rndf_file*            rndf, 
    double                min_radius = 0, 
    int                   min_elements = 1);

  rndf_waypoint_quadtree(
    rndf_file*            rndf, 
    sla::Vec2d&           center, 
    double                radius, 
    double                min_radius = 0, 
    int                   min_elements = 1);

  rndf_waypoint_quadtree(
    const element_list_p  elements,
    const index_list_t&   indices,
    sla::Vec2d&           center, 
    double                radius, 
    double                min_radius = 0, 
    int                   min_elements = 1, 
    int                   parent_index = 0, 
    rndf_waypoint_quadtree*   parent_tree = NULL);

  ~rndf_waypoint_quadtree(void);

  bool search_nearest_neighbor(const sla::Vec2d& p, element_p& element, double& distance, sla::Vec2d& closest_p);
  
  bool is_leaf() { return m_is_leaf; }
  unsigned long long tile_id() { return m_tile_id; }
  
  void get_tiles(std::vector<sla::Vec2d> &points, std::vector<int> &edges, std::vector<unsigned long long> &tile_ids);
  void get_elements(element_list_t& elements);
  
private:
  void initialize(const index_list_t& indices, double min_radius, int min_elements);
  bool closer_to_element(const sla::Vec2d& p, element_p element, 
                         double& distance, sla::Vec2d& closest_point); 
  bool element_inside_quad(const sla::Vec2d& center, double radius, element_p element);
    
private:
  element_list_p                    m_elements;
  index_list_t                      m_indices;
  sla::Vec2d                        m_center;
  double                            m_radius;
  int                               m_parent_index;
  rndf_waypoint_quadtree*           m_parent_tree;
  rndf_waypoint_quadtree*           m_child[4];
  bool                              m_is_leaf;
  unsigned long long                m_tile_id;
};

//=============================================================================
} // NAMESPACE dgc
//=============================================================================
//=============================================================================
#endif // RNDF_WAYPOINT_QUADTREE_H defined
//=============================================================================
