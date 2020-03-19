/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team vlr
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef LANEQUADTREE_H_
#define LANEQUADTREE_H_

//== INCLUDES =================================================================
#include <vector>
#include <stack>
#include <sstream>
#include <limits>
#include <sla.h>

namespace vlr {

namespace rndf
{
class Segment;
class Lane;
class WayPoint;
class RoadNetwork;

  //== CLASS DEFINITION =========================================================
  class LaneQuadTree
  {
  public:
    typedef struct {
      Segment*  s;
      Lane*     l;
      WayPoint* w1;
      WayPoint* w2;
    } element_t, *element_p;
    typedef std::vector<element_p> element_list_t,*element_list_p;

    LaneQuadTree(
      RoadNetwork*          rndf,
      double                min_radius = 0,
      int                   min_elements = 1);

    LaneQuadTree(
      RoadNetwork*            rndf,
      sla::Vec2d&           center,
      double                radius,
      double                min_radius = 0,
      int                   min_elements = 1);

    LaneQuadTree(
      RoadNetwork*            rndf,
      sla::Vec2d&           center,
      double                radius,
      const element_list_t& elements,
      double                min_radius = 0,
      int                   min_elements = 1,
      int                   parent_index = 0,
      LaneQuadTree*   parent_tree = NULL);

    ~LaneQuadTree(void);

    bool search_nearest_neighbor(const sla::Vec2d& p, element_p& element, double& distance, sla::Vec2d& closest_p);

    bool is_leaf() { return m_is_leaf; }
    unsigned long long tile_id() { return m_tile_id; }

    void get_tiles(std::vector<sla::Vec2d> &points, std::vector<int> &edges, std::vector<unsigned long long> &tile_ids);
    void get_elements(element_list_t& elements);
  private:
    void initialize(const element_list_t& elements, double min_radius, int min_elements);
    bool closer_to_element(const sla::Vec2d& p, element_p element,
                           double& distance, sla::Vec2d& closest_point);
    bool element_inside_quad(const sla::Vec2d& center, double radius, element_p element);

  private:
    RoadNetwork*                        m_rndf;
    sla::Vec2d                        m_center;
    double                            m_radius;
    int                               m_parent_index;
    LaneQuadTree*                     m_parent_tree;
    LaneQuadTree*                     m_child[4];
    bool                              m_is_leaf;
    element_list_t                    m_elements;
    unsigned long long                m_tile_id;
  };

};

} // namespace vlr

#endif /*LANEQUADTREE_H_*/
