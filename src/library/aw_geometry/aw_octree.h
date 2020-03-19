/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_OCTREET_H
#define AW_OCTREET_H

//== INCLUDES =================================================================
#include <vec3.h>
#include <vector>
#include <stack>
#include <sstream>
#include <limits>

namespace vlr {

//== CLASS DEFINITION =========================================================
  template <class T>
  class OctreeT
  {

  public:
    typedef T Scalar;
    typedef typename sla::Vec3<Scalar> Point;
    typedef typename std::vector<Point> PointList;
    typedef typename PointList::iterator PointListIterator;
    typedef typename PointList::const_iterator PointListConstIterator;
    typedef typename std::vector<int> IndexList;
    typedef typename IndexList::const_iterator IndexListConstIterator;

    OctreeT(
      int               num_points,
      const Point*      points,
      Scalar            minRadius,
      int               minPoints);

    OctreeT(
      int               num_points,
      const Point*      points,
      Point             center,
      Scalar            radius,
      Scalar            minRadius,
      int               minPoints);

    OctreeT(
      int               num_points,
      const Point*      points,
      Point             center,
      Scalar            radius,
      IndexList&        indices,
      Scalar            minRadius,
      int               minPoints = 1,
      int               parentIndex = 0,
      OctreeT<T>*       parentTree = NULL);

    ~OctreeT(void);

    bool searchNearest(const Point &p, int& nearestNeighbor, Scalar& distance);
    void refineTree(Scalar minRadius, int minPoints);

    void getCubes(PointList &points, std::vector<int> &edges);
    void getCubeCenters(PointList &centers);
    void getPoints(PointList &points);
    void getIndices(IndexList& indices);
    void getLeaves(std::vector< OctreeT<T>* >& leafes);

    Point center() { return m_center; }
    Scalar radius() { return m_radius; }
    bool isLeaf() { return m_isLeaf; }


  private:
    void initialize(IndexList& indices, Scalar& minRadius, int& minVertices);
    bool closerToPoint(const Point &p, const int& otherIndex, Scalar& distance);

  private:
    int                       m_num_points;
    const Point*              m_points;
    Point                     m_center;
    Scalar                    m_radius;
    int                       m_parentIndex;
    OctreeT<T>*               m_parentTree;
    OctreeT<T>*               m_child[8];

    bool                      m_isLeaf;
    std::vector<int>          m_indices; // point indices, groups of three
  };
//=============================================================================

} // namespace vlr

#if !defined(OCTREE_TEMPLATE)
#include "aw_octree_template.h"
#endif

#endif // AW_OCTREET_H defined

