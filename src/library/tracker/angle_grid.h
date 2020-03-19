#ifndef ANGLE_GRID_H
#define ANGLE_GRID_H

#include <roadrunner.h>

#include "projected_scan.h"
#include "vectors.h"

typedef struct {
  double rng;
  bool markedAsCleared;
  bool obsMarked;
  projected_point *origPt;
} AngleCell;

class AngleGrid {

 public:	
  Vec2        vantagePose;
  double      vantageYaw;
  double      ts;
  int         N;
  double      resolution;
  double      startAngle, endAngle;
  AngleCell  *ag;
  double      sinYaw, cosYaw;
  int         ptsAdded;
  bool        debug;

  AngleGrid();
  ~AngleGrid();

  void        clear(void);
  void        set_robot_pose(const Vec2 &inVantagePose, 
			     double inVantageYaw,  double inTS);
  void        unmarkClearedPoints(void);
  void        markClearedPoints(AngleGrid *ag2);
  
  void        addPoint(const Vec2 &p, projected_point *origPt);
  bool        valid(int ind);
  bool        withinBounds(int ind);
  bool        withinBounds(const Vec2 &localPt);
  int         getGridCoords(const Vec2 &localPt);
  int         worldToGrid(const Vec2 &worldPt);
  AngleCell  *get(int ind);
  Vec2        getWorldCoords(int ind);
  Vec2        getWorldCoordsAtRange(int ind, double rng);
  Vec2        toLocal(const Vec2 &p);
  Vec2        toGlobal(const Vec2 &p);
  double      compareRange(const Vec2 &p);
  Vec2i       getCoordRange(Vec2 *pts, int numPts);

};
#endif
