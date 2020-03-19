#include <roadrunner.h>
#include "angle_grid.h"

using std::cout;
using std::endl;

AngleGrid::AngleGrid() 
{
  resolution = dgc_d2r(1) * 0.5;
  startAngle = -dgc_d2r(170);
  endAngle = -startAngle;
  N = (int)((endAngle - startAngle) / resolution);
  ag = new AngleCell[N];
  clear();
}
  
AngleGrid::~AngleGrid() 
{
  delete ag;
}
  
void 
AngleGrid::clear(void) 
{
  int i;
  
  ptsAdded = 0;
  for(i = 0; i < N; i++) {
    ag[i].rng = 0; 
    //    ag[i].x = 0; 
    //    ag[i].y = 0; 
    ag[i].markedAsCleared = false;
    ag[i].obsMarked = false;
  }
  debug = false;
}

void 
AngleGrid::set_robot_pose(const Vec2 &inVantagePose, double inVantageYaw, double inTS) 
{
  vantagePose = inVantagePose;
  vantageYaw = inVantageYaw;
  sinYaw = sin(vantageYaw);
  cosYaw = cos(vantageYaw);
  ts = inTS;
}

void 
AngleGrid::unmarkClearedPoints(void) 
{
  int i;
  
  for(i = 0; i < N; i++) 
    ag[i].markedAsCleared = false;
}	

void 
AngleGrid::markClearedPoints(AngleGrid *ag2) 
{
  int i;
  
  for(i = 0; i < N; i++) {
    if(ag[i].rng < 0.1) 
      continue;
    Vec2 v = getWorldCoords(i);
    double rangeCmp = ag2->compareRange(v);
    if(rangeCmp < -0.25)
      ag[i].markedAsCleared = true;
  }
}

void 
AngleGrid::addPoint(const Vec2 &p, projected_point *origPt)
{
  Vec2 localPt = toLocal(p);
  int ind = getGridCoords(localPt);
  double r;
  
  if(!withinBounds(ind))
    return;
  
  r = localPt.length();
  if(ag[ind].rng == 0 || r < ag[ind].rng) {
    ag[ind].rng = r;
    ag[ind].origPt = origPt;
  }
  ptsAdded++;
}

bool
AngleGrid::valid(int ind) {
  if(!withinBounds(ind)) 
    return false;
  if(get(ind)->rng < 0.1) 
    return false;
  return true;
}

bool
AngleGrid::withinBounds(int ind) {
  if(ind >= 0 && ind < N)
    return true;
  return false;
}

bool
AngleGrid::withinBounds(const Vec2 &localPt) {
  return withinBounds(getGridCoords(localPt));	
}

int
AngleGrid::getGridCoords(const Vec2 &localPt) 
{
  double pYaw = atan2(localPt.v[1], localPt.v[0]);
  pYaw -= startAngle;
  int ind = (int)floor(pYaw / resolution);
  return ind;
}

int
AngleGrid::worldToGrid(const Vec2 &worldPt) {
  Vec2 localPt = toLocal(worldPt);
  return getGridCoords(localPt);	
}

AngleCell *
AngleGrid::get(int ind) { 
  if(!withinBounds(ind)) {
    cout << "AngleGrid: get: index out of bounds: " << ind << endl;
    return NULL;
  }
  return &(ag[ind]); 
}

Vec2
AngleGrid::getWorldCoords(int ind) { 
  double rng = get(ind)->rng;
  double ptYaw = (ind + 0.5) * resolution + startAngle;
  Vec2 ret(rng * cos(ptYaw), rng * sin(ptYaw)); 
  ret = toGlobal(ret);
  return ret; 
}

Vec2
AngleGrid::getWorldCoordsAtRange(int ind, double rng) {
  double ptYaw = (ind + 0.5) * resolution + startAngle;
  Vec2 ret(rng * cos(ptYaw), rng * sin(ptYaw)); 
  ret = toGlobal(ret);
  return ret; 	
}
  
Vec2
AngleGrid::toLocal(const Vec2 &p) {
  Vec2 tp = p;
  tp.v[0] -= vantagePose.v[0];
  tp.v[1] -= vantagePose.v[1];
  Vec2 ret = tp;
  ret.v[0] = cosYaw * tp.v[0] + sinYaw * tp.v[1];
  ret.v[1] = -sinYaw * tp.v[0] + cosYaw * tp.v[1];
  return ret;
}

Vec2
AngleGrid::toGlobal(const Vec2 &p) {
  Vec2 tp = p;
  tp.v[0] = cosYaw * p.v[0] - sinYaw * p.v[1] +vantagePose.v[0];
  tp.v[1] = sinYaw * p.v[0] + cosYaw * p.v[1] +vantagePose.v[1];
  return tp;
}

double
AngleGrid::compareRange(const Vec2 &p) {
  Vec2 localPt = toLocal(p);
  double ptRng = localPt.length();
  int ind = getGridCoords(localPt);
  if(!withinBounds(ind)) 
    return ptRng;
  return ptRng - ag[ind].rng;
}
  

Vec2i
AngleGrid::getCoordRange(Vec2 *pts, int numPts) {
  int ind;
  int minInd = N + 1;
  int maxInd = -1;
  
  for(int i = 0; i < numPts; i++) {
    ind = worldToGrid(pts[i]);
    if(ind < minInd) 
      minInd = ind;
    if(ind > maxInd) 
      maxInd = ind;
  }
  if(minInd < 0)
    minInd = 0;
  if(minInd >= N) 
    minInd = maxInd + 1;
  if(maxInd >= N) 
    maxInd = N - 1;
  if(maxInd < 0) 
    maxInd = minInd - 1;
  Vec2i ret(minInd, maxInd);
  return ret;
}
