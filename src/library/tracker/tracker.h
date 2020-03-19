#ifndef TRACKER_H
#define TRACKER_H

#include <roadrunner.h>
#include <perception_messages.h>
#include <gls_interface.h>

#include <vector>
#include <iostream>
#include <fstream>

#include "vectors.h"
#include "misc.h"

#include "sampling.h"
#include "angle_grid.h"
#include "rndf_lookup.h"

extern vlr::GlsOverlay *lasertrack_gls;

typedef std::vector< Vec2 > PointList;
struct DynObsMsg {
  dgc::PerceptionDynamicObstacle msg;
  PointList pts;
};

typedef std::vector< DynObsMsg > DynObsMsgList;
typedef DynObsMsg * DynObsMsgP;

enum { DO_INSIDE, DO_NEAR, DO_FAR };

class DynamicObstacle {

 public:

  Vec2       origin; //in world coords
  double     yaw; //in radians
  Vec2       dim; //dimentions along x,y in meters
  double     zDim; //dimention along z in meters (for display purposes)
  double     score, wt;
  double     sinYaw,cosYaw;
  double     dNearby; // in meters, max distance to nearby points
  double     vel; //forward velocity in m/s
  double     ts;
  int        obsID;
  int        type;
  double     dYaw, dYaw2, rearWheelOffset;
  PointList  pts;
  
  bool       significantSides[4]; //front, left, back, right
  
  // for 2D line intersection
  Vec2       corners2D[4];
  Vec2       worldCorners2D[4];
  Vec2       boundingCorners2D[4];
  Vec2       worldBoundingCorners2D[4];
  
  
  DynamicObstacle(double len, double width, double height);

  void    toMsg(DynObsMsgP wrapperMsg);
  void    fromMsg(DynObsMsgP wrapperMsg);
  void    moveTo(Vec2 &pos, double inYaw);
  void    drive(double dt);
  void    markOnAG(AngleGrid *ag, double pad=0);
  void    gls_render();
  void    gls_render(Vec3 &color);
  Vec2    toLocal(Vec2 &p);
  Vec2    toGlobal(Vec2 &p);
  bool    containsPoint(Vec2 &p);
  int     nearbyPoint(Vec2 &p);
  void    setSignificantSides(Vec2 &viewPt);
  bool    visibleSurfacePoint(Vec2 &worldPt);
  double  externalDist(Vec2 &p);
  void    initCorners2D();
  void    computeWorldCorners2D();
  bool    getCornerIntersect2D(Vec2 &p1, Vec2 &p2,
			       Vec2 &r1, Vec2 &r2);
  bool    getBoundingIntersect2D(Vec2 &p1, Vec2 &p2,
				 Vec2 &r1, Vec2 &r2);
};


class CarModel : public ModelBase {

 public:
  double           minCount;
  RndfLookup      *rl;
  DynamicObstacle *tmpCar;
  SIR             *sir;
  double           normConst;
  double           varInsideFree, varOutsideOcc, varSurfaceFree;
  Vec2             vantagePoint;
  AngleGrid       *ag, *agPrev;
  int              scoreCnt;
  double           minPercent;
  
  CarModel(RndfLookup *inRl, DynamicObstacle *tCar);
  ~CarModel();
  
  double  computeLikelihood(SampleP s);
  void    sampleToCar(SampleP s, DynamicObstacle *car);
  void    carToSample(DynamicObstacle *car, SampleP s);
  void    computeScore(DynamicObstacle *car);
  DynamicObstacle * computeMeanCar();

};


class DynamicObstaclePF {

public:
  int                life;
  int                badTurns;
  double             lastGoodUpdate, firstSeen;
  SIR               *sir;
  DynamicObstacle  **cars;
  int                N;
  int                maxN;
  RndfLookup        *rl;
  CarModel          *carModel;
  DynamicObstacle   *mean, *est;
  double             lastTs, curTs, dt;
  int                numNewCars;
  bool               initialVel;
  int                obsID;
  double             topScore;
  bool               aCopy;
  
  DynamicObstaclePF(RndfLookup *inRl, double curT, CarModel *cm);
  DynamicObstaclePF(DynamicObstaclePF *pf, CarModel *inCarModel);
  ~DynamicObstaclePF();

  void init(RndfLookup *inRl, double curT, CarModel *cm);

  bool full();
  void measurementUpdate();
  void motionUpdate();
  void update(double ts);
  void computeMean();
  
  void computeCenter();
  void addNewCar(DynamicObstacle &newCar);
  void computeNewCars();
  void initialize();
  double maxScore();
  void gls_render();
  void gls_render_all();

  bool nearbyPF(DynamicObstaclePF *pf);
  void merge(DynamicObstaclePF *pf);

};


typedef std::vector< DynamicObstaclePF * > DOPFVec;

class DOPFList {

public:

  DOPFVec    list;
  CarModel  *carModel;
  
  DOPFList();
  ~DOPFList();

  void init(CarModel *inCarModel);
  int size();
  void steal(int i);
  void nuke(int i);
  void clear();
  void add(DynamicObstaclePF *p);
  void addCopy(DynamicObstaclePF *p);
  DynamicObstaclePF *get(int i);
  DynamicObstaclePF *pop();
  bool nearbyPF(DynamicObstaclePF *pf);
  bool mergeToPriorPf(int num);
};


typedef std::vector< DynamicObstacle * > ObstacleList;

class Tracker {

public:
  RndfLookup         *rl;
  DynamicObstacle    *junior, *modelCar, *tmpCar;
  Vec2                robotPos;
  double              robotYaw;
  double              ts;
  double              lastTs;
  double              lastVel;
  double              dt;
  CarModel           *carModel;
  bool                trackerInitialized;
  DOPFList            babyPF;
  DOPFList            maturePF;
  DOPFList            intermediatePF;
  SIR                *sir;
  bool                doGlobalSearch;
  bool                doTracking;
  DynObsMsgList       dynObsList;
  int                 maxDynObsNum, dynObsNum;
  double              avgTs;
  ObstacleList        staticObsList;
  int                 newStaticObsStart;
  Vec3                vantagePoint; //origin of laser scan
  AngleGrid          *ag, *agPrev;
  bool                debug;
  Vec2                locOffset;
  int                 maxPtsPerObs;
  
  Tracker(RndfLookup *inRl);
  ~Tracker();

  void set_robot_pose(Vec3 &pos, Vec3 &ori, 
		      double curTs, Vec2 &inLO);
  void update(double inAvgTs);
  DynObsMsgList *getDynObsList();
  void generateDynObsList();
  void markDynamicObsOnAG();
  void markObsOnAG();
  void generateStaticObstacles();
  void addPointToObs(Vec2 &pt, DynamicObstacle *obs);
  void addObstaclePoint(Vec2 &pt);
  void globalSearchInner(AngleGrid *agPtr);
  double sampleYaw(double roadYaw);
  void globalSearch();
  DynamicObstaclePF *getNewPF(Vec2 &loc);
  bool nearExistingPF(Vec2 &loc);
  bool   observedByThisSensor(DynamicObstaclePF *pf);
  void   updateTracker();
  void   gls_render(int displayAllCars, int displayInter);
};

#endif

