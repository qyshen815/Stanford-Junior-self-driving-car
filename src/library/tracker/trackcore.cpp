#include <roadrunner.h>
#include <lltransform.h>
#include <ipc_interface.h>
#include <gls_interface.h>
#include "trackcore.h"
#include "projected_scan.h"

using std::vector;
using namespace dgc;

int point_count   = 0;
int use_count     = 0;

LaserTracker::LaserTracker(RndfLookup *rl, 
			   SensorManager *sensorManager) 
{
  this->sensorManager  = sensorManager;
  this->rl             = rl;
  ptsProcessed         = 0;
  useTracker           = 1;
  groundLevel          = -DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT;
  sendMessages         = false;
}

void 
LaserTracker::drawRndfMask() 
{
  Vec2      bot2=robotPos;
  double    reso=0.5; //grid resolution
  int       radius=100; //meters
  int       steps=(int)(radius/reso); //number of grids

  for(int gx=-steps; gx<steps; gx++) { 
    for(int gy=-steps; gy<steps; gy++) {
      Vec2 pos(gx*reso, gy*reso); 
      Vec2 wPt=pos; wPt.add(bot2);
      if (rl->closeToRoad(wPt)) {
	glsColor3f(lasertrack_gls, 0.7,0.7,0.7);
      } else {
	glsColor3f(lasertrack_gls, 0.3,0.3,0.3);
      }
      draw2DCell(pos, 0.5);
    }
  }
}

void 
LaserTracker::draw2DCell( const Vec2 &pos, double resolution ) 
{
  glsBegin(lasertrack_gls, GLS_QUADS);
  double   zVal = groundLevel - 0.1;
  Vec2     p1 =  pos;
  Vec2     p2 =  p1;
  p2.v[0] += resolution; 
  p2.v[1] += resolution; 
  glsVertex3f(lasertrack_gls, p1.v[0],p1.v[1],zVal);
  glsVertex3f(lasertrack_gls, p2.v[0],p1.v[1],zVal);
  glsVertex3f(lasertrack_gls, p2.v[0],p2.v[1],zVal);
  glsVertex3f(lasertrack_gls, p1.v[0],p2.v[1],zVal);
  glsEnd(lasertrack_gls);
}

void 
LaserTracker::gls_render()
{
  for(int i=0; i<(signed)sensorManager->sensorList.size(); i++) {
    SensorRecord *sr = sensorManager->sensorList[i];
    if (!sr->display) continue;
    if (lasertrack_vars.showScans && sr->scan) sr->scan->gls_render();
    if(lasertrack_vars.showObstacles && useTracker) {
      glsPushMatrix(lasertrack_gls);
      glsTranslatef(lasertrack_gls, 0, 0, groundLevel + 1);
      sr->tracker->gls_render(lasertrack_vars.displayAllCars, 
			      lasertrack_vars.displayInter);
      glsPopMatrix(lasertrack_gls);
    }
  }
  
  if (lasertrack_vars.showRndfMask) drawRndfMask();
}

void 
LaserTracker::update( PerceptionRobotPose *pose, 
		      int sensorId, void *sensorMsg, 
		      lasertrack_flags_t &flags ) 
{
  /* update everything with the latest pose */
  localizeOffset.set(pose->localize.x_offset, pose->localize.y_offset);
  rl->update_localize_offset(localizeOffset);
  robotPos.set(pose->pose.x, pose->pose.y);
  
  /* update cached projected scans */
  SensorRecord *sr = sensorManager->updateSensor(sensorId, pose, sensorMsg);

#ifdef VERBOSE
  cout << endl;
  Timer utm("TOTAL");
#endif  

  /* swap previous and current angle grids, clear marked points
     from previous grid, and empty current grid */
  projected_scan *scan = sr->scan;
  Vec2 vantagePt(scan->laserPos.v[0], scan->laserPos.v[1]);
  AngleGrid *ag = sr->agPrev;
  AngleGrid *agPrev = sr->ag;
  sr->ag = ag;
  sr->agPrev = agPrev;
  agPrev->unmarkClearedPoints();
  ag->clear();
  ag->set_robot_pose(vantagePt, scan->laserOri.v[0], scan->timestamp);
  
  /* update tracker bookkeeping */
  Tracker *tracker = sr->tracker;
  tracker->doTracking = flags.doTracking;
  tracker->doGlobalSearch = flags.doGlobalSearch;
  tracker->vantagePoint = scan->laserPos;
  tracker->ag = ag;
  tracker->agPrev = agPrev;
  if(useTracker) {
    tracker->set_robot_pose(scan->robotPos, scan->robotOri,
			    scan->timestamp, localizeOffset);
    sensorManager->replicateMaturePFs(sensorId);
  }
  
  /* mark points that indicate motion */
  if(true) {
#ifdef VERBOSE
    Timer ptsTm("add-laser-points");
#endif  
    ptsProcessed = 0;
    for(int i = 0; i < scan->num_points; i++)
      if(scan->point[i].use) {
	Vec2 pt(scan->point[i].x, scan->point[i].y);
	if (tracker->junior->externalDist(pt)<1) {
	  scan->point[i].use=0;
	  continue;
	}
	ag->addPoint(pt, scan->point + i);
	ptsProcessed++;
	use_count++;
      }
    point_count += scan->num_points;
    ag->markClearedPoints(agPrev);
    agPrev->markClearedPoints(ag);  
  }
  
  if(useTracker) {
    /* Run the car tracker */
    tracker->update(scan->timestamp);
    
    /* classify the original data points */
    if(true) {
#ifdef VERBOSE
      Timer ptListTm("gen-point-lists");
#endif
      for(int i = 0; i < scan->num_points; i++) 
	if(scan->point[i].use) {
	  Vec2 pt(scan->point[i].x, scan->point[i].y);
	  tracker->addObstaclePoint(pt);
	}
    }
  }
  
  //  fprintf(stderr, "Points %d total %d used   - %d rndf lookups %d true\n", 
  //	  point_count, use_count, rl->rndf_count, rl->true_rndf_count);
  
  /* publish IPC obstacle message */
  //  tracker->generateDynObsList();
  makeCombinedMessage();
  if (sendMessages) publish_output();
}

void 
LaserTracker::makeCombinedMessage() {

  DynObsMsgList lst;
  
  for(int i=0; i<(signed)sensorManager->sensorList.size(); i++) {
    SensorRecord *sr = sensorManager->sensorList[i];
    //add dynamic obs messages
    for(int j=0; j<sr->tracker->maturePF.size(); j++) {
      DynObsMsg msg;
      sr->tracker->maturePF.get(j)->mean->toMsg(&msg);
      lst.push_back(msg);
    }
    //add static obs messages
    for(int j=0; j<(signed)(sr->tracker->staticObsList.size()); j++) {
      bool addThisObs=true;
      Vec2 loc=sr->tracker->staticObsList[j]->origin;
      for(int k=0; k<(signed)sensorManager->sensorList.size(); k++) {
	DOPFList *maturePF = &(sensorManager->sensorList[k]->tracker->maturePF);
	for(int m=0; m< maturePF->size(); m++) {
	  if (maturePF->get(m)->mean->nearbyPoint(loc) != DO_FAR) {
	    addThisObs=false;
	    break;
	  }
	}
	if (!addThisObs) break;
      }
      if (!addThisObs) continue;
      DynObsMsg msg;
      sr->tracker->staticObsList[j]->toMsg(&msg);
      lst.push_back(msg);
    }
  }
  make_msg(lst);
}

void 
LaserTracker::make_msg(DynObsMsgList &lst)
{
  static vector< PerceptionDynamicObstacle > obsList;
  static vector< vector< ObstaclePoint > > ptLists;
  static int first=1;
  if(first) {
    strncpy(msg.host, dgc_hostname(), 10);
    first = 0;
  }

  obsList.clear();
  msg.num_obstacles = (signed)lst.size();
  for(int i=0; i<(signed)lst.size(); i++) {
    PerceptionDynamicObstacle obs = lst[i].msg;
    obsList.push_back(obs);
  }
  msg.obstacle = &(obsList[0]);
}

void 
LaserTracker::publish_output()
{
  IPC_RETURN_TYPE err;
  err = IPC_publishData(DGC_PERCEPTION_OBSTACLELIST_NAME, &msg);
  dgc_test_ipc_exit(err, "Could not publish", 
		    DGC_PERCEPTION_OBSTACLELIST_NAME);
}
