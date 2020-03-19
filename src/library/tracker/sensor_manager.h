#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <roadrunner.h>
#include <passat_constants.h>
#include <applanix_interface.h>
#include <localize_interface.h>
#include <gls_interface.h>
#include <transform.h>

#include "rndf_lookup.h"
#include "angle_grid.h"
#include "projected_scan.h"

enum { SENSOR_VELODYNE };

class SensorRecord {

 public:
  int id;
  int type;
  dgc_transform_t offset;
  AngleGrid *ag, *agPrev;
  projected_scan *scan, *prev_scan;
  RndfLookup *rl;
  Tracker *tracker;
  bool display;  	

  SensorRecord(int id, int type, dgc_transform_t offset, RndfLookup *rl) {
    this->id = id;
    this->type = type;
    this->rl = rl;
    display=true;
    dgc_transform_copy(this->offset, offset);
    ag = new AngleGrid();
    agPrev = new AngleGrid();
    scan = NULL;
    prev_scan = NULL;
    tracker = new Tracker(rl);
  }

  ~SensorRecord() {
    delete ag;
    delete agPrev;
    destroyScan(scan);
    destroyScan(prev_scan);
    delete tracker;
  } 

  void processSensorMsg(dgc::PerceptionRobotPose *pose, void *sensorMsg) {
    destroyScan(prev_scan);
    prev_scan = scan;
    switch(type) {
    case SENSOR_VELODYNE:
      scan = new projected_velodyne(pose, 
				    (dgc::PerceptionScan *)sensorMsg, 
				    offset);
      break;
    default:
      scan = NULL;
      dgc_die("SensorRecord::processSensorMsg: unknown sensor type");
      break;
    }
  }

  void destroyScan(projected_scan *s) {
    if (!s) return;
    switch(type) {
    case SENSOR_VELODYNE:
      delete (projected_velodyne *)s;
      s = NULL;
      break;
    default:
      dgc_die("SensorRecord::destroyScan: unknown sensor type");
      break;
    }
  }
};

class SensorManager {
 public:
  std::vector< SensorRecord * > sensorList;
  RndfLookup *rl;
  
  SensorManager(RndfLookup *rl) {
    this->rl = rl;
  }
  int addSensor(int type, dgc_transform_t offset) {
    int id = sensorList.size();
    SensorRecord *sr = new SensorRecord(id, type, offset, rl);
    sensorList.push_back(sr);
    return id;
  }
  SensorRecord *updateSensor(int id, dgc::PerceptionRobotPose *pose, void *sensorMsg) {
    if (id >= (signed)sensorList.size()) {
      dgc_die("SensorManager::updateSensor: id is out of range");
      return NULL;
    }
    SensorRecord *sr = sensorList[id];
    sr->processSensorMsg(pose, sensorMsg);
    return sr;
  }
  void replicateMaturePFs(int id) {
    SensorRecord *sr = sensorList[id];
#ifdef VERBOSE
    cout<<"Replication for sensor "<<id<<endl;
#endif
    for(int i=0; i<(signed)sensorList.size(); i++) {
      if (i==id) continue;
      SensorRecord *tsr = sensorList[i];
      for(int j=0; j<tsr->tracker->maturePF.size(); j++) {
	DynamicObstaclePF *pf = tsr->tracker->maturePF.get(j);
	if (sr->tracker->maturePF.nearbyPF(pf)) continue;
#ifdef VERBOSE
	cout<<"adding from "<<i<<": "<<pf->mean->origin<<endl;
#endif
	sr->tracker->maturePF.addCopy(pf);
      }
    }
  }
};

#endif
