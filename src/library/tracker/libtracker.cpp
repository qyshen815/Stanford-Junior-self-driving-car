#include <roadrunner.h>
#include <lltransform.h>
#include <transform.h>
#include <rndf.h>
#include <passat_constants.h>
#include <applanix_interface.h>
#include <localize_interface.h>
#include <param_interface.h>
#include <perception_interface.h>
#include <gls_interface.h>
#include <pthread.h>

#include "libtracker.h"
#include "trackcore.h"

using std::cout;
using std::ios_base;
using namespace dgc;

lasertrack_vars_t   lasertrack_vars;
int                 lasertrack_initialized = 0;

Param lasertrack_ipc_params[] = {
  {"lasertrack", "useGls", DGC_PARAM_ONOFF, &lasertrack_vars.useGls, 1, lasertrack_param_handler},
  {"lasertrack", "displayVelodyne", DGC_PARAM_ONOFF, &lasertrack_vars.displayVelodyne, 1, lasertrack_param_handler},
  {"lasertrack", "showScans", DGC_PARAM_ONOFF, &lasertrack_vars.showScans, 1, lasertrack_param_handler},
  {"lasertrack", "showAngleGrid", DGC_PARAM_ONOFF, &lasertrack_vars.showAngleGrid, 1, lasertrack_param_handler},
  {"lasertrack", "displayAllCars", DGC_PARAM_ONOFF, &lasertrack_vars.displayAllCars, 1, lasertrack_param_handler},
  {"lasertrack", "displayInter", DGC_PARAM_ONOFF, &lasertrack_vars.displayInter, 1, lasertrack_param_handler},
  {"lasertrack", "showObstacles", DGC_PARAM_ONOFF, &lasertrack_vars.showObstacles, 1, lasertrack_param_handler},
  {"lasertrack", "showRndfMask", DGC_PARAM_ONOFF, &lasertrack_vars.showRndfMask, 1, lasertrack_param_handler},
};


int                        lasertrack_ipc_params_cnt = sizeof(lasertrack_ipc_params)/sizeof(lasertrack_ipc_params[0]);
static pthread_mutex_t     lasertrack_redraw_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t     lasertrack_gls_mutex = PTHREAD_MUTEX_INITIALIZER;
GlsOverlay   *lasertrack_gls = NULL;
GlsOverlay   *lasertrack_completed_gls = NULL; 


class LasertrackInternal {

public:
  rndf_file                    *rndf;
  LaserTracker                 *laserTracker;
  SensorManager                *sensorManager;
  RndfLookup                   *rl;
  PerceptionRobotPose           robot_pose;
  
  int                           velodyne_id;
  
  lasertrack_flags_t            flags;

  LasertrackInternal( void ) {
    rndf                 = NULL;
    laserTracker         = NULL;
    sensorManager        = NULL;
    rl                   = NULL;
    flags.doTracking     = 1;
    flags.doGlobalSearch = 1;
  }

  void gls_completed() {
    GlsOverlay  *tmp_gls = lasertrack_completed_gls; 
    pthread_mutex_lock(&lasertrack_gls_mutex);
    lasertrack_completed_gls = lasertrack_gls;
    lasertrack_gls           = tmp_gls;
    pthread_mutex_unlock(&lasertrack_gls_mutex);
  }

  void request_gls_redraw() {
    if (!lasertrack_vars.useGls) return;
    pthread_mutex_lock(&lasertrack_redraw_mutex);
    lasertrack_vars.redrawFlag = 1;
    pthread_mutex_unlock(&lasertrack_redraw_mutex);
  }
  
  void gls_redraw() {
    if (!lasertrack_vars.useGls) return;
    pthread_mutex_lock(&lasertrack_redraw_mutex);
    int tmpFlag = lasertrack_vars.redrawFlag;
    lasertrack_vars.redrawFlag = 0;
    pthread_mutex_unlock(&lasertrack_redraw_mutex);
    if (tmpFlag == 0) return;
    gls_clear(lasertrack_gls);
    laserTracker->gls_render();
    gls_completed();
  }
  
  void gls_render() {
    if (!lasertrack_vars.useGls) return;
    lasertrack_gls->coordinates = GLS_SMOOTH_COORDINATES;
    lasertrack_gls->origin_x = robot_pose.pose.x;
    lasertrack_gls->origin_y = robot_pose.pose.y;
    lasertrack_gls->origin_z = robot_pose.pose.z;
    request_gls_redraw();
    gls_redraw();
  }
  
  void velodyne_handler(PerceptionScan *scan, PerceptionRobotPose *pose) {
    robot_pose = *pose;
    laserTracker->update(&robot_pose, velodyne_id, scan, flags);
    gls_render();
  }
  
  void param_handler() {
    if (sensorManager) {
      if (flags.useVelodyne) {
	sensorManager->sensorList[velodyne_id]->display = lasertrack_vars.displayVelodyne;
      }
    }
    request_gls_redraw();
  }
  
  void initialize(lasertrack_init_t *init_params) {
    cout.flags(ios_base::dec | ios_base::fixed);
    rndf = new rndf_file;
    if(rndf->load(init_params->rndf_filename) != 0) 
      dgc_die("Error: Could not read RNDF file %s\n", init_params->rndf_filename);
    lasertrack_completed_gls   = gls_alloc("LASERTRACK");
    lasertrack_gls             = gls_alloc("LASERTRACK");
    lasertrack_vars.redrawFlag = 0;
    
    rl = new RndfLookup(rndf, 4, 6);
    sensorManager = new SensorManager(rl);
    laserTracker = new LaserTracker(rl, sensorManager);
    
    flags.useVelodyne = init_params->useVelodyne;
    if (flags.useVelodyne) {
      velodyne_id = sensorManager->addSensor(SENSOR_VELODYNE, 
					     init_params->velodyne_offset);
      sensorManager->sensorList[velodyne_id]->display = lasertrack_vars.displayVelodyne;
    }
    
    lasertrack_initialized = 1;
  }

} ltrackInternal;


/* external C interface functions */

void 
lasertrack_initialize(lasertrack_init_t *init_params) 
{
  ltrackInternal.initialize(init_params);
}

void 
lasertrack_velodyne_handler(PerceptionScan *scan, PerceptionRobotPose *pose) 
{
  ltrackInternal.velodyne_handler(scan, pose); 
}

PerceptionObstacleList *
lasertrack_get_obstacles() 
{
  return ltrackInternal.laserTracker->get_msg();
}

void 
lasertrack_param_handler( __attribute__ ((unused)) char *module, 
			  __attribute__ ((unused)) char *variable, 
			  __attribute__ ((unused)) char *value) 
{
  ltrackInternal.param_handler();
}

void 
lasertrack_gls_send() 
{
  pthread_mutex_lock(&lasertrack_gls_mutex);
  glsSend(lasertrack_completed_gls);
  pthread_mutex_unlock(&lasertrack_gls_mutex);
}

void 
lasertrack_request_gls_redraw() 
{
  ltrackInternal.request_gls_redraw();
}

void 
lasertrack_gls_redraw() 
{
  //NOTE: only param handler is assumed to run in a different thread than the tracker
  //if sensor handlers run in different threads, need a mutex for gls_redraw
  ltrackInternal.gls_redraw();
}

