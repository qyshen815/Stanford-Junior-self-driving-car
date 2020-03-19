#ifndef LASERTRACK_INTERFACE_H
#define LASERTRACK_INTERFACE_H

#include <applanix_interface.h>
#include <localize_interface.h>
#include <param_interface.h>
#include <perception_interface.h>

typedef struct {

  int   useGls;
  int   displayVelodyne;
  int   showScans;
  int   showAngleGrid;
  int   displayAllCars;
  int   displayInter;
  int   showObstacles;
  int   showRndfMask;
  int   redrawFlag;

} lasertrack_vars_t;

extern lasertrack_vars_t   lasertrack_vars;
extern dgc::Param          lasertrack_ipc_params[];
extern int                 lasertrack_ipc_params_cnt;

/*
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
*/


typedef struct {

  char             *rndf_filename;
  dgc_transform_t   velodyne_offset;
  int               useVelodyne;

} lasertrack_init_t;


void lasertrack_initialize(lasertrack_init_t *init_params);
void lasertrack_velodyne_handler(dgc::PerceptionScan *scan, 
				 dgc::PerceptionRobotPose *pose);

dgc::PerceptionObstacleList *lasertrack_get_obstacles();

void lasertrack_param_handler(char *module, char *variable, char *value);

void lasertrack_gls_send();
void lasertrack_request_gls_redraw();
void lasertrack_gls_redraw();

#endif
