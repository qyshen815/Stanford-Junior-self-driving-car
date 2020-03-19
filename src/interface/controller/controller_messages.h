#ifndef DGC_CONTROLLER_H
#define DGC_CONTROLLER_H

#include <ipc_interface.h>

namespace dgc {

/** Controller error message */

typedef struct {
  double target_velocity;           /** target velocity, in m/s */
  double target_steering_angle;     /** target steering angle, in degrees */
  double cross_track_error;         /** perpendicular distance to intended trajectory, in meters */
  double heading_error;             /** heading error, in degrees */
  double timestamp;
  char host[10];
} ControllerTarget;

#define   DGC_CONTROLLER_TARGET_NAME      "dgc_controller_target"
#define   DGC_CONTROLLER_TARGET_FMT       "{double,double,double,double,double,[char:10]}"

const IpcMessageID ControllerTargetID = { DGC_CONTROLLER_TARGET_NAME,
					  DGC_CONTROLLER_TARGET_FMT };

typedef struct {
  int override_status;
  double timestamp;
  char host[10];
} ControllerVelocity;

#define   DGC_CONTROLLER_VELOCITY_NAME      "dgc_controller_velocity"
#define   DGC_CONTROLLER_VELOCITY_FMT       "{int,double,[char:10]}"

const IpcMessageID ControllerVelocityID = { DGC_CONTROLLER_VELOCITY_NAME,
					    DGC_CONTROLLER_VELOCITY_FMT };

}

#endif
