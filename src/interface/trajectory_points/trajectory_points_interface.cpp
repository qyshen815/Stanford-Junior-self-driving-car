#include <roadrunner.h>
#include <ipc_interface.h>
#include <logio.h>

#include "trajectory_points_interface.h"

using namespace dgc;

namespace vlr {

/*****************************************************************************
 *
 *   LOGGER FUNCTIONS
 *
 *****************************************************************************/
char* StringToTrajectoryPoints(char* string, TrajectoryPoints2D* trj_points) {
  char* pos = string;

  int num_points = READ_INT(&pos);
  if(num_points != trj_points->num_points) {
    trj_points->num_points = num_points;
    trj_points->points = (TrajectoryPoint2D*)realloc(trj_points->points, num_points * sizeof(TrajectoryPoint2D));
    dgc_test_alloc(trj_points->points);
  }

  for(int i=0; i<trj_points->num_points; i++) {
    trj_points->points[i].t = READ_DOUBLE(&pos);
    trj_points->points[i].x = READ_DOUBLE(&pos);
    trj_points->points[i].y = READ_DOUBLE(&pos);
    trj_points->points[i].theta = READ_DOUBLE(&pos);
    trj_points->points[i].kappa = READ_DOUBLE(&pos);
    trj_points->points[i].kappa_dot = READ_DOUBLE(&pos);
    trj_points->points[i].v = READ_DOUBLE(&pos);
    trj_points->points[i].a = READ_DOUBLE(&pos);
    trj_points->points[i].jerk = READ_DOUBLE(&pos);
    trj_points->points[i].delta_theta = READ_DOUBLE(&pos);
    trj_points->points[i].d = READ_DOUBLE(&pos);
    trj_points->points[i].a_lat = READ_DOUBLE(&pos);
  }
  READ_HOST(trj_points->host, &pos);
  return pos;
}

void TrajectoryPointsAddLogReaderCallbacks(LogReaderCallbackList* callbacks) {
  callbacks->AddCallback("TRAJECTORYPOINTS2D", TrajectoryPoints2DID, (LogConverterFunc)StringToTrajectoryPoints, sizeof(TrajectoryPoints2D), 0);
}

void TrajectoryPointsWrite(TrajectoryPoints2D* msg, double logger_timestamp, dgc_FILE* outfile) {
  dgc_fprintf(outfile, "TRAJECTORYPOINTS2D %d ", msg->num_points);
  for(int i=0; i<msg->num_points; i++) {
    dgc_fprintf(outfile, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf ",
                               msg->points[i].t, msg->points[i].x, msg->points[i].y, msg->points[i].theta,
                               msg->points[i].kappa, msg->points[i].kappa_dot, msg->points[i].v, msg->points[i].a,
                               msg->points[i].jerk, msg->points[i].delta_theta, msg->points[i].d, msg->points[i].a_lat);
  }
  dgc_fprintf(outfile, "%s %f\n", msg->host, logger_timestamp);
}

void TrajectoryPointsAddLogWriterCallbacks(IpcInterface* ipc, double start_time, dgc_FILE* logfile, dgc_subscribe_t subscribe_how) {
  ipc->AddLogHandler(TrajectoryPoints2DID, NULL, sizeof(TrajectoryPoints2D), (dgc_log_handler_t) TrajectoryPointsWrite,
      start_time, logfile, subscribe_how);
}

} // namespace vlr
