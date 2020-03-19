#ifndef DGC_PERCEPTION_INTERFACE_H
#define DGC_PERCEPTION_INTERFACE_H

#include <obstacle_types.h>
#include <perception_messages.h>

namespace vlr {

#define PERCEPTION_SHM_GRID_MAP_KEY          0x67726964

#define PERCEPTION_MAP_OBSTACLE_FREE         0
#define PERCEPTION_MAP_OBSTACLE_LOW          1
#define PERCEPTION_MAP_OBSTACLE_HIGH         2
#define PERCEPTION_MAP_OBSTACLE_DYNAMIC      3
#define PERCEPTION_MAP_OBSTACLE_STREET       4
#define PERCEPTION_MAP_OBSTACLE_UNKNOWN      255

void PerceptionObstaclesWrite(PerceptionObstacles *msg, 
			      double logger_timestamp, dgc_FILE *outfile);

void PerceptionAddLogWriterCallbacks(dgc::IpcInterface* ipc, double start_time,
				     dgc_FILE *logfile,
				     dgc::dgc_subscribe_t subscribe_how);

void PerceptionMapRequestCommand(dgc::IpcInterface* ipc);

void PerceptionMapResetCommand(dgc::IpcInterface* ipc);

void PerceptionConvMapToGridPoints(PerceptionMap *msg, int *num_points,
				   PerceptionMapGrid **pts);

} // namespace vlr

#endif
