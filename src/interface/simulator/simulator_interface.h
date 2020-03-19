#ifndef DGC_SIMULATOR_INTERFACE_H
#define DGC_SIMULATOR_INTERFACE_H

#include <logio.h>
#include <ipc_interface.h>
#include <simulator_messages.h>

namespace dgc {

void *StringV1ToSimulatorGroundTruth(char *string, 
				     SimulatorGroundTruth *groundtruth);

void *StringV2ToSimulatorGroundTruth(char *string, 
				     SimulatorGroundTruth *groundtruth);

void *StringToSimulatorTag(char *string, SimulatorTag *tag);

void SimulatorAddLogReaderCallbacks(LogReaderCallbackList *callbacks);

void SimulatorTagWrite(SimulatorTag *tag, double logger_timestamp, 
		       dgc_FILE *outfile);

void SimulatorGroundTruthWrite(SimulatorGroundTruth *groundtruth,
			       double logger_timestamp, dgc_FILE *outfile);

void SimulatorAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
				    dgc_FILE *logfile,
				    dgc_subscribe_t subscribe_how);

}

#endif
