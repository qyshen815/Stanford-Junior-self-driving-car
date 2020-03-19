#ifndef DGC_PLANNER_INTERFACE_H
#define DGC_PLANNER_INTERFACE_H

#include <logio.h>
#include <ipc_interface.h>
#include <planner_messages.h>

namespace dgc {

char *StringV3ToPlannerTrajectory(char *string, PlannerTrajectory *trajectory);

char *StringToPlannerFulltraj(char *string, PlannerFulltraj *trajectory);

char *StringToPlannerFsmState(char *string, PlannerFsmState *fsmstate);

char *StringToPlannerMdfGoal(char *string, PlannerMdfGoal *mdf_goal);

void PlannerAddLogReaderCallbacks(LogReaderCallbackList *callbacks);

void PlannerTrajectoryWrite(PlannerTrajectory *trajectory, 
			    double logger_timestamp, dgc_FILE *outfile);

void PlannerFulltrajWrite(PlannerFulltraj *trajectory, double logger_timestamp,
			  dgc_FILE *outfile);

void PlannerFsmStateWrite(PlannerFsmState *fsmstate, double logger_timestamp,
			  dgc_FILE *outfile);

void PlannerStatusWrite(PlannerStatus *status, double logger_timestamp,
			dgc_FILE *outfile);

void PlannerMdfGoalWrite(PlannerMdfGoal *mdf_goal, double logger_timestamp,
			 dgc_FILE *outfile);

void PlannerAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
				  dgc_FILE *logfile,
				  dgc_subscribe_t subscribe_how);

void PlannerTriggerCommand(IpcInterface *ipc, int command);

void RequestPlannerFsm(IpcInterface *ipc);

void PlannerGoalCommand(IpcInterface *ipc, double lat, double lon, 
			double theta);

}

#endif
