#include <roadrunner.h>
#include <ipc_interface.h>
#include <planner_messages.h>
#include <logio.h>

namespace dgc {

char *StringV3ToPlannerTrajectory(char *string, PlannerTrajectory *trajectory)
{
  char *pos = string;
  int i, num_waypoints;

  num_waypoints = READ_INT(&pos);
  if(num_waypoints != trajectory->num_waypoints) {
    trajectory->num_waypoints = num_waypoints;
    if (trajectory->num_waypoints>0) {
      trajectory->waypoint =
	(PlannerWaypoint *)realloc(trajectory->waypoint, 
				   trajectory->num_waypoints *
				   sizeof(PlannerWaypoint));
      if(trajectory->num_waypoints > 0)
	dgc_test_alloc(trajectory->waypoint);
    }
  }
  for(i = 0; i < trajectory->num_waypoints; i++) {
    trajectory->waypoint[i].x = READ_DOUBLE(&pos);
    trajectory->waypoint[i].y = READ_DOUBLE(&pos);
    trajectory->waypoint[i].theta = READ_FLOAT(&pos);
    trajectory->waypoint[i].v = READ_FLOAT(&pos);
    trajectory->waypoint[i].yaw_rate = READ_FLOAT(&pos);
  }
  trajectory->reverse = READ_INT(&pos);
  trajectory->timestamp = READ_DOUBLE(&pos);
  READ_HOST(trajectory->host, &pos);
  return pos;
}

char *StringToPlannerFulltraj(char *string, PlannerFulltraj *trajectory)
{
  char *pos = string;
  int i, num_waypoints;

  num_waypoints = READ_INT(&pos);
  if(num_waypoints != trajectory->num_waypoints) {
    trajectory->num_waypoints = num_waypoints;
    trajectory->waypoint =
      (PlannerWaypoint *)realloc(trajectory->waypoint, 
				 trajectory->num_waypoints *
				 sizeof(PlannerWaypoint));
    if(trajectory->num_waypoints > 0)
      dgc_test_alloc(trajectory->waypoint);
  }
  for(i = 0; i < trajectory->num_waypoints; i++) {
    trajectory->waypoint[i].x = READ_DOUBLE(&pos);
    trajectory->waypoint[i].y = READ_DOUBLE(&pos);
    trajectory->waypoint[i].theta = READ_FLOAT(&pos);
    trajectory->waypoint[i].v = READ_FLOAT(&pos);
    trajectory->waypoint[i].yaw_rate = READ_FLOAT(&pos);
  }
  trajectory->timestamp = READ_DOUBLE(&pos);
  READ_HOST(trajectory->host, &pos);
  return pos;
}

char *StringToPlannerFsmState(char *string, PlannerFsmState *fsmstate)
{
  char *pos = string;

  fsmstate->state = READ_INT(&pos);
  fsmstate->timestamp = READ_DOUBLE(&pos);
  READ_HOST(fsmstate->host, &pos);
  return pos;
}

char *StringToPlannerMdfGoal(char *string, PlannerMdfGoal *mdf_goal)
{
  char *pos = string;

  mdf_goal->goal_s = READ_INT(&pos);
  mdf_goal->goal_l = READ_INT(&pos);
  mdf_goal->goal_w = READ_INT(&pos);
  mdf_goal->current_goal_num = READ_INT(&pos);
  mdf_goal->num_goals = READ_INT(&pos);
  mdf_goal->checkpoint_num = READ_INT(&pos);
  mdf_goal->timestamp = READ_DOUBLE(&pos);
  READ_HOST(mdf_goal->host, &pos);
  return pos;
}

void PlannerAddLogReaderCallbacks(LogReaderCallbackList *callbacks)
{
  callbacks->AddCallback("PLANNER_FULLTRAJ", PlannerFulltrajID, 
			 (LogConverterFunc)StringToPlannerFulltraj, 
			 sizeof(PlannerFulltraj), 1);
  callbacks->AddCallback("PLANNER_TRAJ3", PlannerTrajectoryID, 
			 (LogConverterFunc)StringV3ToPlannerTrajectory, 
			 sizeof(PlannerTrajectory), 1);
  callbacks->AddCallback("PLANNER_STATE", PlannerFsmStateID,
			 (LogConverterFunc)StringToPlannerFsmState, 
			 sizeof(PlannerFsmState), 1);
  callbacks->AddCallback("PLANNER_MDFGOAL", PlannerMdfGoalID,
			 (LogConverterFunc)StringToPlannerMdfGoal, 
			 sizeof(PlannerMdfGoal), 1);
}

void PlannerTrajectoryWrite(PlannerTrajectory *trajectory,
			    double logger_timestamp, dgc_FILE *outfile)
{
  int i;

  dgc_fprintf(outfile, "PLANNER_TRAJ3 %d ",
             trajectory->num_waypoints);
  for(i = 0; i < trajectory->num_waypoints; i++)
    dgc_fprintf(outfile, "%.4f %.4f %f %.2f %f ", 
               trajectory->waypoint[i].x,
               trajectory->waypoint[i].y,
               trajectory->waypoint[i].theta,
               trajectory->waypoint[i].v,
               trajectory->waypoint[i].yaw_rate);
  dgc_fprintf(outfile, "%d %f %s %f\n", trajectory->reverse, 
	     trajectory->timestamp, trajectory->host, 
             logger_timestamp);
}

void PlannerFulltrajWrite(PlannerFulltraj *trajectory, double logger_timestamp,
			  dgc_FILE *outfile)
{
  int i;

  dgc_fprintf(outfile, "PLANNER_FULLTRAJ %d ",
             trajectory->num_waypoints);
  for(i = 0; i < trajectory->num_waypoints; i++)
    dgc_fprintf(outfile, "%.4f %.4f %f %.2f %f ", 
               trajectory->waypoint[i].x,
               trajectory->waypoint[i].y,
               trajectory->waypoint[i].theta,
               trajectory->waypoint[i].v,
               trajectory->waypoint[i].yaw_rate);
  dgc_fprintf(outfile, "%f %s %f\n", 
	     trajectory->timestamp, trajectory->host, 
             logger_timestamp);
}

void PlannerFsmStateWrite(PlannerFsmState *fsmstate, double logger_timestamp,
			  dgc_FILE *outfile)
{
  dgc_fprintf(outfile, "PLANNER_STATE %d ", fsmstate->state);
  dgc_fprintf(outfile, "%f %s %f\n", fsmstate->timestamp, fsmstate->host, 
             logger_timestamp);
}

void PlannerStatusWrite(PlannerStatus *status, double logger_timestamp,
			dgc_FILE *outfile)
{
  int i;

  dgc_fprintf(outfile, "PLANNER_STATUS2 %d %d %d ", status->fsm_state,
	     status->mdf_goal_num, status->num_bts);
  for(i = 0; i < status->num_bts; i++) 
    dgc_fprintf(outfile, "%f %f %f %d %d %d ", 
	       status->bt_info[i].local_score,
	       status->bt_info[i].global_score,
	       status->bt_info[i].obstacle_dist,
	       status->bt_info[i].illegal,
	       status->bt_info[i].too_fast,
	       status->bt_info[i].lane_change);
  dgc_fprintf(outfile, "%d %f %f %f %f %f %d ", status->chosen_bt, 
 	     status->best_path_static_undercar_sum,
	     status->best_path_static_nearcar_sum,
 	     status->best_path_undercar_sum,
	     status->best_path_nearcar_sum,
	     status->best_path_obstacle_dist,
	     status->best_path_obstacle_dist_i);
  dgc_fprintf(outfile, "%f %f ", status->int_static_points, 
	     status->int_moving_points);
	     
  dgc_fprintf(outfile, "%f %s %f\n", status->timestamp, status->host, 
             logger_timestamp);
}

void PlannerMdfGoalWrite(PlannerMdfGoal *mdf_goal, double logger_timestamp,
			 dgc_FILE *outfile)
{
  dgc_fprintf(outfile, "PLANNER_MDFGOAL %d %d %d %d %d %d %f %s %f\n",
	     mdf_goal->goal_s, mdf_goal->goal_l, mdf_goal->goal_w, 
	     mdf_goal->current_goal_num, mdf_goal->num_goals,
	     mdf_goal->checkpoint_num,
	     mdf_goal->timestamp, mdf_goal->host, logger_timestamp);
}

void PlannerAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
				  dgc_FILE *logfile,
				  dgc_subscribe_t subscribe_how)
{
  ipc->AddLogHandler(PlannerFulltrajID, NULL, sizeof(PlannerFulltraj),
		     (dgc_log_handler_t)PlannerFulltrajWrite,
		     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(PlannerTrajectoryID, NULL, sizeof(PlannerTrajectory),
		     (dgc_log_handler_t)PlannerTrajectoryWrite,
		     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(PlannerFsmStateID, NULL, sizeof(PlannerFsmState),
		     (dgc_log_handler_t)PlannerFsmStateWrite,
		     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(PlannerStatusID, NULL, sizeof(PlannerStatus),
		     (dgc_log_handler_t)PlannerStatusWrite,
		     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(PlannerMdfGoalID, NULL, sizeof(PlannerMdfGoal),
		     (dgc_log_handler_t)PlannerMdfGoalWrite,
		     start_time, logfile, subscribe_how);
}

void PlannerTriggerCommand(IpcInterface *ipc, int command)
{
  static PlannerTrigger msg;
  static int first = 1;
  int err;

  if (first) {
    strcpy(msg.host, dgc_hostname());
    err = ipc->DefineMessage(PlannerTriggerID);
    TestIpcExit(err, "Could not define message", PlannerTriggerID);
    first = 0;
  }
  msg.command = command;
  msg.timestamp = dgc_get_time();
  err = ipc->Publish(PlannerTriggerID, &msg);
  TestIpc(err, "Could not publish", PlannerTriggerID);
}

void RequestPlannerFsm(IpcInterface *ipc)
{
  static PlannerFsmRequest msg;
  static int first = 1;
  int err;

  if (first) {
    strcpy(msg.host, dgc_hostname());
    err = ipc->DefineMessage(PlannerFsmRequestID);
    TestIpcExit(err, "Could not define message", PlannerFsmRequestID);
    first = 0;
  }
  msg.timestamp = dgc_get_time();

  err = ipc->Publish(PlannerFsmRequestID, &msg);
  TestIpc(err, "Could not publish", PlannerFsmRequestID);
}

void PlannerGoalCommand(IpcInterface *ipc, double lat, double lon, double theta)
{
  static PlannerGoal msg;
  static int first = 1;
  int err;

  if (first) {
    strcpy(msg.host, dgc_hostname());
    err = ipc->DefineMessage(PlannerGoalID);
    TestIpcExit(err, "Could not define message", PlannerGoalID);
    first = 0;
  }
  msg.goal_lat = lat;
  msg.goal_lon = lon;
  msg.goal_theta = theta;
  msg.timestamp = dgc_get_time();

  err = ipc->Publish(PlannerGoalID, &msg);
  TestIpc(err, "Could not publish", PlannerGoalID);
}

}
