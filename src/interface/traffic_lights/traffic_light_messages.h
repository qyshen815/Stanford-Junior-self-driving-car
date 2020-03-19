#ifndef TRAFFIC_LIGHT_MESSAGES_H
#define TRAFFIC_LIGHT_MESSAGES_H

#include <ipc_interface.h>

namespace vlr {

// --- TrafficLightStateMsgID -------------------------------------------------------------------------------------
#define        TRAFFIC_LIGHT_STATE_NAME        "dgc_traffic_light_state"
#define        TRAFFIC_LIGHT_STATE_FMT         "{[char:20],char,char,double,double,double,double,int,int}"

	typedef struct {
		char name[20];                    // traffic light name/id from rndf
		char state;                       // state of main part of light: r=red, g=green, y=yellow, u=unknown
		char state_arrow; 					 // if has a turn arrow, include its state
		                                  // otherwise this is set to 'n' to indicate that it does not have a turn arrow
		double timestamp_rg_switch;       // time of switch from red to green
		double timestamp_gy_switch;       // time of switch from green to red
		double timestamp;                 // should match timestamp of camera image that the state was determined from
		double confidence;                // correlation value from 0 to 1		
		int u;                            // section of the camera image plane where the light is predicted to be located
		int v;
	} TrafficLightState;

	const dgc::IpcMessageID TrafficLightStateMsgID = { TRAFFIC_LIGHT_STATE_NAME, TRAFFIC_LIGHT_STATE_FMT };

	// --- TrafficLightListMsgID ------------------------------------------------------------------------------------
#define        TRAFFIC_LIGHT_LIST_NAME        "dgc_traffic_light_list"
#define        TRAFFIC_LIGHT_LIST_FMT        "{int,<" TRAFFIC_LIGHT_STATE_FMT ":1>}"

	typedef struct {
        int num_light_states;
        TrafficLightState* light_state;
    } TrafficLightList;

	const dgc::IpcMessageID TrafficLightListMsgID = { TRAFFIC_LIGHT_LIST_NAME, TRAFFIC_LIGHT_LIST_FMT };

  // --- TrafficLightPoseMsgID --------------------------------------------------------------------------------

	typedef struct {
    char name[20];             // traffic light name/id from rndf
    double lat;                // latitude
    double lon;                // longitude
    double z;                  // height above ground
    double orientation;        // orientation around z axis
  } TrafficLightPose;

  #define        TRAFFIC_LIGHT_POSE_NAME        "traffic_light_pose"
  #define        TRAFFIC_LIGHT_POSE_FMT        "{[char:20],[double:4]}"

  const dgc::IpcMessageID TrafficLightPoseMsgID = { TRAFFIC_LIGHT_POSE_NAME, TRAFFIC_LIGHT_POSE_FMT };

  // --- TrafficLightPoseListMsgID ----------------------------------------------------------------------------

  typedef struct {
        int num_traffic_lights;
        TrafficLightPose* light_pose;
    } TrafficLightPoseList;

  #define        TRAFFIC_LIGHT_POSE_LIST_NAME        "traffic_light_pose_list"
  #define        TRAFFIC_LIGHT_POSE_LIST_FMT        "{int,<" TRAFFIC_LIGHT_POSE_FMT ":1>}"

  const dgc::IpcMessageID TrafficLightPoseListMsgID = { TRAFFIC_LIGHT_POSE_LIST_NAME, TRAFFIC_LIGHT_POSE_LIST_FMT };

  // --- TrafficLightDebugMsgID -----------------------------------------------------------------------------------
  #define 		MAX_GRID_CELLS 255
  #define 		TRAFFIC_LIGHT_DEBUG_NAME					"dgc_traffic_light_debug"
  #define 		TRAFFIC_LIGHT_DEBUG_FMT						"{int,[int:255],[int:255],[char:255],[int:2],[int:2],[int:2],int}"

	typedef struct {
		int traffic_light_id;
		int grid_cell_u[MAX_GRID_CELLS];
		int grid_cell_v[MAX_GRID_CELLS];
		char grid_cell_state[MAX_GRID_CELLS];
		int or_vect[2];
		int z_vect[2];
		int cross_vect[2];		
		int num_cells_used;
	} TrafficLightDebug;
	
	const dgc::IpcMessageID TrafficLightDebugMsgID = {	TRAFFIC_LIGHT_DEBUG_NAME, TRAFFIC_LIGHT_DEBUG_FMT };

} // end namespace vlr

#endif
