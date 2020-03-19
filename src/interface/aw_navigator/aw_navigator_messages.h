/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_NAVIGATOR_MESSAGES_H
#define AW_NAVIGATOR_MESSAGES_H

#include <global.h>
#include <ipc_interface.h>
#include <trajectory_points_interface.h>

namespace vlr {

typedef enum {
  UC_NAVI_DRIVE_UNDEFINED,
  UC_NAVI_DRIVE_STOP_DEST,
  UC_NAVI_DRIVE_STOP_WAIT,
  UC_NAVI_DRIVE_FAR,
  UC_NAVI_DRIVE_CLOSE
} uc_navi_drive_states;

typedef enum {
  UC_NAVI_SEARCH_UNDEFINED,
  UC_NAVI_SEARCH_NEW,
  UC_NAVI_SEARCHING,
  UC_NAVI_SOLUTION,
  UC_NAVI_SOLUTION_NONE,
  UC_NAVI_SUPERVISE,
  UC_NAVI_BLOCKADE_SEARCHING,
  UC_NAVI_BLOCKADE_YES,
  UC_NAVI_BLOCKADE_NO
} uc_navi_search_states;

typedef struct {
  uc_navi_drive_states drive_state;
  uc_navi_search_states search_state;

  TrajectoryPoints2D points;
} navigator_feedback_t;

#define                AW_NAVIGATOR_FEEDBACK_NAME       "aw_navigator_feedback"
#define                AW_NAVIGATOR_FEEDBACK_FMT        "{int,int," TRAJECTORY_POINTS2D_FMT "}"

const dgc::IpcMessageID AWNavigatorFeedBackID = {AW_NAVIGATOR_FEEDBACK_NAME, AW_NAVIGATOR_FEEDBACK_FMT};


typedef enum {
  UC_NAVI_IDLE,
  UC_NAVI_INITIALIZE,
  UC_NAVI_KTURN,
  UC_NAVI_PARKING,
  UC_NAVI_STRAIGHT_FWD,
  UC_NAVI_BLOCK_TEST
} uc_navi_mode;

typedef struct {
  double x; // utm
  double y; // utm
  double psi; // rad
  int in;
  char offroad; // flag to enable offroad params
  uc_navi_mode mode;
} kogmo_rtdb_obj_uc_navigator_control_t;

#define                AW_NAVIGATOR_CONTROL_NAME       "aw_navigator_control"
#define                AW_NAVIGATOR_CONTROL_FMT        "{double,double,double,int,char,int}"

const dgc::IpcMessageID AWNavigatorControlID = {AW_NAVIGATOR_CONTROL_NAME, AW_NAVIGATOR_CONTROL_FMT};

static const unsigned int NAVIGATOR_PERIMETER_MAX_POINTS = 500;

  struct navPoint {
    double x;
    double y;
  };

struct NavigatorPerimeter {
  unsigned int num_points;

  char entry_present;
  navPoint entry;

  char exit_present;
  navPoint exit;

  navPoint points[NAVIGATOR_PERIMETER_MAX_POINTS];
};

#define                AW_NAVIGATOR_PERIMETER_NAME       "aw_navigator_perimeter"
#define                AW_NAVIGATOR_PERIMETER_FMT        "{int,char,[double:2],char,[double:2],[{double,double}:500]}"

const dgc::IpcMessageID AWNavigatorPerimeterID = {AW_NAVIGATOR_PERIMETER_NAME, AW_NAVIGATOR_PERIMETER_FMT};

//class RtdbPerimeter {
// public:
//  struct Point {
//    double x;
//    double y;
//  };
//
//  static const unsigned int MAX_POINTS = 5000;
//
//  RtdbPerimeter(): no_points(0), entry_present( false ), exit_present( false ) {}
//
//  int size() { return sizeof( RtdbPerimeter ) - sizeof( points ) + no_points * sizeof( Point ); }
//
//  int no_points;
//
//  bool entry_present;
//  Point entry;
//
//  bool exit_present;
//  Point exit;
//
//  Point points[MAX_POINTS];
//};

} // namespace vlr

#endif
