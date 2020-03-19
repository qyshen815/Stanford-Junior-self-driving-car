#include "view.h"

using namespace vlr;

int  fullscreen = 0;
int  o_width    = 0;
int  o_height   = 0;

typedef struct {
  int    w;
  int    h;
  char   name[40];
} window_size_t;

int  wsize = 1;
#define  NUM_SIZES  10
window_size_t   sizes[NUM_SIZES] = {
    { 640,   480,  "[640 x 480]  VGA" },
    { 800,   600,  "[800 x 600]  SVGA" },
    { 1024,  768,  "[1024 x 768]  XGA" },
    { 1280, 1024,  "[1280 x 1024]  SXGA" },
    { 1400, 1050,  "[1400 x 1050]  SXGA+" },
    { 720,   480,  "[720 x 480]  DVD NTSC" },
    { 720,   576,  "[720 x 578]  DVD PAL" },
    { 1280,  768,  "[1280 x 720] HD720p" },
    { 1280,  960,  "[1280 x 960]  QVGA" },
    { 1366,  768,  "[1366 x 768]  WXGA" }
};

#define CAMERA_NUM_ROT_ANGLES  8
float camera_rot_angles[CAMERA_NUM_ROT_ANGLES] =
{ 180.0, 225.0, 270.0, 315.0, 0.0, 45.0, 90.0, 135.0 };
int  camera_rot_angle_index = 0;
float camera_rot_angle = camera_rot_angles[camera_rot_angle_index];

void
keyboard( unsigned char key,
    __attribute__ ((unused)) int x,
    __attribute__ ((unused)) int y )
{
  char    zone[5] = "10S ";
  double  utm_x, utm_y;
  int  n, i;
  if (show_gls_select) {
    if (key==9) {
      show_gls_select = 1 - show_gls_select;
    } else {
      n = (signed)gls_cache.size();
      if (n>MAX_GLS_SOURCES)
        n = MAX_GLS_SOURCES;
      switch(key) {
      case '0':
        for (i=0; i<MAX_GLS_SOURCES; i++) {
          gls_enable[i] = 0;
        }
        break;
      case '1':
        for (i=0; i<MAX_GLS_SOURCES; i++) {
          gls_enable[i] = 1;
        }
        break;
      default:
        if (key>=97 && key<=97+n) {
          gls_enable[key-97] = 1 - gls_enable[key-97];
        }
        break;
      }
    }
  } else {

    switch(key) {


    /* -------- CTRL KEY VALUES -----------

	 Char	Oct	Dec	Hex	Control-Key	Control Action

	SOH	1	1	1	^A	Start of heading, = console interrupt
	STX	2	2	2	^B	Start of text, maintenance mode on HP console
	ETX	3	3	3	^C	End of text
	EOT	4	4	4	^D	End of transmission, not the same as ETB
	ENQ	5	5	5	^E	Enquiry, goes with ACK; old HP flow control
	ACK	6	6	6	^F	Acknowledge, clears ENQ logon hand
	BEL	7	7	7	^G	Bell, rings the bell...
	BS	10	8	8	^H	Backspace, works on HP terminals/computers
	HT	11	9	9	^I	Horizontal tab, move to next tab stop
	LF	12	10	a	^J	Line Feed
	VT	13	11	b	^K	Vertical tab
	FF	14	12	c	^L	Form Feed, page eject
	CR	15	13	d	^M	Carriage Return
	SO	16	14	e	^N	Shift Out, alternate character set
	SI	17	15	f	^O	Shift In, resume defaultn character set
	DLE	20	16	10	^P	Data link escape
	DC1	21	17	11	^Q	XON, with XOFF to pause listings; ":okay to send".
	DC2	22	18	12	^R	Device control 2, block-mode flow control
	DC3	23	19	13	^S	XOFF, with XON is TERM=18 flow control
	DC4	24	20	14	^T	Device control 4
	NAK	25	21	15	^U	Negative acknowledge
	SYN	26	22	16	^V	Synchronous idle
	ETB	27	23	17	^W	End transmission block, not the same as EOT
	CAN	30	24	17	^X	Cancel line, MPE echoes !!!
	EM	31	25	19	^Y	End of medium, Control-Y interrupt
	SUB	32	26	1a	^Z	Substitute
     */

    case 1:  /* CTRL+A */
      if (show_sensor_info)
        show_sensor_info = 0;
      else
        show_sensor_info = 2;
      draw_msg( show_sensor_info==2?"show applanix info: ON":"show applanix info: OFF" );
      break;
    case 2:  /* CTRL+B */
      if (goal_place_active == GOAL_ALL_SET) {
        action = ACTION_SEND_GOAL;
        latLongToUtm( goal_place.lat, goal_place.lon, &utm_x, &utm_y, zone );
        fprintf( stderr, "# INFO: send goal to planner:\n"
            "# INFO:      (lat/lon/theta): %f %f %f\n"
            "# INFO:      (utm_x/utm_y/theta): %f %f %f\n",
            goal_place.lat, goal_place.lon, goal_place.theta,
            utm_x, utm_y, goal_place.theta );
        draw_msg( "send goal (%f %f) to driving planner",
            goal_place.lat, goal_place.lon );
      } else {
        draw_msg( "set goal place first !" );
      }
      break;
    case 3:  /* CTRL+C */
      if (show_colors)
        show_colors = 0;
      else
        show_colors = 2;
      draw_msg( show_colors?"colored by laser height: ON":"colored by laser height: OFF" );
      break;
    case 4:  /* CTRL+D */
      show_estop = 1 - show_estop;
      draw_msg( show_estop?"show estop: ON":"show estop: OFF" );
      break;
    case 5:  /* CTRL+E */
      camera_limit = 1 - camera_limit;
      draw_msg( camera_limit?"camera limits: ON":"camera limits: OFF" );
      gui3D_setCameraLimit( camera_limit );
      break;
    case 6:  /* CTRL+F */
      show_cte = 1 - show_cte;
      draw_msg( show_cte?"show cross track error: ON":"show cross track error: OFF" );
      break;
    case 7:  /* CTRL+G */
      if (!show_planner_goal)
        show_planner_goal = 1;
      switch (goal_place_active) {
      case GOAL_NOT_SET: case GOAL_ALL_SET:
        goal_place.theta = applanix_current_pose()->yaw;
        goal_place_active = GOAL_DO_SET;
        draw_msg( "set place of goal with the mouse and press CTRL+G again" );
        break;
      case GOAL_DO_SET:
        goal_place_active = GOAL_XY_SET;
        draw_msg( "set orientation of goal with the mouse and press CTRL+G again" );
        break;
      case GOAL_XY_SET:
        goal_place_active = GOAL_ALL_SET;
        draw_msg( "goal place is marked" );
        break;
      default:
        goal_place_active = GOAL_NOT_SET;
        break;
      }
      break;
      case 10:  /* CTRL+J */
        show_canact = 1 - show_canact;
        draw_msg( show_canact?"show can actuator data: ON":"show can actuator: OFF" );
        break;
      case 12:  /* CTRL+L */
        show_localize = 1 - show_localize;
        draw_msg( show_localize?"show localize: ON":"show localize: OFF" );
        break;
      case 13:  /* CTRL+M */
        show_stickers = 1 - show_stickers;
        draw_msg( show_stickers?"show stickered model: ON":"show stickered model: OFF" );
        break;
      case 15:  /* CTRL+O */
        show_coordinates = 1 - show_coordinates;
        draw_msg( show_coordinates?"show coordinates: ON":"show coordinates: OFF" );
        break;
      case 16:  /* CTRL+P */
        action = ACTION_ESTOP_PAUSE;
        draw_msg( "car mode: PAUSE" );
        break;
      case 18:  /* CTRL+R */
        action = ACTION_ESTOP_RUN;
        draw_msg( "car mode: RUN" );
        break;
      case 19:  /* CTRL+S */
        action = ACTION_ESTOP_PAUSE;
        draw_msg( "car mode: PAUSE" );
        break;
      case 21:  /* CTRL+U */
        show_clock = 1-show_clock;
        draw_msg( show_clock?"clock: ON":"clock: OFF" );
        break;
      case 22:  /* CTRL+V */
        show_gmeter = 1 - show_gmeter;
        draw_msg( show_gmeter?"show g-meter: ON":"show g-meter: OFF" );
        break;
      case 23:  /* CTRL+W */
        show_all_radars = 1 - show_all_radars;
        draw_msg( show_all_radars?"filter radar data: OFF":"filter radar data: ON" );
        break;
      case 25:  /* CTRL+Y */
        draw_msg( "sync ghost car" );
        action = ACTION_GHOST_CAR_SYNC;
        break;
      case 8:
        /* BACKSPACE */
        if (!follow_mode) {
          camera_lock = 1-camera_lock;
          draw_msg( camera_lock?"camera lock: ON":"camera lock: OFF" );
        } else {
          draw_msg( "camera lock not possibe - disable follow mode first" );
        }
        break;
      case 9: /* TAB */
        if (show_gls_select || show_gls) {
          show_gls_select = 1 - show_gls_select;
        } else if (show_simulator) {
          show_car_nr = (show_car_nr+1) % (simulator_groundtruth.num_vehicles);
          draw_msg( "show car %d", show_car_nr+1 );
        } else {
          draw_msg( "ERROR: activate GLS first" );
        }
        break;
      case '|':
        wsize = (wsize+1)%NUM_SIZES;
#ifdef USE_QT
        qgui->resize(sizes[wsize].w,sizes[wsize].h);
#else
        glutReshapeWindow( sizes[wsize].w,sizes[wsize].h );
#endif
        draw_msg( sizes[wsize].name );
        break;
      case '0': case ')':
        show_inverse = 1 - show_inverse;
        draw_msg( show_inverse?"inverse: ON":"inverse: OFF" );
        show_sensor_info_layout =
            (show_sensor_info_layout & 199) | (show_inverse<<3);
        break;
      case '3':
        show_ldlrs1 = 1 - show_ldlrs1;
        draw_msg( show_ldlrs1?"ld-lrs 1: ON":"ld-lrs 1: OFF" );
        action = ACTION_SUBSCRIBE;
        break;
      case '4':
        show_ldlrs2 = 1 - show_ldlrs2;
        draw_msg( show_ldlrs2?"ld-lrs 2: ON":"ld-lrs 2: OFF" );
        action = ACTION_SUBSCRIBE;
        break;
      case '#':
        show_lms1 = 1 - show_lms1;
        draw_msg( show_lms1?"lms 1: ON":"lms 1: OFF" );
        action = ACTION_SUBSCRIBE;
        break;
      case '$':
        show_lms3 = 1 - show_lms3;
        draw_msg( show_lms3?"lms 3: ON":"lms 3: OFF" );
        action = ACTION_SUBSCRIBE;
        break;
      case '6':
        show_all_sensors = 1 - show_all_sensors;
        if (show_all_sensors) {
          show_ldlrs1   = 1;
          show_ldlrs2   = 1;
          show_lms1     = 1;
          show_lms3     = 1;
          show_velodyne = 1;
          draw_msg( "all sensors: ON" );
        } else {
          show_ldlrs1   = 0;
          show_ldlrs2   = 0;
          show_lms1     = 0;
          show_lms3     = 0;
          show_velodyne = 0;
          draw_msg( "all sensors: OFF" );
        }

        action = ACTION_SUBSCRIBE;

        break;
      case '8':
        draw_msg( "read transformations" );
        action = ACTION_READ_TRANSFORMATIONS;
        break;
      case '9':
        show_lines = 1 - show_lines;
        draw_msg( show_lines?"use lines: ON":"use lines: OFF" );
        break;
      case '(':
        show_beams = 1 - show_beams;
        draw_msg( show_beams?"use beams: ON":"use beams: OFF" );
        break;

      case '-':
        if (--marked_beam<0)
          marked_beam += NUM_LASER_BEAMS;
        draw_msg( "marked beam: %d", marked_beam );
        break;
      case '=':
        if (++marked_beam>=NUM_LASER_BEAMS)
          marked_beam -= NUM_LASER_BEAMS;
        draw_msg( "marked beam: %d", marked_beam );
        break;
      case 'a':
        if (show_sensor_info)
          show_sensor_info = 0;
        else
          show_sensor_info = 1;
        draw_msg( show_sensor_info==1?"show sensor info: ON":"show sensor info: OFF" );
        break;
      case 'A':
        show_sensor_info_layout =
            (show_sensor_info_layout&248)|(((show_sensor_info_layout&7)+1)%3);
        break;
      case 'b':
        show_radar = 1 - show_radar;
        draw_msg( show_radar?"radar: ON":"radar: OFF" );
        action = ACTION_SUBSCRIBE;
        break;
      case 'B':
        show_radar = 1 - show_radar;
        show_radar_cal = show_radar;
        draw_msg( show_radar_cal?"radar calibration: ON":"radar calibration: OFF" );
        action = ACTION_SUBSCRIBE;
        break;
      case 'c':
        if (show_colors)
          show_colors = 0;
        else
          show_colors = 1;
        draw_msg( show_colors?"colored by laser numbers: ON":"colored by laser numbers: OFF" );
        break;
      case 'C':
        change_colors = (change_colors+1)%NUM_COLORS;
        break;
      case 'd': case 'D':
        show_distances = 1 - show_distances;
        draw_msg( show_distances?"distance rings: ON":"distance rings: OFF" );
        break;
      case 'e':
        show_info = 1 - show_info;
        draw_msg( show_info?"info: ON":"info: OFF" );
        break;
      case 'E':
        show_status = 1 - show_status;
        draw_msg( show_status?"info: STATUS":"info: ERROR" );
        break;
      case 'f':
        follow_mode = 1 - follow_mode;
        draw_msg( follow_mode?"use game camera mode":"use standard camera mode" );
        break;
      case 'F':
        camera_rot_angle_index =
            (camera_rot_angle_index+1)%CAMERA_NUM_ROT_ANGLES;
        camera_rot_angle = camera_rot_angles[camera_rot_angle_index];
        break;
      case 'g':
        show_grid = 1 - show_grid;
        draw_msg( show_grid?"grid: ON":"grid: OFF" );
        break;
      case 'h': case 'H':
        show_help = 1 - show_help;
        break;
      case 'i':
        show_imagery = 1 - show_imagery;
        draw_msg( show_imagery?"imagery: ON":"imagery: OFF" );
        break;
      case 'I':
        dgc_imagery_cycle_imagery_type();
        if (dgc_imagery_current_imagery_type()==DGC_IMAGERY_TYPE_NONE) {
          dgc_imagery_cycle_imagery_type();
        }
        switch (dgc_imagery_current_imagery_type()) {
        case DGC_IMAGERY_TYPE_COLOR:
          draw_msg( "imagery type: COLOR" );
          break;
        case DGC_IMAGERY_TYPE_TOPO:
          draw_msg( "imagery type: TOPO" );
          break;
        case DGC_IMAGERY_TYPE_LASER:
          draw_msg( "imagery type: LASER" );
          break;
        case DGC_IMAGERY_TYPE_GSAT:
          draw_msg( "imagery type: GOOGLE" );
          break;
        case DGC_IMAGERY_TYPE_DARPA:
          draw_msg( "imagery type: DARPA" );
          break;
        case DGC_IMAGERY_TYPE_BW:
          draw_msg( "imagery type: BW" );
          break;
        default:
          draw_msg( "imagery type: UNKNOWN" );
          break;
        }
        break;
        case 'j': case 'J':
          show_actuators = 1 - show_actuators;
          draw_msg( show_actuators?"show actuators: ON":"show actuators: OFF" );
          break;
        case 'k': case 'K':
          show_inside_view = 1 - show_inside_view;
          draw_msg( show_inside_view?"inside view: ON":"inside view: OFF" );
          break;
        case 'l': case 'L':
          show_lower_block = 1 - show_lower_block;
          draw_msg( show_lower_block?"lower velodyne: ON":"lower velodyne: OFF" );
          break;
        case 'm':
          if (!use_ipc_grid) {
            draw_msg( "IPC grid map interface no available" );
          } else {
            show_ipc_grid_map = 1 - show_ipc_grid_map;
            draw_msg( show_ipc_grid_map?"ipc grid map: ON":"ipc grid map: OFF" );
            //	action = ACTION_SUBSCRIBE;
          }
          break;
        case 'n': case 'N':
          show_rndf = 1 - show_rndf;
          draw_msg( show_rndf?"rndf: ON":"rndf: OFF" );
          break;
        case 'o':
          draw_msg( "request map" );
          action = ACTION_REQUEST_MAP;
          break;
        case 'p': case 'P':
          plain_mode = 1 - plain_mode;
          draw_msg( plain_mode?"plain mode: ON":"plain mode: OFF" );
          break;
        case 27: case 'q': case 'Q':
          dgc_imagery_stop();
#ifdef HAVE_VIDEOOUT
          if(record_video)
            dgc_videoout_release_mt(&vo);
#endif
          exit(0);
          break;
        case 'r': case 'R':
          record_video = 1 - record_video;
#ifdef HAVE_VIDEOOUT
          if (record_video) {
            fprintf( stderr, "# INFO: record video with size %d x %d\n",
                gui3D.window_width, gui3D.window_height );
            vo  = dgc_videoout_init_mt(dgc_unique_filename("perception_view.avi"),
                40 * gui3D.window_width * gui3D.window_height,
                gui3D.window_width, gui3D.window_height,
                15, CODEC_ID_MPEG4,
                0, PIX_FMT_YUV420P);
          } else {
            dgc_videoout_release_mt(&vo);
          }
#endif
          draw_msg( record_video?"record video: ON":"record video: OFF" );
          break;
        case 's': case 'S':
          show_dynamic = 1 - show_dynamic;
          draw_msg( show_dynamic?"mark dynamic: ON":"mark dynamic: OFF" );
          break;
        case 't':
          show_trajectory = 1 - show_trajectory;
          draw_msg( show_trajectory?"trajectory: ON":"trajectory: OFF" );
          action = ACTION_SUBSCRIBE;
          break;
        case 'T':
          show_planner_goal = 1 - show_planner_goal;
          draw_msg( show_planner_goal?"planner goal: ON":"planner goal: OFF" );
          break;
        case 'u': case 'U':
          show_upper_block = 1 - show_upper_block;
          draw_msg( show_upper_block?"upper velodyne: ON":"upper velodyne: OFF" );
          break;
        case 'v':
          show_car = 1 - show_car;
          draw_msg( show_car?"car: ON":"car: OFF" );
          break;
        case 'V':
          show_no_car = 1 - show_no_car;
          draw_msg( show_no_car?"hide car: ON":"hide car: OFF" );
          break;
        case 'w':
          mark_single_beam = 1 - mark_single_beam;
          draw_msg( mark_single_beam?"show marked beam: ON":"show marked beam: OFF" );
          break;
        case 'x': case 'X':
          show_intensity = 1 - show_intensity;
          draw_msg( show_intensity?"intensity: ON":"intensity: OFF" );
          break;
        case 'y': case 'Y':
          sep_colors = 1-sep_colors;
          draw_msg( show_colors?"colored beams: ON":"colored beams: OFF" );
          break;
        case 'z':
          if (show_point_size>1)
            show_point_size = 1;
          else
            show_point_size = 3;
          draw_msg( show_point_size>1?"big points: ON":"big points: OFF" );
          break;
        case 'Z':
          if (show_point_size>1)
            show_point_size = 1;
          else
            show_point_size = (int) (gui3D.window_width/100.0);
          draw_msg( show_point_size>1?"huge points: ON":"huge points: OFF" );
          break;
        case ',': case '<':
          if (!use_velo_shm) {
            draw_msg( "velodyne SHM interface no available" );
          } else {
            show_velodyne = 1 - show_velodyne;
            draw_msg( show_velodyne?"velodyne: ON":"velodyne: OFF" );
          }
          break;
        case '~':
          draw_msg( "reset map" );
          action = ACTION_RESET_MAP;
          break;
        case '`':
          show_car_marker = 1 - show_car_marker;
          draw_msg( show_car_marker?"car marker: ON":"car marker: OFF" );
          break;
        case '[': case '{':
          show_gls = 1 - show_gls;
          draw_msg( show_gls?"gls: ON":"gls: OFF" );
          action = ACTION_SUBSCRIBE;
          break;
        case ']': case '}':
          show_simulator = 1 - show_simulator;
          draw_msg( show_simulator?"simulator: ON":"simulator: OFF" );
          action = ACTION_SUBSCRIBE;
          break;
        case 32:
          show_obstacles = 1 - show_obstacles;
          draw_msg( show_obstacles?"obstacles: ON":"obstacles: OFF" );
          break;
        case 26: //Control Z
          show_track_classifications = 1 - show_track_classifications;
          draw_msg( show_track_classifications?"track_classifications: ON":"track_classifications: OFF" );
          break;
        case 24: //Control X
          show_frame_classifications = 1 - show_frame_classifications;
          draw_msg( show_frame_classifications?"frame_classifications: ON":"frame_classifications: OFF" );
          break;
        default:
          break;
    }
  }
  gui3D_forceRedraw();
}


