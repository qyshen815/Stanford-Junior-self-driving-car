#include "view.h"

QApplication *win = NULL;
QTGui *qgui = NULL;

#define ID_SHOW_ACT          10000
#define ID_SHOW_CTE          10001
#define ID_SHOW_CAN          10002
#define ID_SHOW_GMETER       10003

#define ID_SHOW_VELO         11000
#define ID_SHOW_LDLRS1       11003
#define ID_SHOW_LDLRS2       11004
#define ID_SHOW_LMS1         11005
#define ID_SHOW_LMS2         11006
#define ID_SHOW_RADAR        11008
#define ID_SHOW_ALL_SENSORS  11009

#define ID_SHOW_GRID         12000
#define ID_SHOW_MAP          12001
#define ID_SHOW_RNDF         12002
#define ID_SHOW_IMAGERY      12003

#define ID_PSIZE_01          13000
#define ID_PSIZE_02          13001
#define ID_PSIZE_03          13002
#define ID_PSIZE_04          13003
#define ID_PSIZE_05          13004
#define ID_PSIZE_06          13005
#define ID_PSIZE_07          13006
#define ID_PSIZE_08          13007
#define ID_PSIZE_09          13008
#define ID_PSIZE_10          13009
#define ID_PSIZE_15          13010
#define ID_PSIZE_20          13011
#define ID_PSIZE_30          13012
#define ID_SHOW_COLOR        13013
#define ID_SHOW_INTENSITY    13014

#define ID_SHOW_INFO         14000
#define ID_SHOW_APPLANIX     14001
#define ID_SHOW_INFO_WIN     14002
#define ID_SHOW_ERROR_WIN    14003

#define ID_SNAPSHOT          15000
#define ID_REC_VIDEO         15001
#define ID_640_480           15002
#define ID_720_480           15003      
#define ID_720_578           15004
#define ID_800_600           15005
#define ID_1024_768          15006
#define ID_1280_720          15007
#define ID_1280_960          15008
#define ID_1280_1024         15009
#define ID_1366_768          15010
#define ID_1400_1050         15011

#define ID_FOLLOW_MODE       16000
#define ID_INSIDE_VIEW       16001
#define ID_LOCK_CAMERA       16002

void qgui3D_initialize(int argc, char **argv, int window_x, int window_y, int window_width, int window_height,
    double fps) {
  win = new QApplication(argc, argv);
  qgui = new QTGui(argc, argv, fps, window_x, window_y, window_width, window_height);
}

void qgui3D_mainloop(void) {
  win->exec();
  return;
}

QTGui::QTGui(int argc, char **argv, double fps, int window_x, int window_y, int window_width, int window_height,
    QWidget *parent, const char *name) :
  QMainWindow(parent, name) {
  setCaption("Perception View");

  QPopupMenu *prim = new QPopupMenu(this);
  Q_CHECK_PTR(prim);
  connect(prim, SIGNAL(activated(int)), this, SLOT(menuItemSlot(int)));
  {
    QPopupMenu *primPoints = new QPopupMenu(this);
    Q_CHECK_PTR(primPoints);
    connect(primPoints, SIGNAL(activated(int)), this, SLOT(menuItemSlot(int)));
    primPoints->insertItem("1    ", ID_PSIZE_01);
    primPoints->insertItem("2", ID_PSIZE_02);
    primPoints->insertItem("3", ID_PSIZE_03);
    primPoints->insertItem("4", ID_PSIZE_04);
    primPoints->insertItem("5", ID_PSIZE_05);
    primPoints->insertItem("6", ID_PSIZE_06);
    primPoints->insertItem("7", ID_PSIZE_07);
    primPoints->insertItem("8", ID_PSIZE_08);
    primPoints->insertItem("9", ID_PSIZE_09);
    primPoints->insertItem("10", ID_PSIZE_10);
    primPoints->insertItem("15", ID_PSIZE_15);
    primPoints->insertItem("20", ID_PSIZE_20);
    primPoints->insertItem("30", ID_PSIZE_20);
    prim->insertItem("Point Size", primPoints);
  }
  prim->insertItem("Show Color", ID_SHOW_COLOR);
  prim->insertItem("Show Intesity", ID_SHOW_INTENSITY);

  QPopupMenu *act = new QPopupMenu(this);
  Q_CHECK_PTR(act);
  connect(act, SIGNAL(activated(int)), this, SLOT(menuItemSlot(int)));
  act->insertItem("Show Actuators", ID_SHOW_ACT);
  act->insertItem("Show Cross Track Error", ID_SHOW_CTE);
  act->insertItem("Show CAN Data", ID_SHOW_CAN);
  act->insertItem("Show G-Meter", ID_SHOW_GMETER);

  QPopupMenu *sensor = new QPopupMenu(this);
  Q_CHECK_PTR(sensor);
  connect(sensor, SIGNAL(activated(int)), this, SLOT(menuItemSlot(int)));
  sensor->insertItem("Show Velodyne   ", ID_SHOW_VELO);
  sensor->insertItem("Show LD-LRS 1", ID_SHOW_LDLRS1);
  sensor->insertItem("Show LD-LRS 2", ID_SHOW_LDLRS2);
  sensor->insertItem("Show LMS 1", ID_SHOW_LMS1);
  sensor->insertItem("Show LMS 2", ID_SHOW_LMS2);
  sensor->insertItem("Show Radar", ID_SHOW_RADAR);
  sensor->insertSeparator();
  sensor->insertItem("Show All", ID_SHOW_ALL_SENSORS);

  QPopupMenu *surface = new QPopupMenu(this);
  Q_CHECK_PTR(surface);
  connect(surface, SIGNAL(activated(int)), this, SLOT(menuItemSlot(int)));
  surface->insertItem("Show Grid", ID_SHOW_GRID);
  surface->insertItem("Show Map", ID_SHOW_MAP);
  surface->insertItem("Show RNDF", ID_SHOW_RNDF);
  surface->insertItem("Show Imagery", ID_SHOW_IMAGERY);

  QPopupMenu *info = new QPopupMenu(this);
  Q_CHECK_PTR(info);
  connect(info, SIGNAL(activated(int)), this, SLOT(menuItemSlot(int)));
  info->insertItem("Show Overall Info", ID_SHOW_INFO);
  info->insertItem("Show Applanix Info", ID_SHOW_APPLANIX);
  info->insertItem("Show Info Window", ID_SHOW_INFO_WIN);
  info->insertItem("Show Error Window", ID_SHOW_ERROR_WIN);

  QPopupMenu *display_size = new QPopupMenu(this);
  Q_CHECK_PTR(display_size);
  connect(display_size, SIGNAL(activated(int)), this, SLOT(menuItemSlot(int)));
  display_size->insertItem("640 x 480    ", ID_640_480);
  display_size->insertItem("720 x 480", ID_720_480);
  display_size->insertItem("720 x 578", ID_720_578);
  display_size->insertItem("800 x 600", ID_800_600);
  display_size->insertItem("1024 x 768", ID_1024_768);
  display_size->insertItem("1280 x 720", ID_1280_720);
  display_size->insertItem("1280 x 960", ID_1280_960);
  display_size->insertItem("1280 x 1024", ID_1280_1024);
  display_size->insertItem("1366 x 768", ID_1366_768);
  display_size->insertItem("1400 x 1050", ID_1400_1050);

  QPopupMenu *display = new QPopupMenu(this);
  Q_CHECK_PTR(display);
  connect(display, SIGNAL(activated(int)), this, SLOT(menuItemSlot(int)));
  display->insertItem("Take Snapshot   ", ID_SNAPSHOT);
  display->insertItem("Record Video", ID_REC_VIDEO);
  display->insertItem("Size", display_size);

  QPopupMenu *camera = new QPopupMenu(this);
  Q_CHECK_PTR(camera);
  connect(camera, SIGNAL(activated(int)), this, SLOT(menuItemSlot(int)));
  camera->insertItem("Follow Mode", ID_FOLLOW_MODE);
  camera->insertItem("Inside View   ", ID_INSIDE_VIEW);
  camera->insertItem("Lock Position", ID_LOCK_CAMERA);

  menu = new QMenuBar(this);
  Q_CHECK_PTR( menu);
  //  menu->insertItem( "File", file );
  menu->insertItem("View", prim);
  menu->insertItem("Sensors", sensor);
  menu->insertItem("Actuators", act);
  menu->insertItem("Info", info);
  menu->insertItem("Surface", surface);
  menu->insertItem("Display", display);
  menu->insertItem("Camera", camera);
  //  menu->insertItem( "Help", help );
  menu->show();

  qgui3d = new vlr::QGui3D(argc, argv, fps, this);
  setCentralWidget( qgui3d);
  statusBar()->message("Ready", 2000);

  int border_h = (menuBar()->height() + (menuBar()->height() > 0 ? 1 : 0) + statusBar()->height()
      + (statusBar()->height() > 0 ? 1 : 0));

  resize(window_width, window_height + border_h);
  move(window_x, window_y);

  show();
}

void QTGui::keyPressEvent(QKeyEvent *e) {
  if (e->ascii() == 17) { /* CTRL+Q */
    if (menu->isVisible()) {
      menu->hide();
      statusBar()->hide();
    }
    else {
      menu->show();
      statusBar()->show();
    }
  }
  else if (e->ascii() == 20) { /* CTRL+T */
    char *str = dgc_unique_filename("perception_view.png");
    if (qgui3d->screenshot(str)) {
      fprintf(stderr, "# INFO: save snapshot %s\n", str);
    }
    else {
      fprintf(stderr, "# ERROR: could not save snapshot %s\n", str);
    }
  }
  else {
    qgui3d->keyPressEvent(e);
  }
}

void QTGui::closeEvent(QCloseEvent *) {
#ifdef HAVE_VIDEOOUT
  if(record_video)
  dgc_videoout_release_mt(&vo);
#endif
  exit(0);
}

void QTGui::rndf(void) {
  qgui3d->makeCurrent();
  generate_rndf();
}

void QTGui::set_size(int width, int height) {
  int border_h = (qgui->menuBar()->height() + (qgui->menuBar()->height() > 0 ? 1 : 0) + qgui->statusBar()->height()
      + (qgui->statusBar()->height() > 0 ? 1 : 0));
  resize(width, height + border_h);
}

void QTGui::menuItemSlot(int mID) {
  switch (mID) {
    case ID_SHOW_ACT:
      show_actuators = 1 - show_actuators;
      draw_msg(show_actuators ? "show actuators: ON" : "show actuators: OFF");
      break;
    case ID_SHOW_CTE:
      show_cte = 1 - show_cte;
      draw_msg(show_cte ? "show cross track error: ON" : "show cross track error: OFF");
      break;
    case ID_SHOW_CAN:
      show_canact = 1 - show_canact;
      draw_msg(show_canact ? "show can actuator data: ON" : "show can actuator: OFF");
      break;
    case ID_SHOW_GMETER:
      show_gmeter = 1 - show_gmeter;
      draw_msg(show_gmeter ? "show g-meter: ON" : "show g-meter: OFF");
      break;
    case ID_SHOW_VELO:
      if (!use_velo_shm) {
        draw_msg("velodyne SHM interface no available");
      }
      else {
        show_velodyne = 1 - show_velodyne;
        draw_msg(show_velodyne ? "velodyne: ON" : "velodyne: OFF");
      }
      break;
    case ID_SHOW_LDLRS1:
      show_ldlrs1 = 1 - show_ldlrs1;
      draw_msg(show_ldlrs1 ? "ld-lrs 1: ON" : "ld-lrs 1: OFF");
      action = ACTION_SUBSCRIBE;
      break;
    case ID_SHOW_LDLRS2:
      show_ldlrs2 = 1 - show_ldlrs2;
      draw_msg(show_ldlrs2 ? "ld-lrs 2: ON" : "ld-lrs 2: OFF");
      action = ACTION_SUBSCRIBE;
      break;
    case ID_SHOW_LMS1:
      show_lms1 = 1 - show_lms1;
      draw_msg(show_lms1 ? "lms 1: ON" : "lms 1: OFF");
      action = ACTION_SUBSCRIBE;
      break;
    case ID_SHOW_LMS2:
      show_lms3 = 1 - show_lms3;
      draw_msg(show_lms3 ? "lms 3: ON" : "lms 3: OFF");
      action = ACTION_SUBSCRIBE;
      break;
    case ID_SHOW_RADAR:
      show_radar = 1 - show_radar;
      draw_msg(show_radar ? "radar: ON" : "radar: OFF");
      action = ACTION_SUBSCRIBE;
      break;
    case ID_SHOW_ALL_SENSORS:
      show_all_sensors = 1 - show_all_sensors;
      if (show_all_sensors) {
        show_ldlrs1 = 1;
        show_ldlrs2 = 1;
        show_lms1 = 1;
        show_lms3 = 1;
        show_velodyne = 1;
        draw_msg("all sensors: ON");
      }
      else {
        show_ldlrs1 = 0;
        show_ldlrs2 = 0;
        show_lms1 = 0;
        show_lms3 = 0;
        show_velodyne = 0;
        draw_msg("all sensors: OFF");
      }
      action = ACTION_SUBSCRIBE;
      break;
    case ID_PSIZE_01:
      show_point_size = 1;
      draw_msg("point size: %d", show_point_size);
      break;
    case ID_PSIZE_02:
      show_point_size = 2;
      draw_msg("point size: %d", show_point_size);
      break;
    case ID_PSIZE_03:
      show_point_size = 3;
      draw_msg("point size: %d", show_point_size);
      break;
    case ID_PSIZE_04:
      show_point_size = 4;
      draw_msg("point size: %d", show_point_size);
      break;
    case ID_PSIZE_05:
      show_point_size = 5;
      draw_msg("point size: %d", show_point_size);
      break;
    case ID_PSIZE_06:
      show_point_size = 6;
      draw_msg("point size: %d", show_point_size);
      break;
    case ID_PSIZE_07:
      show_point_size = 7;
      draw_msg("point size: %d", show_point_size);
      break;
    case ID_PSIZE_08:
      show_point_size = 8;
      draw_msg("point size: %d", show_point_size);
      break;
    case ID_PSIZE_09:
      show_point_size = 9;
      draw_msg("point size: %d", show_point_size);
      break;
    case ID_PSIZE_10:
      show_point_size = 10;
      draw_msg("point size: %d", show_point_size);
      break;
    case ID_PSIZE_15:
      show_point_size = 15;
      draw_msg("point size: %d", show_point_size);
      break;
    case ID_PSIZE_20:
      show_point_size = 20;
      draw_msg("point size: %d", show_point_size);
      break;
    case ID_PSIZE_30:
      show_point_size = 30;
      draw_msg("point size: %d", show_point_size);
      break;
    case ID_SHOW_COLOR:
      sep_colors = 1 - sep_colors;
      draw_msg(show_colors ? "colored beams: ON" : "colored beams: OFF");
      break;
    case ID_SHOW_INTENSITY:
      show_intensity = 1 - show_intensity;
      draw_msg(show_intensity ? "intensity: ON" : "intensity: OFF");
      break;
    case ID_640_480:
      resize(640, 480);
      draw_msg("[640 x 480]");
      break;
    case ID_720_480:
      resize(720, 480);
      draw_msg("[720 x 480]");
      break;
    case ID_720_578:
      resize(720, 578);
      draw_msg("[720 x 578]");
      break;
    case ID_800_600:
      resize(800, 600);
      draw_msg("[800 x 600]");
      break;
    case ID_1024_768:
      resize(1024, 768);
      draw_msg("[1024 x 768]");
      break;
    case ID_1280_720:
      resize(1280, 720);
      draw_msg("[1280 x 720]");
      break;
    case ID_1280_960:
      resize(1280, 960);
      draw_msg("[1280 x 960]");
      break;
    case ID_1280_1024:
      resize(1280, 1024);
      draw_msg("[1280 x 768]");
      break;
    case ID_1366_768:
      resize(1366, 768);
      draw_msg("[1366 x 768]");
      break;
    case ID_1400_1050:
      resize(1400, 1050);
      draw_msg("[1400 x 1050]");
      break;
    case ID_REC_VIDEO:
      record_video = 1 - record_video;
#ifdef HAVE_VIDEOOUT
      if (record_video) {
        fprintf( stderr, "# INFO: record video with size %d x %d\n",
            vlr::gui3D.window_width, vlr::gui3D.window_height );
        vo = dgc_videoout_init_mt(dgc_unique_filename("perception_view.avi"),
            100 * vlr::gui3D.window_width * vlr::gui3D.window_height,
            vlr::gui3D.window_width, vlr::gui3D.window_height,
            15, CODEC_ID_MPEG4,
            0, PIX_FMT_YUV420P);
      }
      else {
        dgc_videoout_release_mt(&vo);
      }
      draw_msg( record_video?"record video: ON":"record video: OFF" );
#endif
      break;
    case ID_SNAPSHOT: {
      char *str = dgc_unique_filename("perception_view.png");
      if (qgui3d->screenshot(str)) {
        fprintf(stderr, "# INFO: save snapshot %s\n", str);
      }
      else {
        fprintf(stderr, "# ERROR: could not save snapshot %s\n", str);
      }
    }
      break;
    case ID_SHOW_INFO:
      if (show_sensor_info)
        show_sensor_info = 0;
      else
        show_sensor_info = 1;
      draw_msg(show_sensor_info == 1 ? "show sensor info: ON" : "show sensor info: OFF");
      break;
    case ID_SHOW_APPLANIX:
      if (show_sensor_info)
        show_sensor_info = 0;
      else
        show_sensor_info = 2;
      draw_msg(show_sensor_info == 2 ? "show applanix info: ON" : "show applanix info: OFF");
      break;
    case ID_SHOW_ERROR_WIN:
      show_status = 1 - show_status;
      draw_msg(show_status ? "info: STATUS" : "info: ERROR");
      break;
    case ID_SHOW_INFO_WIN:
      show_info = 1 - show_info;
      draw_msg(show_info ? "info: ON" : "info: OFF");
      break;
    case ID_FOLLOW_MODE:
      follow_mode = 1 - follow_mode;
      draw_msg(follow_mode ? "use game camera mode" : "use standard camera mode");
      break;
    case ID_INSIDE_VIEW:
      show_inside_view = 1 - show_inside_view;
      draw_msg(show_inside_view ? "inside view: ON" : "inside view: OFF");
      break;
    case ID_LOCK_CAMERA:
      if (!follow_mode) {
        camera_lock = 1 - camera_lock;
        draw_msg(camera_lock ? "camera lock: ON" : "camera lock: OFF");
      }
      else {
        draw_msg("camera lock not possibe - disable follow mode first");
      }
      break;
    case ID_SHOW_GRID:
      show_grid = 1 - show_grid;
      draw_msg(show_grid ? "grid: ON" : "grid: OFF");
      break;
    case ID_SHOW_MAP:
      if (!use_ipc_grid) {
        draw_msg("IPC grid map interface no available");
      }
      else {
        show_ipc_grid_map = 1 - show_ipc_grid_map;
        draw_msg(show_ipc_grid_map ? "ipc grid map: ON" : "ipc grid map: OFF");
        action = ACTION_SUBSCRIBE;
      }
      break;
    case ID_SHOW_RNDF:
      show_rndf = 1 - show_rndf;
      draw_msg(show_rndf ? "rndf: ON" : "rndf: OFF");
      break;
    case ID_SHOW_IMAGERY:
      show_imagery = 1 - show_imagery;
      draw_msg(show_imagery ? "imagery: ON" : "imagery: OFF");
      break;

    default:
      break;
  }
}
