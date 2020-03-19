#include <vector>
#include <iostream>
#include <sys/stat.h>

#include <videoout.h>
#include <passatmodel.h>
#include <passat_constants.h>
#include <gui3D.h>
#include <track_manager.h>
#include <multibooster_support.h>
#include <track_descriptors.h>

#include <boost/filesystem.hpp>
#include <cluster_descriptors/cluster_descriptors.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace dgc;
using namespace std;
using boost::shared_ptr;
using namespace track_manager;
using namespace sensor_msgs;
using namespace vlr;
using namespace Eigen;
using namespace pipeline;
using namespace track_descriptors;

#define NUM_THREADS getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 1

TrackManager g_tm;
string g_track_manager_filename;
shared_ptr<PointCloud> g_cloud;
shared_ptr<Frame> g_frame;
shared_ptr<Track> g_track;
int g_track_num = 0;
int g_frame_num = 0;
bool g_changes_made = false;

bool g_show_extended_info = false;
bool g_draw_centroid = false;
bool g_draw_robot = true;
bool g_draw_grid = true;
bool g_camera_hold = false;
bool g_pause = false;
bool g_follow = true;
int g_playback_speed = 1;
bool g_wrap = true; //When at the end of a track, if this is true it will wrap back to cloud 0 of the current track; otherwise it will move on to the next track.
bool g_random_mode = false;
bool g_record = false;
dgc_videoout_p g_vo = NULL;
pthread_mutex_t g_video_mutex = PTHREAD_MUTEX_INITIALIZER;
bool g_display_intensity = true;
bool g_dumping_images = false;

shared_ptr<CombinedClassifierPipeline> g_ccp;
int g_classified_track_num = -1;
vector<VectorXf> g_frame_responses;
VectorXf g_adbf_response;
VectorXf g_global_response;
vector<bool> g_selected;

dgc_passatwagonmodel_t* g_passat = NULL;


IplImage* vo2Ipl(unsigned char *vo, int width, int height) {
  int vo_width_step = width * 3;
  IplImage* ipl = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
  for(int y = 0; y < height; ++y) {
    uchar* ipl_ptr = (uchar*)(ipl->imageData + y * ipl->widthStep);
    uchar* vo_ptr = (uchar*)(vo + y * vo_width_step);
    for(int x = 0; x < width; ++x) {
      ipl_ptr[3*x+2] = vo_ptr[3*x+0];
      ipl_ptr[3*x+1] = vo_ptr[3*x+1];
      ipl_ptr[3*x+0] = vo_ptr[3*x+2];
    }
  }

  return ipl;
}

IplImage* grabFrame() {
  static bool allocated = false;
  static unsigned char *buf = NULL;
  static unsigned char *buf2 = NULL;
  static GLenum format;
  static int bpp;

  if(!allocated) {
    bpp = glutGet(GLUT_WINDOW_RGBA) ? 4 : 3;
    format = (bpp==4) ? GL_RGBA : GL_RGB;
    buf = (unsigned char*)realloc(buf, gui3D.window_width * gui3D.window_height * bpp);
    dgc_test_alloc(buf);
    buf2 = (unsigned char*)realloc(buf2, gui3D.window_width * gui3D.window_height * 3);
    dgc_test_alloc(buf2);
    allocated = true;
  }

  glReadPixels(0, 0, gui3D.window_width, gui3D.window_height,
	       format, GL_UNSIGNED_BYTE, buf);

  if (bpp==4)
    dgc_videoout_flip24_vertically(buf, 
				   gui3D.window_width, gui3D.window_height, bpp,
				   buf2);
  else
    dgc_videoout_flip_vertically(buf, 
				 gui3D.window_width, gui3D.window_height, bpp,
				 buf2);

  IplImage* ipl = vo2Ipl(buf2, gui3D.window_width, gui3D.window_height);
  return ipl;
}

void getCloudStats(const PointCloud& cloud, double* x, double* y, double* z, double* min_z = NULL, double* max_z = NULL) {
  if(min_z)
    *min_z = FLT_MAX;
  if(max_z)
    *max_z = FLT_MIN;
  for(size_t i=0; i<cloud.get_points_size(); ++i) {
    *x += cloud.points[i].x;
    *y += cloud.points[i].y;
    *z += cloud.points[i].z;
    if(min_z && cloud.points[i].z < *min_z)
      *min_z = cloud.points[i].z;
    if(max_z && cloud.points[i].z > *max_z)
      *max_z = cloud.points[i].z;
  }
  if(cloud.get_points_size() > 0) {
    *x /= cloud.get_points_size();
    *y /= cloud.get_points_size();
    *z /= cloud.get_points_size();
  }
  else {
    *x = 0;
    *y = 0;
    *z = 0;
  } 
}

void drawGrid(double center_x, double center_y) {
  glBegin(GL_LINES);
  glLineWidth(3);
  glColor3f(0.5, 0.5, 0.5);


  int increment = 5;
  double z = -1.4;
  int extent = 50;
  for(int grid_x = -extent; grid_x <= extent; grid_x+=increment) {
    glVertex3f(grid_x - center_x, -extent - center_y, z);
    glVertex3f(grid_x - center_x,  extent - center_y, z);
  }
  for(int grid_y = -extent; grid_y <= extent; grid_y+=increment) {
    glVertex3f(-extent - center_x, grid_y - center_y, z );
    glVertex3f( extent - center_x, grid_y - center_y, z );
  }
  glEnd();
}

void incrementTrack(int i) {
  g_frame_num = 0;
  g_track_num += i;
  if(g_track_num >= (int)g_tm.tracks_.size())
    g_track_num = 0;
  if(g_track_num < 0)
    g_track_num = g_tm.tracks_.size() - 1;
}

void incrementTrackSameLabel(int incr) {
  assert(incr == -1 || incr == 1);
  for(size_t i = g_track_num + incr; i < g_tm.tracks_.size(); i += incr) {
    if(g_tm.tracks_[i]->label_.compare(g_track->label_) == 0) { 
      g_track_num = i;
      g_frame_num = 0;
      break;
    }
  }
}

void incrementCloud(int i) {
  g_frame_num += i;
  if(g_wrap) { 
    if(g_frame_num >= (int)g_tm.tracks_[g_track_num]->frames_.size())
      g_frame_num = 0;
    if(g_frame_num < 0)
      g_frame_num = g_tm.tracks_[g_track_num]->frames_.size() - 1;
  }
  else {
    if(g_frame_num >= (int)g_tm.tracks_[g_track_num]->frames_.size()) { 
      g_frame_num = 0;

      if(g_random_mode)
	g_track_num = rand() % g_tm.tracks_.size();
      else
	incrementTrack(1);
    }
    if(g_frame_num < 0) { 
      g_frame_num = 0;
      incrementTrack(-1);
    }
  }

}

string getNextSavename() {
  int video_num = 0;
  char filename[100];
  while(true) {
    sprintf(filename, "track%04d.avi", video_num);
    struct stat file_info;
    int status = stat(filename, &file_info);
    if(status == 0) //File exists.
      video_num++;
    else
      break;
  }
  string str(filename);
  return str;
}

void printTrackDescriptors(shared_ptr<Track> track) {
  TrackClassifierPipeline tdp(NULL, true);
  
  Object* obj = tdp.computeMultiBoosterObject(track);
  cout << endl << obj->status(vector<string>(), tdp.getDescriptorNames()) << endl;
  delete obj;
}

void classifyTrack() {
  g_adbf_response = g_ccp->classify(g_track, &g_global_response, &g_frame_responses);
  g_classified_track_num = g_track_num;
}

void classifyFrame() {
  shared_ptr<Track> tmp(new Track(*g_track));
  tmp->frames_ = vector< shared_ptr<Frame> >(1);
  tmp->frames_[0] = g_frame;

  // -- Only show the virtual orthographic camera intensity images.
  Pipeline& pl = g_ccp->fcp_->pipeline_;
  vector< shared_ptr<CloudProjector> > projectors = pl.filterNodes<CloudProjector>();
  for(size_t i = 0; i < projectors.size(); ++i) {
    projectors[i]->debug_ = true;
  }
  
  g_ccp->fcp_->classify(tmp);
  g_ccp->setDebug(false);
}


string getImageFilename(int num) {
  char str[300];
  sprintf(str, "image_%06d.png", num);
  return str;
}

string getNextImageFilename() {
  int num = 0;
  while(true) {
    if(!boost::filesystem::exists(getImageFilename(num)))
      break;
    ++num;
  }
  return getImageFilename(num);
}    


void timer(int)
{
  // -- Increment the cloud we are looking at.
  if(!g_pause) {
    incrementCloud(g_playback_speed);
  }
  if(g_dumping_images) {
    g_track_num = rand() % g_tm.tracks_.size();
    g_frame_num = rand() % g_tm.tracks_[g_track_num]->frames_.size();
  }

  g_track = g_tm.tracks_[g_track_num];
  g_frame = g_track->frames_[g_frame_num];

  // -- Set g_cloud to point to the cloud, or NULL if there is none.
  if(g_tm.tracks_[g_track_num]->frames_.empty()) {
    g_cloud.reset();
    assert(g_cloud.get() == NULL);
    cout << "Warning: track " << g_track_num << " has no point clouds." << endl;
  }
  else
    g_cloud = g_tm.tracks_[g_track_num]->frames_[g_frame_num]->cloud_;
  
  // -- Redraw.
  gui3D_forceRedraw();
  gui3D_add_timerFunc(100, timer, 0);

  // -- Dump out an image and save it.
  if(g_dumping_images) { 
    IplImage* img = grabFrame();
    string filename = getNextImageFilename();
    cout << "Saving to " << filename << endl;
    cvSaveImage(filename.c_str(), img);
    cvReleaseImage(&img);
  }
}

void toggleRecord() {
  pthread_mutex_lock(&g_video_mutex);
  g_record = !g_record;
  if(g_record) {
    //string video_name = getNextSavename();
    g_vo  = dgc_videoout_init(dgc_unique_filename("track_visualizer.avi"),
			      60 * gui3D.window_width *
			      gui3D.window_height, gui3D.window_width,
			      gui3D.window_height, 10, CODEC_ID_MPEG4,
			      0, PIX_FMT_YUV420P);

 
//     g_vo  = dgc_videoout_init(video_name.c_str(), 4000000, 720, 480,
// 			      10, CODEC_ID_MPEG4, 0, PIX_FMT_YUV420P);
  }
  else {
    cout << "Done recording." << endl;
    dgc_videoout_release(&g_vo);
    g_vo = NULL;
  }
  pthread_mutex_unlock(&g_video_mutex);
}

void saveSelected()
{
  int num_selected = 0;
  for(size_t i = 0; i < g_selected.size(); ++i)
    if(g_selected[i])
      ++num_selected;

  if(num_selected == 0) {
    cout << "No tracks selected.  Not saving..." << endl;
    return;
  }
  
  cout << "Save " << num_selected << " selected tracks to (c to cancel): ";
  string filename;
  cin >> filename;
  if(filename.size() == 1 && filename[0] == 'c') {
    cout << "Cancelled." << endl;
    return;
  }
  cout << "Saving to " << filename << endl;

  vector< shared_ptr<Track> > tracks;
  tracks.reserve(num_selected);
  for(size_t i = 0; i < g_selected.size(); ++i) {
    if(g_selected[i])
      tracks.push_back(g_tm.tracks_[i]);
  }
  
  TrackManager sel(tracks);
  sel.save(filename);
  cout << "Saved." << endl;
}

void findObject()
{
  string label;
  cout << "Find object with label: "; cout.flush();
  cin >> label;
  cout << "Searching for \"" << label << "\"" << endl;

  for(size_t i = g_track_num; i < g_tm.tracks_.size(); ++i) {
    if(g_tm.tracks_[i]->label_.compare(label) == 0) { 
      g_track_num = i;
      g_frame_num = 0;
      cout << "Found." << endl;
      break;
    }
    if(i == g_tm.tracks_.size() - 1)
      cout << "None found between here and the end of the database." << endl;
  }
}

void keyboard(unsigned char key, int x, int y)
{
  string str;
  switch(key) {
  case 'f':
    findObject();
    break;
  case 'h':
    g_camera_hold = !g_camera_hold;
    break;
  case 'n':
    g_draw_centroid = !g_draw_centroid;
    break;
  case 'R':
    g_draw_robot = !g_draw_robot;
    break;
  case 'I':
    g_dumping_images = !g_dumping_images;
    break;
  case 'i':
    g_display_intensity = !g_display_intensity;
    break;
  case 'D':
    printTrackDescriptors(g_track);
    break;
  case 'd':
    if(!g_ccp) {
      cout << "No classifier has been supplied." << endl;
    }
    else { 
      classifyFrame();
    }
    break;
  case '+':
    g_playback_speed++;
    break;
  case '-':
    if(g_playback_speed > 1)
      g_playback_speed--;
    break;
  case '*':
    g_playback_speed *= 2;
    break;
  case '/':
    if(g_playback_speed > 1)
      g_playback_speed /= 2;
    break;
  case 'k':
    if(g_pause)
      incrementCloud(1);
    else
      incrementTrack(1);
    break;
  case 'w':
    g_wrap = !g_wrap;
    break;
  case 'J':
    if(!g_pause)
      incrementTrackSameLabel(-1);
    break;
  case 'K':
    if(!g_pause)
      incrementTrackSameLabel(1);
    break;
  case 'j':
    if(g_pause)
      incrementCloud(-1);
    else
      incrementTrack(-1);
    break;
  case 'q':
    if(g_changes_made) {
      cout << "Changes made since last save.  Are you sure (y/N)?" << endl;
      cin >> str;
      if(str.compare("y") == 0)
	exit(0);
      else
	cout << "Not quitting yet..." << endl;
    }
    else
      exit(0);
    break;
  case 'r':
    g_random_mode = !g_random_mode;
    cout << "Random mode: " << g_random_mode << endl;
    break;
  case 'v':
    toggleRecord();
    break;
  case 'e':
    g_show_extended_info = !g_show_extended_info;
    break;
  case ' ':
    g_pause = !g_pause;
    break;

  case 'C':
    classifyTrack();
    break;

    
    
  // -- Labeling commands.
  case 'c':
    g_tm.tracks_[g_track_num]->label_ = "car";
    g_changes_made = true;
    break;
  case 'b':
    g_tm.tracks_[g_track_num]->label_ = "bicyclist";
    g_changes_made = true;
    break;
  case 'p':
    g_tm.tracks_[g_track_num]->label_ = "pedestrian";
    g_changes_made = true;
    break;
  case 'u':
    g_tm.tracks_[g_track_num]->label_ = "unlabeled";
    g_changes_made = true;
    break;
  case 'g':
    g_tm.tracks_[g_track_num]->label_ = "background";
    g_changes_made = true;
    break;
  case 'G':
    g_draw_grid = !g_draw_grid;
    break;
  case 's':
    g_tm.save(g_track_manager_filename);
    cout << "Saved track manager as " << g_track_manager_filename << endl;
    g_changes_made = false;
    break;

  case '\\':
    g_selected[g_track_num] = !g_selected[g_track_num];
    break;
  case 'S':
    saveSelected();
    break;
  default:
    break;
  }

  g_cloud = g_tm.tracks_[g_track_num]->frames_[g_frame_num]->cloud_;
  gui3D_forceRedraw();
}


void mouse(__attribute__ ((unused)) int button, 
    int state, int x, int y)
{
}

void motion(int x, int y)
{
}

void setColor(int label, int c, double logodds) {
  if(label == -2)
    glColor3f(1, 1, 1);
  else if((label == c && logodds > 0) || (label != c && logodds <= 0))
    glColor3f(0, 1, 0);
  else
    glColor3f(1, 0, 0);
}

void drawInfoBox(void)
{
//  cout << setprecision(16) << g_frame->timestamp_ << endl;
  
  char str[200];
  set_display_mode_2D(gui3D.window_width, gui3D.window_height);
  glColor3f(1, 1, 1);
  
  // -- If we're on the last frame, let the user know.
  if(g_frame_num >= (int)g_tm.tracks_[g_track_num]->frames_.size() - g_playback_speed && !g_dumping_images) {
    sprintf(str, "LAST SEGMENT");
    renderBitmapString((gui3D.window_width - bitmapStringWidth(GLUT_BITMAP_HELVETICA_18, str)) / 2.0,
		       gui3D.window_height / 7.0, GLUT_BITMAP_TIMES_ROMAN_24, str);
  }

  sprintf(str, "%s", g_tm.tracks_[g_track_num]->label_.c_str());
  renderBitmapString(gui3D.window_width - bitmapStringWidth(GLUT_BITMAP_HELVETICA_18, str) - 10, 20, GLUT_BITMAP_HELVETICA_18, str);

  if(g_selected[g_track_num]) {
    sprintf(str, "selected");
    renderBitmapString(gui3D.window_width / 2, 20, GLUT_BITMAP_HELVETICA_18, str);
  }

  
  if(g_pause) { 
    sprintf(str, "Paused");
    renderBitmapString(20, 160, GLUT_BITMAP_HELVETICA_18, str);
  }

//   double dist = 0;
//   if(g_frame_num > 5)
//     dist = (g_frame->getCentroid() - g_track->frames_[g_framep_num - 5]->getCentroid()).norm();
//   sprintf(str, "Dist to prev: %f", dist);
//   renderBitmapString(20, 140, GLUT_BITMAP_HELVETICA_18, str);

  sprintf(str, "Track %d of %d", g_track_num+1, (int)g_tm.tracks_.size());
  renderBitmapString(20, 140, GLUT_BITMAP_HELVETICA_18, str);

  sprintf(str, "Cloud %d of %d", g_frame_num+1, (int)g_tm.tracks_[g_track_num]->frames_.size());
  renderBitmapString(20, 120, GLUT_BITMAP_HELVETICA_18, str);

  sprintf(str, "%d points", g_cloud->get_points_size());
  renderBitmapString(20, 100, GLUT_BITMAP_HELVETICA_18, str);
 
  sprintf(str, "Time from start (s): %.2f", g_frame->timestamp_ - g_track->frames_[0]->timestamp_);
  renderBitmapString(20, 80, GLUT_BITMAP_HELVETICA_18, str);

  sprintf(str, "Distance to sensor (m): %.2f", g_frame->getDistance(g_track->velodyne_offset_));
  renderBitmapString(20, 60, GLUT_BITMAP_HELVETICA_18, str);
  
  sprintf(str, "Playback Speed: %d", g_playback_speed);
  renderBitmapString(20, 40, GLUT_BITMAP_HELVETICA_18, str);
  
  if(g_wrap)
    sprintf(str, "Wrap mode: on");
  else
    sprintf(str, "Wrap mode: off ");
  renderBitmapString(20, 20, GLUT_BITMAP_HELVETICA_18, str);


  // -- Classifier related messages.
  if(g_classified_track_num == g_track_num) { 
    int frame_x = 150;
    int global_x = 300;
    int adbf_x = 400;

    sprintf(str, "Class");
    renderBitmapString(20, gui3D.window_height - 20, GLUT_BITMAP_HELVETICA_18, str);
    sprintf(str, "Appearance");
    renderBitmapString(frame_x, gui3D.window_height - 20, GLUT_BITMAP_HELVETICA_18, str);
    sprintf(str, "Behavior");
    renderBitmapString(global_x, gui3D.window_height - 20, GLUT_BITMAP_HELVETICA_18, str);
    sprintf(str, "ADBF");
    renderBitmapString(adbf_x, gui3D.window_height - 20, GLUT_BITMAP_HELVETICA_18, str);

    assert(g_cloud == g_tm.tracks_[g_track_num]->frames_[g_frame_num]->cloud_);

    int label = -2;
    string label_str = g_tm.tracks_[g_track_num]->label_;
    if(label_str.compare("background") == 0)
      label = -1;
    else if(label_str.compare("unlabeled") == 0)
      label = -2;
    else
      label = g_ccp->getClassMap().toId(label_str);

    for(size_t i=0; i<g_ccp->getClassMap().size(); ++i) {
      int height = gui3D.window_height - 40 - 20*i;
      
      glColor3f(1, 1, 1);
      sprintf(str, "%s", g_ccp->getClassMap().toName(i).c_str());
      renderBitmapString(20, height, GLUT_BITMAP_HELVETICA_18, str);

      // -- Print out frame response.
      setColor(label, i, g_frame_responses[g_frame_num](i));
      sprintf(str, "%.2f", g_frame_responses[g_frame_num](i));
      renderBitmapString(frame_x, height, GLUT_BITMAP_HELVETICA_18, str);

      // -- Print out global response.
      setColor(label, i, g_global_response(i));
      sprintf(str, "%.2f", g_global_response(i));
      renderBitmapString(global_x, height, GLUT_BITMAP_HELVETICA_18, str);
      
      // -- Print out final ADBF response.
      setColor(label, i, g_adbf_response(i));
      sprintf(str, "%.2f", g_adbf_response(i));
      renderBitmapString(adbf_x, height, GLUT_BITMAP_HELVETICA_18, str);
    }
    glColor3f(1, 1, 1);
  }
  
}

void display(void)
{
  
  // -- Clear to black.
  glClearColor(0, 0, 0, 0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // -- Turn on smooth lines.
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_LINE_SMOOTH);
  
  // -- If this track doesn't have any pointclouds associated with it, don't show anything.  
  if(!g_cloud)
    return;
  if(g_draw_grid)
    drawGrid(0,0);
  
  // -- Set the camera position.
  double old_offset_x = gui3D.camera_pose.x_offset;
  double old_offset_y = gui3D.camera_pose.y_offset;
  double old_offset_z = gui3D.camera_pose.z_offset;
    
  shared_ptr<PointCloud> cloud0_ptr;
  cloud0_ptr = g_tm.tracks_[g_track_num]->frames_[0]->cloud_;
  
  double x_init = 0;
  double y_init = 0;
  double z_init = 0;
  double min_z_init = 0;
  double max_z_init = 0;
  getCloudStats(*cloud0_ptr.get(), &x_init, &y_init, &z_init, &min_z_init, &max_z_init);

  double x_curr = 0;
  double y_curr = 0;
  double z_curr = 0;
  double min_z_curr = 0;
  double max_z_curr = 0;
  getCloudStats(*g_frame->cloud_, &x_curr, &y_curr, &z_curr, &min_z_curr, &max_z_curr);

  if(g_dumping_images) {
    x_init = x_curr;
    y_init = y_curr;
    z_init = z_curr;
    min_z_init = min_z_curr;
    max_z_init = max_z_curr;
  }
  
  if(!g_camera_hold) {
    static int prev_track_num = -1;
    //double alpha = 0.2;
    double alpha = 1.0;
    if(g_frame_num == 0 || g_track_num != prev_track_num)
      alpha = 1.0;
    prev_track_num = g_track_num;
    
    double new_offset_x = x_curr - x_init;
    double new_offset_y = y_curr - y_init;
    double new_offset_z = z_curr - g_frame->robot_pose_.z;;
    
    gui3D.camera_pose.x_offset = alpha * new_offset_x + (1 - alpha) * old_offset_x;
    gui3D.camera_pose.y_offset = alpha * new_offset_y + (1 - alpha) * old_offset_y;
    gui3D.camera_pose.z_offset = alpha * new_offset_z + (1 - alpha) * old_offset_z;
  }

  
  // -- Draw the robot.
  if(g_draw_robot) { 
    dgc_pose_t robot = g_track->frames_[g_frame_num]->robot_pose_;
    glEnable(GL_LIGHTING);
    glPushMatrix();
    glTranslatef(robot.x - x_init, robot.y - y_init, 0);
    glRotatef(dgc_r2d(robot.yaw), 0, 0, 1);
    glRotatef(dgc_r2d(robot.pitch), 0, 1, 0);
    glRotatef(dgc_r2d(robot.roll), 1, 0, 0);
    glTranslatef(1.65, 0, -0.6);
    passatwagonmodel_draw(g_passat, 0, 0, 0);
    glPopMatrix();
    glDisable(GL_LIGHTING);
  }

//   glColor3f(0, 0, 1);
//   glPushMatrix();
//   glTranslatef(robot.x - x_init,
// 	       robot.y - y_init,
// 	       robot.z - g_frame->robot_pose_.z);
//   glutSolidCube(0.5);
//   glPopMatrix();

//   // -- Draw the velodyne.
//   double x_velo = 0;
//   double y_velo = 0;
//   double z_velo = 0;
//   g_track->frames_[g_frame_num]->getVelodyneXYZ(g_track->velodyne_offset_, &x_velo, &y_velo, &z_velo);
//   glColor3f(1, 0, 0);
//   glPushMatrix();
//   glTranslatef(x_velo - x_init,
// 	       y_velo - y_init,
// 	       z_velo - g_frame->robot_pose_.z);
//   glutSolidCube(0.5);
//   glPopMatrix();


  // -- Draw the centroid.
  if(g_draw_centroid) { 
    Vector3f centroid = g_frame->getCentroid();
    glColor3f(0, 1, 0);
    glPushMatrix();
    glTranslatef(centroid(0) - x_init,
		 centroid(1) - y_init,
		 centroid(2) - g_frame->robot_pose_.z);
    glutSolidCube(0.25);
    glPopMatrix();
  }  
  
  // -- Draw the point cloud.
  for(size_t i=0; i<g_cloud->get_points_size(); ++i) {
    assert(g_cloud->get_channels_size() > 0);
    assert(g_cloud->channels[0].get_values_size() == g_cloud->get_points_size());
      
    if(!g_display_intensity) { 
      double u = (g_cloud->points[i].z - g_frame->robot_pose_.z) / 3.0;
      if(u > 1)
	u = 1;
      else if(u < 0)
	u = 0;
      glColor3f(1 - u, u, 0);
    }
    else {
      double u = (double)g_cloud->channels[0].values[i] / 255.;
      glColor3f(u, 0.5+0.5*u, u);
    }

    glPushMatrix();
    glTranslatef(g_cloud->points[i].x - x_init,
		 g_cloud->points[i].y - y_init,
		 g_cloud->points[i].z - g_frame->robot_pose_.z);
    glutSolidCube(0.05);
    glPopMatrix();
  }

  drawInfoBox();

  pthread_mutex_lock(&g_video_mutex);
  if(g_record && g_vo) {
    dgc_videoout_add_opengl_frame(g_vo);
//     free(g_vo->screenshot_buffer);
//     free(g_vo->screenshot_buffer2);
//     g_vo->screenshot_buffer = NULL;
//     g_vo->screenshot_buffer2 = NULL;
  }
  pthread_mutex_unlock(&g_video_mutex);

}

string usageString() {
  ostringstream oss;
  oss << "usage: " << endl;
  oss << " track_visualizer TM" << endl;
  oss << endl;
  oss << " track_visualizer --adbf SEGMENT_CLASSIFIER HOLISTIC_CLASSIFIER ADBF_WEIGHTS TM" << endl;
  return oss.str();
}

int main(int argc, char **argv)
{
  // -- Just visualize tracks.
  if(argc == 2 && strstr(argv[1], ".tm") != NULL) {
    g_track_manager_filename = argv[1];
  }
  else if(argc == 6 &&
	  strcmp(argv[1], "--adbf") == 0 &&
	  boost::filesystem::extension(argv[2]).compare(".mb") == 0 &&
	  boost::filesystem::extension(argv[3]).compare(".mb") == 0 &&
	  boost::filesystem::extension(argv[5]).compare(".tm") == 0) {

    string frame_classifier_filename(argv[2]);
    string global_classifier_filename(argv[3]);
    string adbf_weights_filename(argv[4]);
    g_track_manager_filename = argv[5];

    cout << "Visualizing " << g_track_manager_filename << ", classifying with ADBF." << endl;
    cout << "Appearance classifier: " << frame_classifier_filename << endl;
    cout << "Behavior classifier: " << global_classifier_filename << endl;
    cout << "ADBF weights file: " << adbf_weights_filename << endl;

    VectorXd adbf_weights;
    eigen_extensions::load(adbf_weights_filename, &adbf_weights);
    cout << "ADBF weights: " << adbf_weights.transpose() << endl;
    assert(adbf_weights.rows() == 3);
    g_ccp = shared_ptr<CombinedClassifierPipeline>(new CombinedClassifierPipeline(frame_classifier_filename,
										  global_classifier_filename,
										  adbf_weights,
										  NUM_THREADS));
										 
  }
  else {
    cout << usageString() << endl;
    return 1;
  }

  // -- Load tracks, etc.
  cout << "Loading track manager " << g_track_manager_filename << endl;
  g_tm = TrackManager(g_track_manager_filename);
  cout << "Loaded " << g_tm.tracks_.size() << " tracks." << endl;
  if(g_tm.tracks_.size() == 0) {
    cerr << "0 tracks.  Aborting." << endl;
    exit(1);
  }
  g_tm.sortTracks();
  g_track = g_tm.tracks_[0];
  g_frame = g_track->frames_[0];
  g_cloud = g_frame->cloud_;

  g_selected = vector<bool>(g_tm.tracks_.size(), false);
  
  double num_clouds = 0;
  for(size_t i=0; i<g_tm.tracks_.size(); ++i) {
    num_clouds += g_tm.tracks_[i]->frames_.size();
  }
  cout << "Total number of clouds: " << num_clouds << endl;
  
  // -- Setup gui.
  gui3D_initialize(argc, argv, 10, 10, 720, 480, 10.0);
  gui3D_setCameraParams(0.2 * .05, 0.5, 0.001, 10.5, 30, 1, 60000);
  gui3D_set_displayFunc(display);
  gui3D_set_motionFunc(motion);
  gui3D_set_mouseFunc(mouse);
  gui3D_set_keyboardFunc(keyboard);
  gui3D_add_timerFunc(100, timer, 0);
  gui3D_setInitialCameraPos(0, -1, 15, 0, 0, 0);
  g_passat = passatwagonmodel_load(0.0, 0.0, 0.5, 1);

  gui3D_mainloop();
  return 0;
}

