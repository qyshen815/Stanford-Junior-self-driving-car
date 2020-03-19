#include <iomanip>

#include <track_manager.h>
#include <ladybug_playback.h>
#include <ladybug_model.h>
#include <param_interface.h>
#include <ipc_std_interface.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <boost/filesystem.hpp>

using namespace Eigen;
using boost::shared_ptr;
using namespace track_manager;
using namespace std;
using namespace vlr;
using namespace dgc;
namespace bfs = boost::filesystem;

int g_line_thickness = 3;

char *g_intrinsics_path;
char *g_extrinsics_path;
double g_ladybug_sync_offset;
double g_ladybug_synchronizer_angle;
int g_base_cam;

bool getPointsInImage(int cam_num, const Frame& frame, const LadybugModel& lbmodel,
		      MatrixXi* img_pts) { 
  MatrixXd velo_pts;
  if(!frame.getCloudInVeloCoords(&velo_pts)) {
    cerr << "Warning: No points in frame." << endl;
    return false;
  }
  
  if(!lbmodel.projectVeloToWarpedImage(cam_num, velo_pts, img_pts)) { 
    //cerr << "Warning: none of the " << velo_pts.rows() << " velo points in the frame projected into the image." << endl;
    return false;
  }
  
  return true;
}


CvScalar getColor(const string& label) { 
  CvScalar color = cvScalar(rand()%255, rand()%255, rand()%255);  
  if(label.compare("pedestrian") == 0)
    color = cvScalar(255, 0, 0);
  else if(label.compare("car") == 0)
    color = cvScalar(0, 0, 255);
  else if(label.compare("bicyclist") == 0)
    color = cvScalar(0, 255, 0);
  else if(label.compare("background") == 0)
    color = cvScalar(127, 127, 127);
  else {
    color = cvScalar(127, 127, 127);
    //assert(false);
  }
  return color;
}

void drawInterpolatedBoundingBox(IplImage* img,
				 const string& label,
				 const CvRect& r0,
				 const CvRect& r1,
				 double interpolation)
{
  CvRect rect = cvRect(r1.x*interpolation + r0.x*(1-interpolation),
		       r1.y*interpolation + r0.y*(1-interpolation),
		       r1.width*interpolation + r0.width*(1-interpolation),
		       r1.height*interpolation + r0.height*(1-interpolation));
  cvRectangle(img, cvPoint(rect.x, rect.y), cvPoint(rect.x + rect.width, rect.y + rect.height), getColor(label), g_line_thickness);
}
  
//! Returns NULL if no box.
shared_ptr<CvRect> getFrameBoundingBox(int cam_num,
				       const Frame& frame,
				       const LadybugModel& lbmodel)
{
  shared_ptr<CvRect> box((CvRect*)NULL);
  MatrixXi img_pts;
  if(!getPointsInImage(cam_num, frame, lbmodel, &img_pts))
    return box;
 
  // -- Get the bounding box.
  int max_u = 0, max_v = 0;
  int min_u = 1e6, min_v = 1e6;
  for(int i = 0; i < img_pts.rows(); ++i) {
    int u = img_pts(i, 0);
    int v = img_pts(i, 1);

    if(u > max_u)
      max_u = u;
    if(v > max_v)
      max_v = v;
    if(u < min_u)
      min_u = u;
    if(v < min_v)
      min_v = v;
  }
    
  box = shared_ptr<CvRect>(new CvRect);
  box->x = min_u;
  box->width = max_u - min_u;
  box->y = min_v;
  box->height = max_v - min_v;
  return box;
}

int drawClusterInImage(IplImage* img,
		       int cam_num,
		       const Frame& frame,
		       const string& label,
		       const LadybugModel& lbmodel)
{
  assert(img->nChannels == 3);
  
  // -- Project the points into the image.
  CvScalar color = getColor(label);
  int num_drawn = 0;
  int max_u = 0, max_v = 0;
  int min_u = 1e6, min_v = 1e6;
  for(size_t i = 0; i < frame.cloud_->get_points_size(); ++i) {
    Vector4f smooth_pt;
    smooth_pt(0) = frame.cloud_->points[i].x;
    smooth_pt(1) = frame.cloud_->points[i].y;
    smooth_pt(2) = frame.cloud_->points[i].z;
    smooth_pt(3) = 1;
    Vector4f velo_pt = smooth_pt.transpose() * frame.smooth_to_velo_;
    assert(fabs(velo_pt(3) - 1) < 1e-6);

    int u = 0;
    int v = 0;
    bool in_img = lbmodel.projectVeloToWarpedImage(cam_num, velo_pt(0), velo_pt(1), velo_pt(2), &u, &v);
    if(!in_img)
      continue;

    if(u > max_u)
      max_u = u;
    if(v > max_v)
      max_v = v;
    if(u < min_u)
      min_u = u;
    if(v < min_v)
      min_v = v;
    
    if(u < img->width && u >= 0 && v <= img->height-1 && v >= 0) //stupid editor parsing bug.
      ++num_drawn;

    if(getenv("DRAW_POINTS"))
      cvCircle(img, cvPoint(u, v), 1, color, -1);
  }
  if(num_drawn > 0)
    cvRectangle(img, cvPoint(max_u, max_v), cvPoint(min_u, min_v), color, g_line_thickness);

  return num_drawn;
}

void drawConvexHull(IplImage* img, const MatrixXi& img_pts, const string& label) {
  assert(img_pts.cols() == 2);

    
  // -- Create the CvMat of image points.
  CvPoint* points = (CvPoint*) malloc(img_pts.rows() * sizeof(points[0]));
  CvMat pts_mat = cvMat(1, img_pts.rows(), CV_32SC2, points);
  for(int i = 0; i < img_pts.rows(); ++i) {
    CvPoint pt;
    pt.x = img_pts(i, 0);
    pt.y = img_pts(i, 1);
    points[i] = pt;
    if(getenv("DRAW_POINTS"))
      cvCircle(img, cvPoint(pt.x, pt.y), 1, getColor(label), -1);
      

  }

  // -- Find the convex hull.
  int* hull = (int*)malloc(img_pts.rows() * sizeof(hull[0])); 
  CvMat hull_mat = cvMat(1, img_pts.rows(), CV_32SC1, hull);
  cvConvexHull2(&pts_mat, &hull_mat, CV_CLOCKWISE, 0);
  
  // -- Draw it in the image.
  CvPoint pt0 = points[hull[hull_mat.cols - 1]];
  for(int i = 0; i < hull_mat.cols; ++i) {
    CvPoint pt = points[hull[i]];
    cvLine(img, pt0, pt, getColor(label), g_line_thickness, CV_AA, 0);
    pt0 = pt;
  }

  // -- Clean up.
  free(points);
  free(hull);
}

void read_parameters(ParamInterface *pint, int argc, char **argv) {
  Param params[] = {
    {"ladybug", "intrinsics_path", DGC_PARAM_FILENAME, &g_intrinsics_path, 0, NULL},
    {"ladybug", "extrinsics_path", DGC_PARAM_FILENAME, &g_extrinsics_path, 0, NULL},
    {"ladybug", "base_cam", DGC_PARAM_INT, &g_base_cam, 0, NULL},
    {"ladybug", "default_sync_offset", DGC_PARAM_DOUBLE, &g_ladybug_sync_offset, 0, NULL},
    {"ladybug", "synchronizer_angle", DGC_PARAM_DOUBLE, &g_ladybug_synchronizer_angle, 0, NULL},
  };

  pint->InstallParams(argc, argv, params, sizeof(params)/sizeof(params[0]));
}

string usageString()
{
  ostringstream oss;
  oss << "usage: track_projector TRACK_MANAGER LADYBUG_LOG IMAGE_DIR" << endl;
  return oss.str();
}

int main(int argc, char** argv) {
  if(argc != 4) {
    cout << usageString() << endl;
    return 1;
  }  
  string track_manager_path = argv[1];
  string image_dir = argv[3];
    
  // -- Set up image dump.
  if(bfs::exists(image_dir)) {
    cout << image_dir << " already exists, will not overwrite." << endl;
    return -1;
  }
  bfs::create_directory(image_dir);
  cout << "Output images will be saved in " << image_dir << endl;
  
  // -- Load track manager.
  if(bfs::path(track_manager_path).extension().compare(".tm") != 0) {
    cout << "Track manager " << track_manager_path << " must have .tm extension." << endl;
    cout << usageString() << endl;
    return 1;
  }

  // -- Connect to IPC and get params.
  IpcInterface *ipc = new IpcStandardInterface();
  ParamInterface *pint = new ParamInterface(ipc);
  if(ipc->ConnectLocked("calibration_viewer") < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  read_parameters(pint, argc, argv);
  delete ipc;
  delete pint;

  // -- Load the logs.
  cout << "Loading Track Mananger... " << track_manager_path << endl;
  TrackManager tm(track_manager_path);
  cout << " loaded " << tm.tracks_.size() << " tracks." << endl;
  
  cout << "Loading Ladybug Log File " << argv[2] << endl;
  LadybugPlayback lbp;
  lbp.openLLF(argv[2]);

  // -- Load the ladybug model.
  cout << "Loading Ladybug intrinsics at " << g_intrinsics_path << endl;
  cout << "Base cam is " << g_base_cam << ", extrinsics at " << g_extrinsics_path << endl;
  LadybugModel lbmodel(g_base_cam, g_extrinsics_path, g_intrinsics_path);
  cout << lbmodel.intrinsics_.cameras_[g_base_cam].status() << endl;

  // -- Skip the first few frames, as they tend to be bad for our logs.
  int skip = 50;
  for(int i = 0; i < skip; ++i) {
    lbp.readNextPacket();
  }

  cvNamedWindow("Image");
  int ctr = 0;
  while(true) { 
    cout << "Frame " << ctr << endl;
    ++ctr;
    
    // -- Get the next image.
    if(!lbp.readNextPacket())
      break;
    dgc_image_t* dgcimg = lbp.cameraImage(g_base_cam);
    IplImage* img = dgcToIpl(*dgcimg);
    IplImage* rect = lbmodel.intrinsics_.cameras_[g_base_cam].rectify(img);
	
    // -- Find the closest cluster for each track and draw it in the image.
    for(size_t i = 0; i < tm.tracks_.size(); ++i) {
      Track& tr = *tm.tracks_[i];
      if(tr.label_.compare("background") == 0 && !getenv("DRAW_BACKGROUND"))
	continue;
      
      size_t idx = 0;
      double offset_ladybug_timestamp = lbp.getTimestamp() - g_ladybug_sync_offset; // TODO: Fix timing offsets!  How?
      double max_time_difference = 0.07;
      
      if(getenv("DRAW_BBOX") && tr.seek(offset_ladybug_timestamp, max_time_difference, &idx)) {
	drawClusterInImage(img, g_base_cam, *tr.frames_[idx], tr.label_, lbmodel);
      }
      else if(getenv("DRAW_HULL") && tr.seek(offset_ladybug_timestamp, max_time_difference, &idx)) { 
	MatrixXi img_pts;
	if(getPointsInImage(g_base_cam, *tr.frames_[idx], lbmodel, &img_pts)) {
// 	  IplImage* vis = cvCloneImage(img);
// 	  drawConvexHull(vis, img_pts, tr.label_);
// 	  cvShowImage("tmp", vis);
// 	  cout << "Timestamp: " << tr.frames_[idx]->timestamp_
// 	       << ", Adjusted timestamp: " << tr.frames_[idx]->estimateAdjustedTimestamp()
// 	       << ", adjusted - orig: " << tr.frames_[idx]->estimateAdjustedTimestamp() - tr.frames_[idx]->timestamp_ << endl;
// 	  cvWaitKey();
	  drawConvexHull(img, img_pts, tr.label_);
	}
      }
      else if(getenv("DRAW_INTERPOLATED_BBOX")) { 
	double interpolation = -1;
	if(tr.interpolatedSeek(offset_ladybug_timestamp, max_time_difference, &idx, &interpolation)) {
	  shared_ptr<CvRect> r0 = getFrameBoundingBox(g_base_cam, *tr.frames_[idx], lbmodel);
	  shared_ptr<CvRect> r1 = getFrameBoundingBox(g_base_cam, *tr.frames_[idx+1], lbmodel);
	  if(r0 && r1)
	    drawInterpolatedBoundingBox(img, tr.label_, *r0, *r1, interpolation);
	  else if(r0)
	    drawInterpolatedBoundingBox(img, tr.label_, *r0, *r0, 0);
	  else if(r1)
	    drawInterpolatedBoundingBox(img, tr.label_, *r1, *r1, 1);
	}
      }
	
    }
  
    // -- Display.
    cvShowImage("Image", img);

    // -- Save the image.
    ostringstream path;
    path << image_dir << "/" << setprecision(16) << lbp.getTimestamp() << "_cam" << g_base_cam << ".png";
    cvSaveImage(path.str().c_str(), img);
    
    // -- Clean up.
    cvReleaseImage(&rect);
    cvReleaseImage(&img);
    
    char key = cvWaitKey(10);
    if(key == 'q')
      break;
  }
  
  return 0;
}

