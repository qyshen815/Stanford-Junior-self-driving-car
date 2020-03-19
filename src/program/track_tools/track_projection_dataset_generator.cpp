#include <iomanip>

#include <track_manager.h>
#include <ladybug_playback.h>
#include <ladybug_model.h>
#include <param_interface.h>
#include <ipc_std_interface.h>
#include <image_labeler/image_label_manager.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace Eigen;
using boost::shared_ptr;
using namespace track_manager;
using namespace std;
using namespace vlr;
using namespace dgc;

#define PADDING 10
#define SKIP (getenv("SKIP") ? atoi(getenv("SKIP")) : 50)

char *g_intrinsics_path;
char *g_extrinsics_path;
double g_ladybug_sync_offset;
double g_ladybug_synchronizer_angle;


bool getPointsInImage(int cam_num, const Frame& frame, const LadybugModel& lbmodel,
		      MatrixXi* img_pts) { 
  MatrixXd velo_pts;
  if(!frame.getCloudInVeloCoords(&velo_pts)) {
    cout << "Warning: No points in frame." << endl;
    return false;
  }
  
  if(!lbmodel.projectVeloToWarpedImage(cam_num, velo_pts, img_pts)) { 
    //cerr << "Warning: none of the " << velo_pts.rows() << " velo points in the frame projected into the image." << endl;
    return false;
  }
  
  return true;
}

shared_ptr<CvRect> interpolateBoundingBox(const CvRect& r0,
					  const CvRect& r1,
					  double interpolation)
{
  CvRect* r = new CvRect;
  r->x = r1.x*interpolation + r0.x*(1-interpolation);
  r->y = r1.y*interpolation + r0.y*(1-interpolation);
  r->width = r1.width*interpolation + r0.width*(1-interpolation);
  r->height = r1.height*interpolation + r0.height*(1-interpolation);
  shared_ptr<CvRect> rect(r);
  return rect;
}

void padBoundingBox(int pad, const IplImage* img, CvRect* box)
{
  assert(box);
  assert(img);
  
  box->x = box->x - pad/2;
  box->width = box->width + pad;
  box->y = box->y - pad/2;
  box->height = box->height + pad;

  if(box->x < 0)
    box->x = 0;
  if(box->y < 0)
    box->y = 0;
  if(box->width + box->x >= img->width)
    box->width = img->width - box->x - 1;
  if(box->height + box->y >= img->height)
    box->height = img->height - box->y - 1;
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

void read_parameters(ParamInterface *pint, int argc, char **argv) {
  Param params[] = {
    {"ladybug", "intrinsics_path", DGC_PARAM_FILENAME, &g_intrinsics_path, 0, NULL},
    {"ladybug", "extrinsics_path", DGC_PARAM_FILENAME, &g_extrinsics_path, 0, NULL},
    {"ladybug", "default_sync_offset", DGC_PARAM_DOUBLE, &g_ladybug_sync_offset, 0, NULL},
    {"ladybug", "synchronizer_angle", DGC_PARAM_DOUBLE, &g_ladybug_synchronizer_angle, 0, NULL},
  };

  pint->InstallParams(argc, argv, params, sizeof(params)/sizeof(params[0]));
}


void displayForDebug(const ImageLabelManager& dataset)
{
  int idx = dataset.size() - 1;

  cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
  cout << "Dataset contains " << dataset.size() << " images." << endl;
  cout << dataset.getPathForImage(idx) << endl;
  
  IplImage* labeled = dataset.getLabeledImage(idx);
  cvShowImage("labeled", labeled);
  cvWaitKey(100);
  cvReleaseImage(&labeled);

  vector<Label> labels = dataset.getLabelsForImage(idx);
  for(size_t i = 0; i < labels.size(); ++i) 
    labels[i].serialize(cout);
  cout << endl;

  
}
      

void testOpencv(const ImageLabelManager& dataset)
{
  IplImage* img = dataset.getRawImage(0);
  try {
    cvSaveImage("opencv_test.jpg", img);
    cout << "OpenCV jpg write succeeded." << endl;
  }
  catch(...) {
    cout << "OpenCV jpg write failed." << endl;
  }

  CvVideoWriter* writer = cvCreateVideoWriter("opencv_test.avi",
					      CV_FOURCC('I', '4', '2', '0'),
					      10, cvSize(640, 480), true);
  if(!writer)
    cout << "OpenCV video write failed." << endl;
  else
    cout << "OpenCV video write succeeded." << endl;
  cvReleaseVideoWriter(&writer);
}

int main(int argc, char** argv)
{
  if(argc != 5) {
    cout << "usage: " << argv[0] << " TRACK_MANAGER LADYBUG_LOG CAMERA_ID OUTPUT_DIRECTORY" << endl;
    return 1;
  }
  string tm_path = argv[1];
  string llf_path = argv[2];
  int cam_id = atoi(argv[3]);
  string run_name = argv[4];  

  // -- Set up the dataset.
  if(boost::filesystem::exists(run_name)) {
    cout << run_name << " already exists, aborting." << endl;
    ImageLabelManager dataset(run_name);
    testOpencv(dataset);
    return 1;
  }
  ImageLabelManager dataset(run_name);
  
  // -- Connect to IPC and get params.
  IpcInterface *ipc = new IpcStandardInterface();
  ParamInterface *pint = new ParamInterface(ipc);
  if(ipc->ConnectLocked("calibration_viewer") < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  read_parameters(pint, argc, argv);
  delete ipc;
  delete pint;
  
  // -- Load the tracks.
  cout << "Loading Track Mananger... " << tm_path << endl;
  TrackManager tm(tm_path);
  cout << " loaded " << tm.tracks_.size() << " tracks." << endl;

//   // -- Drop all the background and unlabeled tracks.
//   vector< shared_ptr<Track> > foreground;
//   foreground.reserve(all.tracks_.size());
//   for(size_t i = 0; i < all.tracks_.size(); ++i) {
//     Track& tr = *all.tracks_[i];
//     if(tr.label_.compare("background") == 0 ||
//        tr.label_.compare("unlabeled") == 0)
//       continue;
//     foreground.push_back(all.tracks_[i]);
//   }
//   TrackManager tm(foreground);
    
  // -- Load the ladybug log.
  cout << "Loading Ladybug Log File " << llf_path << endl;
  LadybugPlayback lbp;
  lbp.openLLF(argv[2]); // stupid old c-style functions

  // -- Skip ahead a bit.
  for(int i = 0; i < SKIP; ++i)
    lbp.readNextPacket();
  
  // -- Load the ladybug model.
  cout << "Loading Ladybug intrinsics at " << g_intrinsics_path << endl;
  cout << "Loading Ladybug extrinsics at " << g_extrinsics_path << endl;
  LadybugModel lbmodel(g_extrinsics_path, g_intrinsics_path);
  cout << "Using camera " << cam_id << endl;
  cout << lbmodel.intrinsics_.cameras_[cam_id].status() << endl;


  // -- Main loop.
  int count = 0;
  while(true) { 
    // -- Get the next image.
    if(!lbp.readNextPacket())
      break;
    dgc_image_t* dgcimg = lbp.cameraImage(cam_id);
    IplImage* img = dgcToIpl(*dgcimg);
    
    // -- Find the closest cluster for each track and add to the set of labels.
    vector<Label> labels;
    for(size_t i = 0; i < tm.tracks_.size(); ++i) {
      Track& tr = *tm.tracks_[i];
      size_t idx = 0;
      double offset_ladybug_timestamp = lbp.getTimestamp() - g_ladybug_sync_offset; // TODO: Fix timing offsets!  How?
      //double max_time_difference = 0.1;
      double max_time_difference = 0.05;
      
      double interpolation = -1;
      if(tr.interpolatedSeek(offset_ladybug_timestamp, max_time_difference, &idx, &interpolation)) {
	shared_ptr<CvRect> r0 = getFrameBoundingBox(cam_id, *tr.frames_[idx], lbmodel);
// 	shared_ptr<CvRect> r1 = getFrameBoundingBox(g_base_cam, *tr.frames_[idx+1], lbmodel);
// 	shared_ptr<CvRect> box((CvRect*)NULL);
// 	if(r0 && r1)
// 	  box = interpolateBoundingBox(*r0, *r1, interpolation);
// 	else if(r0)
// 	  box = r0;
// 	else if(r1)
// 	  box = r1;
	
	if(r0) {
	  padBoundingBox(PADDING, img, r0.get());
	  if(tr.label_.compare("unlabeled") != 0) { 
	    labels.push_back(Label(i, tr.label_, r0->x, r0->y, r0->width, r0->height));
	  }
	}
      }
    }

    // -- Add the labeled image to the dataset.
    ostringstream oss;
    oss << setprecision(16) << "cam" << cam_id << "_" << lbp.getTimestamp() << ".png";
    string img_name = oss.str();
    dataset.addLabeledImage(img_name, img, labels);
    cvReleaseImage(&img);

    // -- Test.
    if(count % 10 == 0) { 
      displayForDebug(dataset);
    }
    ++count;
  }
  
  return 0;
}

