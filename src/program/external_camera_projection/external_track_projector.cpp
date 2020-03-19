#include "external_camera_model.h"
#include <image_labeler/image_label_manager.h>
#include <track_manager.h>
#include <bag_of_tricks/bag_of_tricks.h>
#include <multibooster_support.h>

using namespace Eigen;
using namespace std;
namespace bfs = boost::filesystem;
using namespace track_manager;
using boost::shared_ptr;

string usageString()
{
  ostringstream oss;
  oss << "Usage: external_track_projector FRAME_CLASSIFIER INTRINSICS VIDEO LABELED_TRACK_MANAGER OUTPUT_PATH" << endl;
  oss << "  VIDEO must end with .avi." << endl;
  oss << "  Required files: " << endl;
  oss << "    VIDEO" << endl;
  oss << "    VIDEO.timestamp" << endl;
  oss << "    VIDEO.sync_offset.eig" << endl;
  oss << "    VIDEO.extrinsics.eig" << endl;
  oss << "  OUTPUT_PATH will be created as a directory in which an ImageLabelManager lives." << endl;
  return oss.str();
}

bool getFrameLabel(const ExternalCameraModel& ecm, int track_id, const Frame& frame, const string& class_name, Label* label)
{ 
  // -- Put the points in the camera coordinate system.
  sensor_msgs::PointCloud& cloud = *frame.cloud_;
  MatrixXd camera(4, cloud.get_points_size());
  for(size_t i = 0; i < cloud.get_points_size(); ++i) {
    camera(0, i) = cloud.points[i].x;
    camera(1, i) = cloud.points[i].y;
    camera(2, i) = cloud.points[i].z;
    camera(3, i) = 1;
  }
  camera = ecm.transform_ * camera;

  // -- Get the points in the camera image.
  vector<cv::Point2d> uvs;
  uvs.reserve(cloud.get_points_size());
  bool valid = false;
  for(int i = 0; i < camera.cols(); ++i) { 
    cv::Point2d uv;
    bool in_img = ecm.xyzCameraToUVRect(camera.col(i), &uv);
    if(in_img) {
      valid = true;
      uvs.push_back(uv);
    }
  }

  // -- If the object isn't in the image, then we're done.
  if(!valid)
    return false;
  
  cout << cloud.get_points_size() << " " << uvs.size() << endl;
  
  // -- Get the min and max points.
  cv::Point2d minpt;
  minpt.x = FLT_MAX;
  minpt.y = FLT_MAX;
  cv::Point2d maxpt;
  maxpt.x = -FLT_MAX;
  maxpt.y = -FLT_MAX;
  for(size_t i = 0; i < uvs.size(); ++i) {
    double x = uvs[i].x;
    double y = uvs[i].y;
    if(x > maxpt.x)
      maxpt.x = x;
    if(x < minpt.x)
      minpt.x = x;
    if(y > maxpt.y)
      maxpt.y = y;
    if(y < minpt.y)
      minpt.y = y;
  }

  // -- Only return labels that are at least a certain size.
  //    Also, add some padding on either side.
  double dx = maxpt.x - minpt.x;
  double dy = maxpt.y - minpt.y;
  double thresh = 15;
  if(dx < thresh || dy < thresh)
    return false;
  else {
    double pad = 15;
    assert(ecm.raw_.cols > 0);
    assert(ecm.raw_.rows > 0);
    maxpt.x = min(maxpt.x + pad, (double)ecm.raw_.cols - 1.0);
    maxpt.y = min(maxpt.y + pad, (double)ecm.raw_.rows - 1.0);
    minpt.x = max(minpt.x - pad, 0.0);
    minpt.y = max(minpt.y - pad, 0.0);
    *label = Label(track_id, class_name, minpt.x, minpt.y, maxpt.x - minpt.x, maxpt.y - minpt.y);
  }

  // -- Exclude labels that are in areas we don't want to consider, i.e. those that we know are occluded.
  if(ecm.excluded(*label))
    return false;
  else
    return true;

}

void labelImage(ClassifierPipeline& cp, ExternalCameraModel& ecm, const TrackManager& tm, ImageLabelManager* ilm)
{
  // -- Get frames that are close to the image in time.
  double timestamp = ecm.getTimestamp();
  double tol = 0.51 / ecm.getFps();
  //double tol = 0.75*ecm.getFps(); // Debugging...
  cout << "Segment to image timestamp must be less than " << tol << " seconds." << endl;
  vector< shared_ptr<Frame> > frames;
  vector<string> class_names;
  vector<int> track_ids;
  HighResTimer hrt;
  tm.getFramesNear(timestamp, tol, &frames, &class_names, &track_ids);
  hrt.stop();
  cout << "Getting frames near timestamp took " << hrt.getSeconds() << " seconds." << endl;

  // -- Get the labels.
  vector<Label> labels;
  labels.reserve(frames.size());
  for(size_t i = 0; i < frames.size(); ++i) {
    // Only consider objects in the image.
    Label label;
    bool in_img = getFrameLabel(ecm, track_ids[i], *frames[i], class_names[i], &label);
    //cout << class_names[i];
    
    if(!in_img) {
      //cout << " not in image." << endl;
      continue;
    }

    // Only use this frame if the prediction for the frame matches the prediction for the track.
    // This eliminates false positives due to transient segmentation and tracking errors.
    string frame_prediction;
    cp.classify(*frames[i], &frame_prediction);
    if(frame_prediction.compare(tm.tracks_[track_ids[i]]->label_) != 0) {
      //cout << " doesn't match frame prediction." << endl;
      continue;
    }

    //cout << " added." << endl;
    labels.push_back(label);
  } 
	

  // -- Add the labeled image.
  ostringstream oss;
  oss << setfill('0') << setiosflags(ios::fixed) << setprecision(2) << setw(8) << ecm.getRelativeTimestamp() << ".png";
  string filename = oss.str();
  IplImage ipl = IplImage(ecm.rect_);
  ilm->addLabeledImage(filename, &ipl, labels);
  cout << "Saved " << filename << endl;
}

int main(int argc, char** argv)
{

  if(argc != 6) {
    cout << usageString() << endl;
    return 1;
  }

  string classifier_path = argv[1];
  string intrinsics_path = argv[2];
  string video_path = argv[3];
  string tm_path = argv[4];
  string output_path = argv[5];

  assert(!bfs::exists(output_path));

  MultiBooster mb(classifier_path);
  int num_threads = 40;
  ClassifierPipeline cp(&mb, num_threads);
  ExternalCameraModel ecm(intrinsics_path, video_path);
  TrackManager tm(tm_path);
  ImageLabelManager ilm(output_path);

  // -- Seek to a desired start point.
  if(getenv("START_SEC")) {
    double start = atof(getenv("START_SEC"));
    cout << "Seeking start time of " << start << endl;
    while(ecm.getRelativeTimestamp() < start) {
      cout << ecm.getRelativeTimestamp() << " / " << start << endl;
      ecm.advance(1);
    }
  }

  // -- Label the images.
  while(ecm.advance(1)) {
    HighResTimer hrt;
    labelImage(cp, ecm, tm, &ilm);
    hrt.stop();
    cout << "Labeling entire image took " << hrt.getSeconds() << " seconds." << endl;
  }
    
  return 0;
}
