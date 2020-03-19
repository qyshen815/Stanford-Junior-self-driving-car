#include <maxflow/graph.h>
#include <image_labeler/opencv_view.h>
#include <bag_of_tricks/high_res_timer.h>
#include <sstream>
#include <iostream>
#include <set>
#include <Eigen/Eigen>
#include <eigen_extensions/eigen_extensions.h>
#include <dst/sequence_segmentation_view_controller.h>
#include <dst/segmentation_pipeline.h>

#define SCALE (getenv("SCALE") ? atof(getenv("SCALE")) : 1.0)

using namespace std;
using namespace Eigen;
using namespace dst;
using namespace pipeline2;

string usageString()
{
  ostringstream oss;
  oss << "Usage: kinect_cut_interactive SEQUENCE [WEIGHTS]" << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc < 2) {
    cout << usageString() << endl;
    return 0;
  }

  KinectSequence::Ptr seq(new KinectSequence());
  seq->load(argv[1]);
  ROS_ASSERT(seq->images_.size() == seq->pointclouds_.size());

  SequenceSegmentationViewController ssvc(seq);
  ssvc.img_view_.scale_ = SCALE;
  ssvc.seg_view_.scale_ = SCALE;

  if(argc == 3) {
    VectorXd weights;
    eigen_extensions::load(argv[2], &weights);
    cout << "Loaded weights: " << endl << weights.transpose() << endl;
    ssvc.setWeights(weights);
  }

  ssvc.run();

  return 0;
}
