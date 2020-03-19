#include <segmentation_and_tracking/hand_segmenter_view_controller.h>

using namespace std;
using namespace Eigen;
namespace bfs = boost::filesystem;
  

string usageString()
{
  ostringstream oss;
  oss << "Usage: hand_segmenter SEQUENCE_DIR" << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc != 2) {
    cout << usageString() << endl;
    return 1;
  }

  string path = argv[1];
  if(!bfs::exists(path)) {
    cout << path << " does not exist." << endl;
    return 2;
  }

  double scale = 1.0;
  if(getenv("SCALE"))
    scale = atof(getenv("SCALE"));
  OpenCVView view("Image", scale);
  HandSegmenterViewController vc(&view, path);
  view.setDelegate(&vc);
  vc.run();
  
  return 0;
}
