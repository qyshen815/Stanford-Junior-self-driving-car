//#include <opencv2/imgproc/imgproc.hpp>

#include <segmentation_and_tracking/scene.h>

using namespace std;
namespace bfs = boost::filesystem;

string usageString()
{
  ostringstream oss;
  oss << "Usage: view_data DATA_DIR" << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc != 2) { 
    cout << usageString();
    return 1;
  }

  string dirpath = argv[1];
  
  if(!bfs::exists(dirpath)) {
    cout << dirpath << " does not exist." << endl;
    return 1;
  }

  Sequence seq(dirpath);
  for(size_t i = 0; i < seq.size(); ++i) {
    Scene& sc = *seq.getScene(i);
    cv::Mat overlay = sc.getDepthOverlay();
    cv::imshow("test", overlay);
    cv::waitKey(0);
  }

  return 0;
}
