#include <image_labeler/image_label_manager.h>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
namespace bfs = boost::filesystem;

int main(int argc, char** argv)
{
  string root = ".";

  // -- Get the number of images.
  size_t num_imgs = 0;
  bfs::directory_iterator end_itr;
  for(bfs::directory_iterator itr(root); itr != end_itr; ++itr)
    if((itr->path().extension().compare(".png") == 0) | 
       (itr->path().extension().compare(".jpg") == 0))
      ++num_imgs;

  // -- Get the sorted scene names.
  size_t idx = 0;
  vector<string> names(num_imgs);
  for(bfs::directory_iterator itr(root); itr != end_itr; ++itr) {
    if((itr->path().extension().compare(".png") == 0) | 
       (itr->path().extension().compare(".jpg") == 0)) {
      string filename = itr->path().filename();
      names[idx] = filename;
      ++idx;
    }
  }
  sort(names.begin(), names.end());

  int skip = 1;
  if(getenv("SKIP"))
    skip = atoi(getenv("SKIP"));
  double scale = 1.0;
  if(getenv("SCALE"))
    scale = atof(getenv("SCALE"));
  
  cv::Mat img;
  cv::Mat scaled;
  for(size_t i = 0; i < names.size(); ++i) {
    if(i % skip != 0)
      continue;
    cout << names[i] << endl;

    img = cv::imread(names[i]);

    if(scale == 1.0)
      scaled = img;
    else 
      cv::resize(img, scaled, cv::Size(), scale, scale);
    
    cv::imshow("test", scaled);
    char key = cv::waitKey(30);
    if(key == 'q')
      break;
  }
  
  return 0;
}
