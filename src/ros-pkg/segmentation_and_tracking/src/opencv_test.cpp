#include <segmentation_and_tracking/scene.h>

using namespace std;

int main(int argc, char** argv)
{
  cv::Mat img = cv::imread(argv[1]);
  cv::imshow("test", img);
  cv::waitKey(0);
}
