#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <iomanip>
#include <sys/time.h>

using namespace std;

string usageString()
{
  ostringstream oss;
  oss << "Usage: dump_images_from_video VIDEO" << endl;
  oss << "  Pressing enter during the video will save an image to disk." << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc != 2) {
    cout << usageString() << endl;
    return -1;
  }

  // -- Load the video.
  string video_path = argv[1];
  cv::VideoCapture video(video_path);
  if(!video.isOpened()) {
    cout << "Failed to load " << video_path << endl;
    return -2;
  }

  // -- Show the frames.
  cv::Mat img;
  while(video.grab()) {
    video.retrieve(img);
    cv::imshow("Video", img);
    char key = cvWaitKey(33);

    ostringstream oss;
    timeval tv;
    double seconds;
    switch(key) {
    case 'q':
      exit(0);
      break;
    case '\n':
      gettimeofday(&tv, NULL);
      seconds = tv.tv_sec + (double)tv.tv_usec / (double)1e6;
      oss << setprecision(16) << "img" << seconds << ".png";
      cout << "Saving image to " << oss.str() << endl;
      cv::imwrite(oss.str(), img);
      break;
    default:
      break;
    }
  }

  return 0;
}
