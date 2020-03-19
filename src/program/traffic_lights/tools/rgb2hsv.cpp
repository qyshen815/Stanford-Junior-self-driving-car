#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <stdio.h>

int main(int argc, char** argv)
{
	for (int i = 1; i < argc; i ++)
	{
		try {
			IplImage* img = cvLoadImage(argv[i], 1);
			cvCvtColor(img, img, CV_RGB2HSV);
			cvSaveImage(argv[i], img);
			cvReleaseImage(&img);
		} catch (cv::Exception& e) {
			fprintf(stderr, "Failed to convert %s.\n", argv[i]);
		}
	}

	return 0;
}
