#include "CvSquareLatticeWalker.h"
#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace CvWord;

int main(int argc, char** argv)
{
	IplImage* I   = cvLoadImage(argv[1], 1);
	IplImage* hsv = cvCreateImage(cvGetSize(I), IPL_DEPTH_8U, 3);
	IplImage* h   = cvCreateImage(cvGetSize(I), IPL_DEPTH_8U, 1);
	IplImage* s   = cvCreateImage(cvGetSize(I), IPL_DEPTH_8U, 1);
	IplImage* v   = cvCreateImage(cvGetSize(I), IPL_DEPTH_8U, 1);
	IplImage* t   = cvCreateImage(cvGetSize(I), IPL_DEPTH_8U, 1);

	cvCvtColor(I, hsv, CV_BGR2HSV);
	cvSplit(hsv, h, s, v, NULL);
	cvCmpS(v, 225, t, CV_CMP_GE);

	cvSaveImage("505.patch.threshold.png", t);

	CvSquareLatticeWalker walker(t);
	std::list<region> rList;

	while (walker.size() > 0)
	{
		walker.ProcessNextBoundary();
		
		region r = {
			walker.GetBoundary(),
			walker.GetRelativeWord(),
			walker.GetAbsoluteWord()
		};

		printf("%ld\n", r.uv.size());

		rList.push_back(r);
	}
	
	cvReleaseImage(&I);
	cvReleaseImage(&hsv);
	cvReleaseImage(&h);
	cvReleaseImage(&s);
	cvReleaseImage(&v);
	cvReleaseImage(&t);

	return 0;
}
