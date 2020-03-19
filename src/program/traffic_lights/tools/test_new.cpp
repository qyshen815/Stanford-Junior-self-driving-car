//#include "CvSquareLatticeWalker.h"
#include "ShapeTables.h"
#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

int main(int argc, char** argv)
{
	const int DATA_SIZE = 100000;
	char hsv_data[DATA_SIZE];
	char h_data[DATA_SIZE];
	char s_data[DATA_SIZE];
	char v_data[DATA_SIZE];
	char t_data[DATA_SIZE];
	char r_data[DATA_SIZE];
	char g_data[DATA_SIZE];
	char y_data[DATA_SIZE];

	IplImage* I   = cvLoadImage(argv[1], 1);
	IplImage* hsv = cvCreateImageHeader(cvGetSize(I), IPL_DEPTH_8U, 3);
	IplImage* h   = cvCreateImageHeader(cvGetSize(I), IPL_DEPTH_8U, 1);
	IplImage* s   = cvCreateImageHeader(cvGetSize(I), IPL_DEPTH_8U, 1);
	IplImage* v   = cvCreateImageHeader(cvGetSize(I), IPL_DEPTH_8U, 1);
	IplImage* t   = cvCreateImageHeader(cvGetSize(I), IPL_DEPTH_8U, 1);
	IplImage* r   = cvCreateImageHeader(cvGetSize(I), IPL_DEPTH_8U, 1);
	IplImage* g   = cvCreateImageHeader(cvGetSize(I), IPL_DEPTH_8U, 1);
	IplImage* y   = cvCreateImageHeader(cvGetSize(I), IPL_DEPTH_8U, 1);

	hsv->imageData = hsv_data;
	h->imageData = h_data;
	s->imageData = s_data;
	v->imageData = v_data;
	t->imageData = t_data;
	r->imageData = r_data;
	g->imageData = g_data;
	y->imageData = y_data;

	cvCvtColor(I, hsv, CV_RGB2HSV);
	cvSplit(hsv, h, s, v, NULL);
	//cvCmpS(s, 225, t, CV_CMP_GE);

	uint8_t red_hue_data[256];
	uint8_t grn_hue_data[256];
	uint8_t ylw_hue_data[256];
	uint8_t saturation_data[256];
	
	CvMat* red_hue_lut    = cvCreateMatHeader(256,1,CV_8UC1);
	CvMat* grn_hue_lut    = cvCreateMatHeader(256,1,CV_8UC1);
	CvMat* ylw_hue_lut    = cvCreateMatHeader(256,1,CV_8UC1);
	CvMat* saturation_lut = cvCreateMatHeader(256,1,CV_8UC1);

	red_hue_lut->data.ptr = red_hue_data;
	grn_hue_lut->data.ptr = grn_hue_data;
	ylw_hue_lut->data.ptr = ylw_hue_data;
	saturation_lut->data.ptr = saturation_data;

	ShapeTable<uint8_t> reader;
	reader.Load("red_hue_shape_table.uint8");
	reader.Export(red_hue_data, 256);
	reader.Load("grn_hue_shape_table.uint8");
	reader.Export(grn_hue_data, 256);
	reader.Load("ylw_hue_shape_table.uint8");
	reader.Export(ylw_hue_data, 256);
	reader.Load("saturation_shape_table.uint8");
	reader.Export(saturation_data, 256);

	cvLUT(h,r,red_hue_lut);
	cvLUT(h,g,grn_hue_lut);
	cvLUT(h,y,ylw_hue_lut);
	cvCmpS(s,225,t,CV_CMP_GE);

	memset(h_data,0,DATA_SIZE);
	memset(s_data,0,DATA_SIZE);
	memset(v_data,0,DATA_SIZE);

	cvCopy(r,h,t);
	cvCopy(g,s,t);
	cvCopy(y,v,t);

	cvSaveImage("r.png", h);
	cvSaveImage("g.png", s);
	cvSaveImage("y.png", v);
	cvSaveImage("t.png", t);

	cvReleaseMatHeader(&red_hue_lut);
	cvReleaseMatHeader(&grn_hue_lut);
	cvReleaseMatHeader(&ylw_hue_lut);
	cvReleaseMatHeader(&saturation_lut);
	
//	cvSaveImage("505.patch.threshold.png", t);

/*	CvSquareLatticeWalker walker(t);
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
*/
	cvReleaseImage(&I);
	cvReleaseImageHeader(&hsv);
	cvReleaseImageHeader(&h);
	cvReleaseImageHeader(&s);
	cvReleaseImageHeader(&v);
	cvReleaseImageHeader(&t);
	cvReleaseImageHeader(&r);
	cvReleaseImageHeader(&g);
	cvReleaseImageHeader(&y);

	return 0;
}
