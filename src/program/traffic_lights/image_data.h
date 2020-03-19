#ifndef TRAFFIC_LIGHT_IMAGE_DATA_H
#define TRAFFIC_LIGHT_IMAGE_DATA_H

#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <cv.h>
#include <highgui.h>
#include <cxtypes.h>
#include <cxcore.h>
#include "detection_grid.h"

#define MAX_ROI_SIZE MAX_GRID_SIZE_TL*MAX_GRID_SIZE_TL*2  

#include "ShapeTable.h"

struct Color8U
{
	uchar R;
	uchar G;
	uchar B;
};

struct ColorHSV
{
	int16_t H;
	uint8_t S;
	uint8_t V;
};

struct Point
{
	int X;
	int Y;
};

class ImageData
{

public:
	ImageData(bool, float*, ShapeTable<uint8_t>*, ShapeTable<uint8_t>*, ShapeTable<uint8_t>*, ShapeTable<uint8_t>*);
	bool UpdateDataScores(DetectionGrid *dg, IplImage *curr_image, double dist_to_light, bool downsample, bool save = false);
	
	// for debug viewer:
	void getFeaturePatchDimensions(int *ssd_width_dbg, int *ssd_height_dbg);
	IplImage* getCamImagePatch(IplImage * curr_frame, int u_win_data_only_dbg, int v_win_data_only_dbg,
	                           int posterior_u_dbg, int posterior_v_dbg,
	                           char win_state_data_only_dbg, char posterior_state_dbg);
	int getLastTplWidth();
	int getLastTplHeight();
	
	// helper functions:
	static void FindMinMax(int a, int b, int c, int d, int *min, int *max);

private:
	//region of interest within the camera image
	//(usually set as min and max bounds of the projection of detection grid onto the image)
	int min_u_;
	int max_u_;
	int min_v_;
	int max_v_;

	//width and height of the camera image
	int im_width_;
	int im_height_;

	//floats because of IplImage issues with 64F
	float max_ssd_val_;
	float min_ssd_val_;

	//the dimensions of the most recently-used light template
	int last_tpl_width_;
	int last_tpl_height_;

	// shape tables
	ShapeTable<uint8_t> stRedHue_;
	ShapeTable<uint8_t> stYlwHue_;
	ShapeTable<uint8_t> stGrnHue_;
	ShapeTable<uint8_t> stSaturation_;

	ShapeTable<uint8_t> stFrameVal_;

	///// calibration values from param server
	
	// traffic light lens radius = (m_ / x) + b_
	//   where x is distance from camera to lens in meters.
	int m_;
	int b_;
	int n_;
	int c_;

	//major functions:
	IplImage* CreateMasterTemplate(int desired_width);
	IplImage* GetLightTemplate(double dist_to_light, bool downsample);
	IplImage* GetCircleTemplate(double dist_to_light, bool downsample);

	IplImage* ConvolveHue
		(IplImage *tpl, IplImage *img, int frame_weight, int lens_weight, 
		 ShapeTable<uint8_t>* h_table, ShapeTable<uint8_t>* s_table, ShapeTable<uint8_t>* v_table, int );

	IplImage* ConvolveCircleTemplate(IplImage* ssd, double dist_to_light, bool downsample);
	
	void CalculateCellScore(int, int, IplImage *, IplImage *, IplImage *,
	                        std::vector<Point> *, float *, float *, float *);
	
	void CalculatePixelScore(int, int, IplImage *, IplImage *, IplImage *,
	                         float *, float *, float *);
	
	// aka normalize_circle_ssds
	void NormalizeFeatures(IplImage *, IplImage *, IplImage *); 

	void RecalcMinMax(IplImage* a, float& min, float& max);

	//time it takes to perform the weighted SSD on the ROI in this camera frame
	//the time is proportional to size of grid projection the frame and is set by ImageData class methods
	double cpu_time_;

	//for viewing SSDs, if in debug mode
	bool debug_mode_;
	float* feature_patches_;
	int dbg_width_;
	int dbg_height_;
	void WriteToFeaturesDisplayBuffer(IplImage*, IplImage*, IplImage*);
};



#endif // TRAFFIC_LIGHT_IMAGE_DATA_H

