#ifndef TRAFFIC_LIGHT_HISTOGRAM_FILTER_TL_H
#define TRAFFIC_LIGHT_HISTOGRAM_FILTER_TL_H

#include <cv.h>
#include <highgui.h>
#include <cxtypes.h>
#include <cxcore.h>
#include "detection_grid.h"

class HistogramFilterTL
{
public:
	HistogramFilterTL();
	HistogramFilterTL(int width, int height);
	void UpdateHistogramFilterTL(DetectionGrid *dg, double curr_speed, double detection_grid_spacing);
	~HistogramFilterTL();
	void setMeasGain(double exponent);
	//image access methods (mostly for debug viewer)
	IplImage *getGridPrior();
	IplImage *getWinnersImage();
	void Initialize();
private:
	void NormalizeGrid(double sum);
	void BlurPrior(double curr_speed, double detection_grid_spacing);
	void setWinnersImage(char state, float score, int x, int y);
	IplImage *grid_prior_;
	IplImage *winners_image_;
	int grid_prior_width_;
	int grid_prior_height_;
	int measurement_exponent_;

};

#endif //TRAFFIC_LIGHT_HISTOGRAM_FILTER_TL_H
