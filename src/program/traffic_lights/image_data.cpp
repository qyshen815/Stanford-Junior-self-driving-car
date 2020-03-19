#include "image_data.h"

#include <limits>
#include <time.h>
#include <sys/types.h>

ImageData::ImageData
(bool debug_mode, 
 float *patch_buffer_ptr_dbg, 
 ShapeTable<uint8_t>* _red, 
 ShapeTable<uint8_t>* _ylw, 
 ShapeTable<uint8_t>* _grn,
 ShapeTable<uint8_t>* _sat) :
	stRedHue_(*_red), 
	stYlwHue_(*_ylw), 
	stGrnHue_(*_grn),
	stSaturation_(*_sat),
	m_(450.),
	b_(1.),
	n_(20),
	c_(4),
	debug_mode_(debug_mode), 
	feature_patches_(patch_buffer_ptr_dbg)
{
	stFrameVal_.SetParams(0,	255,	255,	-.035,	0.);
	stFrameVal_.GenerateExponentialTable();
	stFrameVal_.InvertValues();
}

IplImage* ImageData::ConvolveHue
(IplImage *tpl, IplImage *img, int frame_weight, int lens_weight, 
 ShapeTable<uint8_t>* h_table, ShapeTable<uint8_t>* s_table, ShapeTable<uint8_t>* v_table, int table_c)
{
	IplImage *diffs = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_32F, 1);

	int halftpl_v = tpl->height / 2;
	int halftpl_u = tpl->width  / 2;

	int start_u, start_v, end_u, end_v;
	start_u = min_u_;
	start_v = min_v_;
	end_u = max_u_;
	end_v = max_v_;
	
	if (min_u_ < halftpl_u + 1) start_u = tpl->width  + 1;
	if (min_v_ < halftpl_v + 1) start_v = tpl->height + 1;

	if (max_u_ > (img->width  - halftpl_u-1)) end_u = img->width  - tpl->width;
	if (max_v_ > (img->height - halftpl_v-1)) end_v = img->height - tpl->height;

	int64_t score;
	int tpl_index_u, tpl_index_v; 
	int img_index_u, img_index_v;
	int v_tpl, h_img, s_img, v_img;
	
	for (int v = start_v; v <= end_v; v++)
	{
		for (int u = start_u; u <= end_u; u++)
		{
			score = 0; //reset features for this pixel
			for (int y = -halftpl_v; y < halftpl_v; y++)
			{
				tpl_index_v = y + halftpl_v;
				img_index_v = v + y;
				uchar* tpl_offset_v = (uchar *)(tpl->imageData + tpl_index_v*tpl->widthStep);
				uchar* img_offset_v = (uchar *)(img->imageData + img_index_v*img->widthStep);
				
				for (int x = -halftpl_u; x < halftpl_u; x++)
				{
					tpl_index_u = x + halftpl_u;
					img_index_u = u + x;

					int tpl_offset_u = tpl_index_u*tpl->nChannels;

					v_tpl = tpl_offset_v[tpl_offset_u + 2];

					int img_offset_u = img_index_u*img->nChannels;

					h_img = img_offset_v[img_offset_u + 0];
					s_img = img_offset_v[img_offset_u + 1];
					v_img = img_offset_v[img_offset_u + 2];

					unsigned char h_score = 0;				
					if(table_c == 1) {
						if((h_img >= 0 && h_img <= 6) || (h_img >= 180 && h_img <= 180))
							h_score = 200;
					}
					if(table_c == 2) {
						if(h_img >= 90 && h_img <= 95)
							h_score = 200;
					}
					if(table_c == 3) {
						if(h_img >= 12 && h_img <= 18)
							h_score = 200;
					}
					score += (v_tpl > 0) ?
							lens_weight  * h_score +  
							//(255 - frame_weight * (*v_table)[v_img]) + 
							frame_weight * (*s_table)[s_img] : 
							frame_weight * (*v_table)[v_img];
				}
			}//end template match loops

			//set features value
			((float *)(diffs->imageData + v*diffs->widthStep))[u] = (float)score;

		}
	}

	return diffs;
}
//*/


IplImage* ImageData::GetCircleTemplate(double dist_to_light, bool downsample)
{

	int side = 100;
	IplImage *tpl = cvCreateImage(cvSize(side, side), IPL_DEPTH_32F, 1);

	int x_center, y_center;
	x_center = y_center = side / 2;
	int radius_sq = 27*27;
	//float sum = 0.0;
	for (int y = 0; y < tpl->height; y++)
	{
		float* _tpl = (float *)(tpl->imageData + y*tpl->widthStep);

		for (int x = 0; x < tpl->width; x++)
			_tpl[x] = (((x - x_center)*(x - x_center) + (y - y_center)*(y - y_center)) < radius_sq) ? 1 : -0.5;
	}

	// width of the entire light (including the frame)
	int dim_width_light = b_ + m_ / dist_to_light;
	if (downsample)
	{
		dim_width_light /=2;
	}
	//based on the radius for the light template formula:
	int new_radius = (c_+n_/dim_width_light);
	if ((new_radius%2)==0)
	{
		new_radius--;
	}

	IplImage *resized = cvCreateImage(cvSize(new_radius, new_radius), IPL_DEPTH_32F, 1);
	cvResize(tpl, resized, CV_INTER_LINEAR);
	cvReleaseImage(&tpl);

	return resized;
}

IplImage* ImageData::ConvolveCircleTemplate(IplImage* ssd, double dist_to_light, bool downsample)
{

	//extend features image borders so that blurring/sharpening doesn't have an artifical boundary
	int im_height = ssd->height;
	int im_width = ssd->width;
	int max_extend = 50; //pixels

	int start_u = min_u_ - max_extend;
	int end_u = max_extend + max_u_;
	int start_v = min_v_ - max_extend;
	int end_v = max_extend + max_v_;

	if (start_u < 0) start_u = 0;
	if (start_v < 0) start_v = 0;
	if (end_v >= im_height) end_v = im_height;
	if (end_u >= im_width) end_u = im_width;

	int near_u, near_v;
	for (int y = start_v; y <end_v; y++)
	{
		float* ssd_row =  ((float *)(ssd->imageData + y*ssd->widthStep));
		
		for (int x = start_u; x < end_u; x++)
		{
			if (x < min_u_ || x > max_u_ || y < min_v_ || y > max_v_)
			{
				if (x < min_u_)
					near_u = min_u_;
				else if (x > max_u_)
					near_u = max_u_;
				else near_u = x;

				if (y < min_v_)
					near_v = min_v_;
				else if (y > max_v_)
					near_v = max_v_;
				else near_v = y;

				float* near_row = ((float *)(ssd->imageData + near_v*ssd->widthStep));

				ssd_row[x] = near_row[near_u];
			}
		}
	}

	// specify the size needed by the match function
	IplImage *tpl = GetCircleTemplate(dist_to_light, downsample);
	IplImage *diffs = cvCreateImage(cvSize(ssd->width, ssd->height), IPL_DEPTH_32F,1);

	int halftpl_v = tpl->height/2;
	int halftpl_u = tpl->width/2;

	int start_u_ssd, start_v_ssd, end_u_ssd, end_v_ssd;

	start_u_ssd = min_u_;
	start_v_ssd = min_v_;
	end_u_ssd = max_u_;
	end_v_ssd = max_v_;

	if (min_u_ < halftpl_u + 1) start_u_ssd = tpl->width + 1;
	if (max_u_ > (ssd->width - halftpl_u-1)) end_u_ssd = ssd->width - tpl->width;
	if (min_v_ < halftpl_v + 1) start_v_ssd = tpl->height + 1;
	if (max_v_ > (ssd->height - halftpl_v-1)) end_v_ssd = ssd->height - tpl->height;

	int tpl_index_u, tpl_index_v, frame_index_u, frame_index_v;
	float tpl_val, ssd_val, sum_pixel;

	for (int v = start_v_ssd; v <= end_v_ssd; v++)
	{
		for (int u = start_u_ssd; u <= end_u_ssd; u++)
		{
			sum_pixel = 0;

			for (int y = -halftpl_v; y < halftpl_v; y++)
			{
				for (int x = -halftpl_u; x < halftpl_u; x++)
				{
					tpl_index_u = x + halftpl_u;
					tpl_index_v = y + halftpl_v;
					frame_index_u = u + x;
					frame_index_v = v + y;

					tpl_val = ((float *)(tpl->imageData + tpl_index_v*tpl->widthStep))[tpl_index_u];
					ssd_val = ((float *)(ssd->imageData + frame_index_v*ssd->widthStep))[frame_index_u];

					sum_pixel += tpl_val*ssd_val;
				}
			}//end template loops

			//set features value to new convolved value
			((float *)(diffs->imageData + v*diffs->widthStep))[u] = sum_pixel;
		}
	}

	cvReleaseImage(&tpl);

	return diffs;

}

// master light lens template
IplImage* ImageData::CreateMasterTemplate(int desired_width) //, bool isArrow)
{
	int len = 56;
	int radius = (desired_width == 0) ? len/3 : c_ + n_/desired_width;
	CvPoint center = { len/2, len/2 };

	IplImage *tpl = cvCreateImage(cvSize(len,len), IPL_DEPTH_8U, 1);

/*	if (isArrow)
	{
		int lineWidth = radius/5;

	}
	else
*/	{
		cvCircle(tpl, center, radius, cvScalar(255), -1);
	}
	cvSmooth(tpl, tpl, CV_GAUSSIAN, 9, 9);

	return tpl;
}

IplImage* ImageData::GetLightTemplate(double dist_to_light, bool downsample)
{
	//calculate size - in pixels- based on distance away (and implicitly based on camera image pixel dimensions)
	int dim_width = round(b_ + m_ / dist_to_light);
	int dim_height = dim_width;

	//dimensions of template should be odd, for opencv acceleration
	if (downsample)
	{
		dim_width /=2;
		dim_height /=2;
	}
	if ((dim_width%2) == 0)
	{
		dim_width++;
	}
	if ((dim_height%2) == 0)
	{
		dim_height++;
	}

	//record the last-used template dimensions
	last_tpl_width_  = dim_width;
	last_tpl_height_ = dim_height;

	IplImage *tpl = CreateMasterTemplate(dist_to_light < 45. ? dim_width : 0);
	IplImage *resized = cvCreateImage(cvSize(dim_width, dim_height), IPL_DEPTH_8U, 1);

	cvResize(tpl, resized, CV_INTER_LINEAR);
	cvReleaseImage(&tpl);
	return resized;

}

// normalize to [0,1]
void ImageData::NormalizeFeatures(IplImage *red_features, IplImage *grn_features, IplImage *ylw_features)
{
	// Normalize featuress
	float min_val = std::numeric_limits<float>::max();
	float max_val = std::numeric_limits<float>::min();
	
	RecalcMinMax(red_features, min_val, max_val);
	RecalcMinMax(grn_features, min_val, max_val);
	RecalcMinMax(ylw_features, min_val, max_val);

	float range = max_val - min_val;

	for (int v = 0; v < red_features->height; v++)
	{
		float* red_row    = ((float *)(red_features->imageData + v*red_features->widthStep));
		float* green_row  = ((float *)(grn_features->imageData + v*grn_features->widthStep));
		float* yellow_row = ((float *)(ylw_features->imageData + v*ylw_features->widthStep));

		for (int u = 0; u < red_features->width; u++)
		{
			red_row[u]    = (red_row[u]    - min_val)/range;
			green_row[u]  = (green_row[u]  - min_val)/range;
			yellow_row[u] = (yellow_row[u] - min_val)/range;
		}
	}
}

void ImageData::FindMinMax(int a, int b, int c, int d, int *min, int *max)
{
	*min = std::min(a, std::min(b, std::min(c, d)));
	*max = std::max(a, std::max(b, std::max(c, d)));
}

void ImageData::CalculatePixelScore
(int u, int v, IplImage *red_features, IplImage *grn_features, IplImage *ylw_features,
 float *r_score, float *g_score, float *y_score)
{
	*r_score = 1.0 - (((float *)(red_features->imageData + v*red_features->widthStep))[u]);
	*g_score = 1.0 - (((float *)(grn_features->imageData + v*grn_features->widthStep))[u]);
	*y_score = 1.0 - (((float *)(ylw_features->imageData + v*ylw_features->widthStep))[u]);
}

void ImageData::CalculateCellScore
(int iymin, int iymax, IplImage *red_features, IplImage *grn_features, IplImage *ylw_features,
 std::vector<Point> *ps, float *r_score, float *g_score, float *y_score)
{

	int num_scanlines = iymax-iymin + 1;
	std::vector<double> xmins(num_scanlines,10000);
	std::vector<double> xmaxs(num_scanlines,-10000);

	// find bounds of polygons and scanline min and max points
	for ( int lc = 0; lc < 4; ++lc )
	{
		Point from = ps->at( lc );
		Point to = ps->at( (lc+1) % 4 );

		double fx = from.X;
		double fy = from.Y;
		double tx = to.X;
		double ty = to.Y;

		if ( fy == ty ) // Horizontal line.
		{
			double y = floor( fy );
			int iy = (int)y;
			int offs = iy - iymin;

			if ( fx < xmins[offs] ) xmins[offs] = fx;
			if ( fx > xmaxs[offs] ) xmaxs[offs] = fx;
			if ( tx < xmins[offs] ) xmins[offs] = tx;
			if ( tx > xmaxs[offs] ) xmaxs[offs] = tx;
		}
		else   // Not horizontal.  Make from Point lower.
		{
			if ( fy > ty )
			{
				double tmp = fy;
				fy = ty;
				ty = tmp;
				tmp = fx;
				fx = tx;
				tx = tmp;
			}

			double y = floor( fy );
			int iy = (int)y;

			double ym = floor( ty );
			double iym = (int)ym;

			double m = (tx-fx)/(ty-fy);
			double x = fx + (y - fy) * m;

			while ( iy <= iym )
			{
				int offs = iy - iymin;
				double nx = x + m;
				double ny = y + 1;

				if ( y <= fy )
				{
					if (fx < xmins[offs]) xmins[offs] = fx;
					if (fx > xmaxs[offs]) xmaxs[offs] = fx;
				}
				else
				{
					if ( x < xmins[offs] ) xmins[offs] = x;
					if ( x > xmaxs[offs] ) xmaxs[offs] = x;
				}
				if ( ny > ty )
				{
					if ( tx < xmins[offs] ) xmins[offs] = tx;
					if ( tx > xmaxs[offs] ) xmaxs[offs] = tx;
				}
				else
				{
					if ( nx < xmins[offs] ) xmins[offs] = nx;
					if ( nx > xmaxs[offs] ) xmaxs[offs] = nx;
				}

				iy++;
				y = ny;
				x = nx;
			}
		}
	}

	//go through each scanline from min bound to max bound to operate row by row on pixels
	float min_r_features = max_ssd_val_;
	float min_y_features = max_ssd_val_;
	float min_g_features = max_ssd_val_;
	float avg_r_features = 0.;
	float avg_y_features = 0.;
	float avg_g_features = 0.;

	float r_features, g_features, y_features;

	int n = 0;

	for (int idx = 0; idx < (num_scanlines); idx++)
	{
		int y = idx + iymin;
		int startx = (int)floor( xmins[idx] );
		int endx = (int)floor( xmaxs[idx] );

		while ( startx < endx )
		{
			r_features = ((float *)(red_features->imageData + y*red_features->widthStep))[startx];
			y_features = ((float *)(ylw_features->imageData + y*ylw_features->widthStep))[startx];
			g_features = ((float *)(grn_features->imageData + y*grn_features->widthStep))[startx];

			avg_r_features += r_features;
			avg_y_features += y_features;
			avg_g_features += g_features;

			if (r_features < min_r_features ) min_r_features = r_features;
			if (y_features < min_y_features ) min_y_features = y_features;
			if (g_features < min_g_features ) min_g_features = g_features;
			startx++;
			n ++;
		}
	}

	avg_r_features /= n;
	avg_y_features /= n;
	avg_g_features /= n;

	// return scores
	//printf("minr: %f, avgr: %f\n", min_r_features, avg_r_features);
	//printf("ming: %f, avgg: %f\n", min_g_features, avg_g_features);
	//printf("miny: %f, avgy: %f\n", min_y_features, avg_y_features);
        *r_score = 1.0 - (float)min_r_features;
	*g_score = 1.0 - (float)min_g_features;
	*y_score = 1.0 - (float)min_y_features;
}


template<typename type>
void IplAddWeighted(IplImage *a, type alpha, IplImage *b, type beta, type gamma, IplImage *c)
{
	if (a->width != b->width || b->width != c->width ||
		 a->height != b->height || b->height != c->height ||
		 a->depth != b->depth || b->depth != c->depth)
		return;

	for (int v = 0; v < a->height; v ++)
	{
		type* a_row = (type*)(a->imageData + v*a->widthStep);
		type* b_row = (type*)(b->imageData + v*b->widthStep);
		type* c_row = (type*)(c->imageData + v*c->widthStep);
		
		for (int u = 0; u < a->width; u ++)
			c_row[u] = a_row[u]*alpha + b_row[u]*beta + gamma;
	}
}

void ImageData::RecalcMinMax(IplImage* a, float& min, float& max)
{
	for (int v = min_v_; v < max_v_; v ++)
	{
		float* a_row = (float*)(a->imageData + v*a->widthStep);

		for (int u = min_u_; u < max_u_; u ++)
		{
			if (a_row[u] < min) min = a_row[u];
			if (a_row[u] > max) max = a_row[u];
		}
	}

	min_ssd_val_ = min;
	max_ssd_val_ = max;
}

bool ImageData::UpdateDataScores
(DetectionGrid *dg, IplImage *curr_image, double dist_to_light, bool downsample, bool save)
{
	if (dg == NULL) printf("what happened!\n");

	//set image width and height
	im_width_  = dg->getImageWidth();
	im_height_ = dg->getImageHeight();

	//set ROI with respect to camera image
	min_u_ = dg->getMinU();
	max_u_ = dg->getMaxU();
	min_v_ = dg->getMinV();
	max_v_ = dg->getMaxV();

	if (min_u_ < 0 || min_u_ > max_u_ || max_u_ > curr_image->width) return false;
	if (min_v_ < 0 || min_v_ > max_v_ || max_v_ > curr_image->width) return false;

	CvRect roi = { min_u_, min_v_, max_u_-min_u_, max_v_-min_v_ };

	// Convert img to the HSV color space
	IplImage* hsv  = cvCreateImage(cvGetSize(curr_image), 8, 3);
	IplImage* sat  = cvCreateImage(cvGetSize(curr_image), 8, 1);
	IplImage* val  = cvCreateImage(cvGetSize(curr_image), 8, 1);
	IplImage* mask = cvCreateImage(cvGetSize(curr_image), 8, 1);
	
	cvSetImageROI(curr_image, roi);
	cvSetImageROI(hsv,  roi);
	cvSetImageROI(sat,  roi);
	cvSetImageROI(val,  roi);
	cvSetImageROI(mask, roi);

	cvCvtColor(curr_image, hsv, CV_RGB2HSV);
	cvResetImageROI(curr_image);
	cvSplit(hsv, NULL, sat, val, NULL);
	cvCmpS(sat, 225, sat, CV_CMP_GE);
	cvCmpS(val,  64, val, CV_CMP_GE);
	cvAnd(sat, val, mask);
	cvReleaseImage(&sat);
	cvReleaseImage(&val);

	//re-initialize min and max ssd to starting point values (to be changed by the subsequent functions)
	max_ssd_val_ = std::numeric_limits<float>::min();
	min_ssd_val_ = std::numeric_limits<float>::max();

	IplImage *tpl = GetLightTemplate(dist_to_light, downsample);

	IplImage *red_features_hue = ConvolveHue(tpl, hsv, 1, 6, &stRedHue_, &stSaturation_, &stFrameVal_, 1);
	IplImage *grn_features_hue = ConvolveHue(tpl, hsv, 1, 7, &stGrnHue_, &stSaturation_, &stFrameVal_, 2);
	IplImage *ylw_features_hue = ConvolveHue(tpl, hsv, 1, 5, &stYlwHue_, &stSaturation_, &stFrameVal_, 3);

	cvReleaseImage(&tpl);

	IplImage* red_features0 = cvCreateImage(cvGetSize(curr_image), IPL_DEPTH_32F, 1);
	IplImage* grn_features0 = cvCreateImage(cvGetSize(curr_image), IPL_DEPTH_32F, 1);
	IplImage* ylw_features0 = cvCreateImage(cvGetSize(curr_image), IPL_DEPTH_32F, 1);

	cvSetImageROI(red_features0, roi);
	cvSetImageROI(grn_features0, roi);
	cvSetImageROI(ylw_features0, roi);

	cvSetImageROI(red_features_hue, roi);
	cvSetImageROI(grn_features_hue, roi);
	cvSetImageROI(ylw_features_hue, roi);

	cvCopy(red_features_hue, red_features0, mask);
	cvCopy(grn_features_hue, grn_features0, mask);
	cvCopy(ylw_features_hue, ylw_features0, mask);

	cvReleaseImage(&mask);

	IplImage* red_features = ConvolveCircleTemplate(red_features0, dist_to_light, downsample); 
	IplImage* grn_features = ConvolveCircleTemplate(grn_features0, dist_to_light, downsample); 
	IplImage* ylw_features = ConvolveCircleTemplate(ylw_features0, dist_to_light, downsample); 

	cvReleaseImage(&red_features0);
	cvReleaseImage(&grn_features0);
	cvReleaseImage(&ylw_features0);

	//IplImage *red_features = (IplImage*)cvClone(red_features_hue);
	//IplImage *grn_features = (IplImage*)cvClone(grn_features_hue);
	//IplImage *ylw_features = (IplImage*)cvClone(ylw_features_hue);


	float a = 1.0/(float)(red_features->height);



	cvSetImageROI(red_features, roi);
	cvSetImageROI(grn_features, roi);
	cvSetImageROI(ylw_features, roi);

	NormalizeFeatures(red_features, grn_features, ylw_features);

	cvConvertScale(red_features, red_features, -1., 1);
	cvConvertScale(grn_features, grn_features, -1., 1);
	cvConvertScale(ylw_features, ylw_features, -1., 1);
	
	for (int v = 0; v < red_features->height; v++)
	{
		float* red_ftrs = (float*)(red_features->imageData + v*red_features->widthStep);
		float* ylw_ftrs = (float*)(ylw_features->imageData + v*ylw_features->widthStep);
		float* grn_ftrs = (float*)(grn_features->imageData + v*grn_features->widthStep);

		for (int u = 0; u < red_features->width; u++)
		{
			if ((u < min_u_ + 10 || u > (max_u_ - 10) || v < min_v_ +10 || v > (max_v_ -10))) {
				red_ftrs[u] = 1;
				ylw_ftrs[u] = 1;
				grn_ftrs[u] = 1;			
			}			
			//red_ftrs[u] *= a*(red_features->height - v + 1);
			//ylw_ftrs[u] *= a*(red_features->height - v + 1);
			//grn_ftrs[u] *= a*(red_features->height - v + 1);
		}
	}
	
	min_ssd_val_ = 0.;
	max_ssd_val_ = 1.;

	static int64_t instance = 0;
	instance ++;

	if (save)
	{
		char instance_char[80];
		sprintf(instance_char, "%ld", instance);
		std::string instance_id(instance_char);

		std::string hsv_id = instance_id + ".hsv.png";
		cvSaveImage(hsv_id.c_str(), hsv);

		int piece_width  = max_u_ - min_u_ + 1;
		int piece_height = max_v_ - min_v_ + 1;
		IplImage *cam_patch = cvCreateImage(cvSize(piece_width, piece_height), IPL_DEPTH_8U, 3);
		
		for (int y = 0; y < piece_height; y++)
		{
			int index_v = min_v_ + y;
			uchar* img_row = ((uchar *)(curr_image->imageData + index_v*curr_image->widthStep));
			uchar* patch_row = ((uchar *)(cam_patch->imageData + y*cam_patch->widthStep));		
			
			for (int x = 0; x < piece_width; x++)
			{
				int index_u = min_u_ + x;
				patch_row[x*cam_patch->nChannels + 0] = img_row[index_u*curr_image->nChannels + 0]; 
				patch_row[x*cam_patch->nChannels + 1] = img_row[index_u*curr_image->nChannels + 1]; 
				patch_row[x*cam_patch->nChannels + 2] = img_row[index_u*curr_image->nChannels + 2]; 
			}
		}

		std::string patch_id = instance_id + ".patch.png";
		cvSaveImage(patch_id.c_str(), cam_patch);
		cvReleaseImage(&cam_patch);

		std::string frame_id = instance_id + ".png";
		cvSaveImage(frame_id.c_str(), curr_image);

		std::string grid_id = instance_id + ".grid";
		std::ofstream f(grid_id.c_str(), std::ios::binary);
		f.write(reinterpret_cast<char*>(dg), sizeof(DetectionGrid));
		f.close();

		std::string param_id = instance_id + ".param";
		f.open(param_id.c_str(), std::ios::binary);
		f.write(reinterpret_cast<char*>(&dist_to_light), sizeof(double));
		f.write(reinterpret_cast<char*>(&downsample), sizeof(bool));
		f.close();
	}

	if (debug_mode_)
		WriteToFeaturesDisplayBuffer(red_features, grn_features, ylw_features);

	cvReleaseImage(&red_features_hue);
	cvReleaseImage(&grn_features_hue);
	cvReleaseImage(&ylw_features_hue);
	cvReleaseImage(&hsv);

	int grid_width = dg->getWidth();
	int grid_height = dg->getHeight();
	for (int y = 0; y < grid_height; y++)
	{
		for (int x = 0; x < grid_width; x++)
		{
			GridCell *curr_cell = dg->getCell(x,y);
			if (!curr_cell->in_frame) //skip cells that are out of the camera frame
			{
				continue;
			}

			int u1 = curr_cell->u1;
			int u2 = curr_cell->u2;
			int u3 = curr_cell->u3;
			int u4 = curr_cell->u4;
			int v1 = curr_cell->v1;
			int v2 = curr_cell->v2;
			int v3 = curr_cell->v3;
			int v4 = curr_cell->v4;

			float r_score, g_score, y_score;
			r_score = g_score = y_score = 0;

			if ((u1 == u2) && (u1 == u3) && (u1 == u4) && (v1 == v2) && (v1 == v3)
			        && (v1 == v4)) //handle trivial case where cell is one pixel or less in size
			{
				CalculatePixelScore(u1, v1, red_features, grn_features, ylw_features, 
					&r_score, &g_score, &y_score);
			}
			else //otherwise, operate on pixels that fall within the cell
			{
				int minV, maxV;
				FindMinMax(v1, v2, v3, v4, &minV, &maxV);
				//printf("minV %d maxV %d\n", minV, maxV);
				//printf("p1x %d p1y %d p2x %d p2y %d p3x %d p3y %d p4x %d p4y %d\n", u1, v1, u2, v2, u3, v3, u4, v4);

				//make sure before this that points are in some kind of clockwise winding order...
				Point p1 = {u1, v1};
				Point p2 = {u2, v2};
				Point p3 = {u3, v3};
				Point p4 = {u4, v4};
				std::vector<Point> points;
				points.push_back(p1);
				points.push_back(p2);
				points.push_back(p3);
				points.push_back(p4);
				CalculateCellScore(minV, maxV, red_features, grn_features, ylw_features, 
					&points, &r_score, &g_score, &y_score);
			}

			//set new scores
			curr_cell->r_score = r_score;
			curr_cell->g_score = g_score;
			curr_cell->y_score = y_score;

		}//end width loop
	}//end height loop

	//release images
	cvReleaseImage(&red_features);
	cvReleaseImage(&grn_features);
	cvReleaseImage(&ylw_features);

	return true;
}


//methods for debug viewer:
void ImageData::getFeaturePatchDimensions(int *ssd_patch_width, int *ssd_patch_height)
{
	*ssd_patch_width = dbg_width_;
	*ssd_patch_height = dbg_height_;
}

void ImageData::WriteToFeaturesDisplayBuffer(IplImage* features_r, IplImage* features_g, IplImage* features_y)
{
	int piece_width  = max_u_ - min_u_ + 1;
	int piece_height = max_v_ - min_v_ + 1;
	dbg_width_ = piece_width*3;
	dbg_height_ = piece_height;

	int index = 0;
	for (int y = 0; y < dbg_height_; y++)
	{
		int index_v = min_v_ + y;
		float* _r = (float*)(features_r->imageData + index_v*features_r->widthStep);
		float* _g = (float*)(features_g->imageData + index_v*features_g->widthStep);
		float* _y = (float*)(features_y->imageData + index_v*features_y->widthStep);

		int x = 0;
		for (; x < piece_width;   x++) feature_patches_[index++] = _r[min_u_ + x];
		for (; x < 2*piece_width; x++) feature_patches_[index++] = _g[min_u_ + (x-piece_width)];
		for (; x < dbg_width_;    x++) feature_patches_[index++] = _y[min_u_ + (x-2*piece_width)];
	}
}

int ImageData::getLastTplWidth()
{
	return last_tpl_width_;
}
int ImageData::getLastTplHeight()
{
	return last_tpl_height_;
}

