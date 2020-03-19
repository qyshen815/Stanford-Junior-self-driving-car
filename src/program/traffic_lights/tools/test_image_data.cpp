#include <opencv/cv.h>
#include "traffic_light_transforms.h"
#include "detection_grid.h"
#include "image_data.h"
#include "histogram_filter_tl.h"

using namespace dgc;
using namespace std;
using namespace vlr;

DetectionGrid* LoadGrid(const char* filename)
{
	DetectionGrid* dg = NULL;
	int length = 0;

	std::ifstream in(filename, std::ios::binary);
	if (!in.good() || in.eof() || !in.is_open()) return NULL;

	in.seekg(0, std::ios_base::beg);
	std::ifstream::pos_type begin_pos = in.tellg();
	in.seekg(0, std::ios_base::end);
	length = static_cast<int>(in.tellg() - begin_pos);
	if (length != sizeof(DetectionGrid)) return NULL;

	dg = new DetectionGrid;
	in.seekg(0, std::ios_base::beg);
	in.read(reinterpret_cast<char*>(dg), length);
	in.close();

	return dg;
}

void LoadParams(const char* filename, double* dist, bool* downsample)
{
	int length = 0;

	std::ifstream in(filename, std::ios::binary);
	if (!in.good() || in.eof() || !in.is_open()) return;

	in.seekg(0, std::ios_base::beg);
	std::ifstream::pos_type begin_pos = in.tellg();
	in.seekg(0, std::ios_base::end);
	length = static_cast<int>(in.tellg() - begin_pos);
	if (length != sizeof(double)+sizeof(bool)) return;

	in.seekg(0, std::ios_base::beg);
	in.read(reinterpret_cast<char*>(dist), sizeof(double));
	in.read(reinterpret_cast<char*>(downsample), sizeof(bool));
	in.close();
}

int main(int argc, char** argv)
{
	if (argc < 2)
	{
		printf("Usage: %s [instance id]\n"
				 "Files [instance id].png, [instance id].grid,\n"
				 "and [instance id].param should exist.\n\n", argv[0]);
		return -1;
	}

	int32_t ssd_width, ssd_height;
	double dist;
	bool downsample;
	float SSD_patches[490000];

	std::string frame_filename(argv[1]);
	frame_filename = frame_filename + ".png";
	IplImage* frame = cvLoadImage(frame_filename.c_str(), CV_LOAD_IMAGE_COLOR);
	if (frame == NULL) return -1;

	std::string grid_filename(argv[1]);
	grid_filename = grid_filename + ".grid";
	DetectionGrid* grid = LoadGrid(grid_filename.c_str());
	if (grid == NULL) return -1;

	std::string params_filename(argv[1]);
	params_filename = params_filename + ".param";
	LoadParams(params_filename.c_str(), &dist, &downsample);

	double brightness = 0.;
	if (argc == 3)
		sscanf(argv[2], "%lf", &brightness);

	TrafficLightTransform tt('l');
	grid->trans_ = &tt;

	ImageData imgDat(true, SSD_patches);
	imgDat.UpdateDataScores(grid, frame, dist, downsample, false);
	imgDat.getSSDPatchDimensions(&ssd_width, &ssd_height);
	
	IplImage* ssd = cvCreateImageHeader(cvSize(ssd_width, ssd_height), IPL_DEPTH_32F, 1);
	IplImage* ssd8 = cvCreateImage(cvGetSize(ssd), 8, 1);
	ssd->imageData = reinterpret_cast<char*>(SSD_patches);

	if (brightness != 0.)
	{
		for (int v = 0; v < ssd_height; v ++)
			for (int u = 0; u < ssd_width; u ++)
			{
				float val = ((float*)(ssd->imageData + v*ssd->widthStep))[u];
				((float*)(ssd->imageData + v*ssd->widthStep))[u] = pow(pow(10,brightness),val);
			}

		cvNormalize(ssd, ssd, 255.);
		cvConvertImage(ssd, ssd8);
		cvSaveImage("ssd.png", ssd8);
	}
	else
	{
		int i = 0;
		float* buffer = new float[ssd_height*ssd_width];
		for (int v = 0; v < ssd_height; v ++)
			for (int u = 0; u < ssd_width; u ++)
				buffer[i++] = ((float*)(ssd->imageData + v*ssd->widthStep))[u];

		std::ofstream f("ssd.dat", std::ios::binary);
		f.write(reinterpret_cast<char*>(&ssd_width), sizeof(int32_t));
		f.write(reinterpret_cast<char*>(&ssd_height), sizeof(int32_t));
		f.write(reinterpret_cast<char*>(buffer), sizeof(float)*ssd_height*ssd_width);
		f.close();
	}

	cvReleaseImage(&ssd8);
	cvReleaseImageHeader(&ssd);
	return 0;
}

