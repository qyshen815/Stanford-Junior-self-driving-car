#include <stdio.h>
#include "ShapeTables.h"
#include <stdint.h>

int main(int argc, char** argv)
{
	ShapeTable<uint8_t> stRedHue_;
	ShapeTable<uint8_t> stGrnHue_;
	ShapeTable<uint8_t> stYlwHue_;

	ShapeTable<uint8_t> stSatur_;

/*	ShapeTable<int64_t> stRedSat_;
	ShapeTable<int64_t> stGrnSat_;
	ShapeTable<int64_t> stYlwSat_;
*/
	ShapeTable<int64_t> stFrameVal_;
	
	double a = 255.;
	double b = 100.;

	//stRedHue_.SetParams(0/2, 	20/2,		a,	4.3285/2.,	0.36109/2.); // From icra_18_10 samples
	//stGrnHue_.SetParams(100/2,	200/2,	a,	12.023/2.,	162.5  /2.); 
	stRedHue_.Load("red_hue_shape_table.uint8");
	stGrnHue_.Load("grn_hue_shape_table.uint8");
	stYlwHue_.Load("ylw_hue_shape_table.uint8");
	//stYlwHue_.SetParams(0/2, 	130/2,	a,	9.3623/2.,	31.195 /2.);
	
/*	stRedHue_.GenerateGaussianTable();
	//stGrnHue_.GenerateGaussianTable();
	stYlwHue_.GenerateGaussianTable();
*/
/*	stRedHue_.ApplyCompression(b, 4);
	stGrnHue_.ApplyCompression(b, 4); 
	stYlwHue_.ApplyCompression(b, 4);
*/
/*	stRedHue_.InvertValues();
	stGrnHue_.InvertValues();
	stYlwHue_.InvertValues();
*/	

	stSatur_.Load("saturation_shape_table.uint8");
//	stSatur_.ApplyCompression(10, 50);
	//                 min	max	a	b         unused
/*	stRedSat_.SetParams(0,	20,	a, -.434715, 0.);
	stGrnSat_.SetParams(245, 255, a/2, 0.85944, 255.14);
	stYlwSat_.SetParams(0,	30,	a, -.317710, 0.);

	stRedSat_.GenerateExponentialTable();
	stGrnSat_.GenerateGaussianTable();
	stYlwSat_.GenerateExponentialTable();

	stRedSat_.ApplyCompression(b, 14);
	stGrnSat_.ApplyCompression(b, 14);
	stYlwSat_.ApplyCompression(b, 14);

	stRedSat_.InvertValues();
	stGrnSat_.InvertValues();
	stYlwSat_.InvertValues();

	stRedSat_.MoveDomain(255, 235);
//	stGrnSat_.MoveDomain(255, 205);
	stYlwSat_.MoveDomain(255, 225);
*/
	stFrameVal_.SetParams(0,	255,	a,	-.035,	0.);
	stFrameVal_.GenerateExponentialTable();
	stFrameVal_.InvertValues();

	int hue = atoi(argv[1]);
	int sat = atoi(argv[2]);
	int val = atoi(argv[3]);

	printf(" %ld ", stRedHue_[hue]);
	printf(" %ld ", stGrnHue_[hue]);
	printf(" %ld ", stYlwHue_[hue]);
	printf(" %ld \n", stSatur_[sat]);

	for (int i = 0; i < 256; i ++) printf("%ld ", stRedHue_[i]);
	printf("\n");
	for (int i = 0; i < 256; i ++) printf("%ld ", stGrnHue_[i]);
	printf("\n");
	for (int i = 0; i < 256; i ++) printf("%ld ", stYlwHue_[i]);
	printf("\n");

	return 0;
}
