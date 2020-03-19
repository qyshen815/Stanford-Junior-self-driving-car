#include <roadrunner.h>
#include "intensitycal.h"

int IntensityCalibration::Load(char *filename) 
{
  FILE *fp = fopen(filename, "r");
  int i, j;

  if(fp == NULL)
    return -1;

  if(fscanf(fp, "%d %d\n", &min_intensity_, &max_intensity_) != 2) {
    dgc_error("Trouble reading from calibration file %s.", filename);
    fclose(fp);
    return -1;
  }
  for(i = 0; i < 64; i++)
    for(j = 0; j < 256; j++)
      if(fscanf(fp, "%lf ", &map_[i][j]) != 1) {
        dgc_error("Trouble reading from calibration file %s.", filename);
        fclose(fp);
        return -1;
      }
  fclose(fp);
  return 0;
}

