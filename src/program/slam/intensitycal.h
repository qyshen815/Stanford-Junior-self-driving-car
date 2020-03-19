#ifndef DGC_INTENSITYCAL_H
#define DGC_INTENSITYCAL_H

#include <roadrunner.h>

class IntensityCalibration {
public:
  int Load(char *filename);
  inline double Lookup(int ring_num, unsigned char value);

private:
  int min_intensity_, max_intensity_;
  double map_[64][256];
};

inline double IntensityCalibration::Lookup(int ring_num, unsigned char value)
{
  return map_[ring_num][value];
}

#endif
