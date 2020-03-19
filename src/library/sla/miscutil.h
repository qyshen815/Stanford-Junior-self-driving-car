//-*-c++-*-
#ifndef MISCUTIL_H
#define MISCUTIL_H

namespace sla {
  /// Returns a time vector for a count of seconds
  inline Vec3<int> secondsToClock(double time) {
    Vec3<int> clock;
    clock(0) = int(time/3600.0);
    clock(1) = int((time - 3600.0*double(clock(0)))/60.0);
    clock(2) = int(time - 3600.0*double(clock(0)) - 
      60.0*double(clock(1)));
    return clock;
  }

  /// Prints a clock value to passed stream
  inline void printClock(Vec3<int> clock, std::ostream& os) {
    os << clock(0) << ":" << clock(1) << ":" << clock(2);
  }

  // Define various constants used for colorspace conversion
  const SMat3f rgb2xyz_matrix(0.412453f, 0.357580f, 0.180423f, 
                        0.212671f, 0.715160f, 0.072169f, 
                        0.019334f, 0.119193f, 0.950227f); 
  const SMat3f xyz2rgb_matrix(3.240479f, -1.537150f, -0.498535f, 
                        -0.969256f,  1.875992f,  0.041556f, 
                        0.055648f, -0.204043f, 1.057311f); 
  const Vec3f xyz_n(242.366287231f, 255.0f, 277.632263184f);
  const Vec3f lab_offset(0.0f, 87.0f, 108.0f);
  const float lab_scale = 255.0f/205.0f;

  /// Used in colorspace conversion from XYZ to Lab
  inline float labF(float t) {
    if ( t > 0.008856 ) { 
      return pow(t, 1.0f/3.0f); 
    } else {
      return 7.787f * t + 16.0f/116.0f;
    }
  }

  /// Converts from XYZ to Lab for floating point math, uses xyz_n as reference white
  inline Vec3f xyzToLab(Vec3f xyz) {
    xyz /= xyz_n; 
    Vec3f lab;
    if (xyz(1) > 0.008856) 
      lab(0) = 116.0f * pow(xyz(1),1.0f/3.0f) - 16.0f; 
    else
      lab(0) = 903.3f * xyz(1); 

    lab(1) = 500 * (labF(xyz(0)) - labF(xyz(1))); 
    lab(2) = 200 * (labF(xyz(1)) - labF(xyz(2))); 

    return lab;
  }

  /// Converts from Lab to XYZ for floating point math, uses xyz_n as reference white
  inline Vec3f labToXyz(Vec3f lab) {
    float p = (lab(0) + 16.0f)/116.0f;
    Vec3f xyz(xyz_n(0)*cube<float>(p + lab(1)/500.0f),
              xyz_n(1)*cube<float>(p),
              xyz_n(2)*cube<float>(p - lab(2)/200.0f));
    return xyz;
  }

  /// Converts from RGB [0-255] to Lab through XYZ using floating point math
  inline Vec3f rgbToLab(Vec3f rgb) {
    Vec3f xyz = rgb2xyz_matrix * rgb; 
    return xyzToLab(xyz);
  }

  /// Converts from Lab to RGB [0-255] throuhg XYZ using floating point math
  inline Vec3f labToRgb(Vec3f lab) {
    Vec3f xyz = labToXyz(lab);
    return xyz2rgb_matrix * xyz;
  }

  /// Converts from RGB [0-255] to Lcd through XYZ, Lab using floating point math
  inline Vec3f rgbToLcd(Vec3f rgb) {
    return lab_scale*(rgbToLab(rgb) + lab_offset);
  }

  /// Converts from Lcd to RGB [0-255] through XYZ, Lab using floating point math
  inline Vec3f lcdToRgb(Vec3f lcd) {
    return labToRgb((1.0f/lab_scale)*lcd - lab_offset);
  }

}

#endif
