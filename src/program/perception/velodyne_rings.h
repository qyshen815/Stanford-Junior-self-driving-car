/*
 * VelodyneRings.h
 *
 *  Created on: Apr 20, 2010
 *      Author: duhadway
 */

#ifndef VELODYNERINGS_H_
#define VELODYNERINGS_H_

#include "perception_defines.h"
#include "perception_types.h"

namespace dgc {

/*
 * Beam and index have specific meanings:
 *   beam: index into beams sorted by vertical angle of beams, 0 maps to nearest beam
 *   index: index into beams as sorted by incoming data
 */

typedef struct {
  int     idx;      /* velodyne index beam */
  int     pb;       /* partner beam for the comparison */
  float   v_angle;  /* vertical angle of the laser beam */
  float   h_angle;  /* horizontal angle of the laser beam */
  int     h_offset; /* horizontal offset of the beam in velodyne ticks */
  float   fac;      /* approximation factor of function with pb */
} velodyne_ring_settings_t;


typedef struct {
  float  x;
  float  y;
} sample_t;

class VelodyneRings {
private:
  velodyne_ring_settings_t    ring[NUM_LASER_BEAMS];
  int                         ridx[NUM_LASER_BEAMS];

  int     min_h_offset;

  double  velodyne_min_beam_diff_;

public:
  VelodyneRings(dgc_velodyne_config_p config, double velodyne_min_beam_diff);
  virtual ~VelodyneRings();

  int beamToIndex(int beam); // go from index to sorted index
  int indexToBeam(int index);  // go from sorted index to index

//  int beam(int index);
//  int index(int idx);
  int nextBeam(int index, int step=1); // returns the index of the next farther ring
  int prevBeam(int index, int step=1); // returns the index of the next closer beam

  int partnerBeam(int index);
  int partnerIndex(int index);

  int minHorizontalOffset();

  int horizontalOffset(int index);
  float factor(int beam);
};

}

#endif /* VELODYNERINGS_H_ */
