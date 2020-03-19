/********************************************************
  This source code is part of the Carnegie Mellon Robot
  Navigation Toolkit (CARMEN). 

  CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
  Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
  and Jared Glover
  All rights reserved.

  Redistribution and use in source and binary forms, with 
  or without modification, are permitted provided that the 
  following conditions are met:

* Redistributions of source code must retain the above 
  copyright notice, this list of conditions and the 
  following disclaimer.
* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the 
  following disclaimer in the documentation and/or other
  materials provided with the distribution.
* The names of the contributors may not be used to endorse
  or promote products derived from this software
  without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGE.
 ********************************************************/

#ifndef DGC_LASER_MESSAGES_H
#define DGC_LASER_MESSAGES_H

#include <ipc_interface.h>

namespace dgc {

  /** SICK laser scan message */

typedef struct {
  int laser_id;               /**< unique ID of laser */
  float fov;                  /**< field of view - in radians */
  int num_range;              /**< number of range values */
  float *range;               /**< range values - in meters */
  int num_intensity;          /**< number of intensity values - may be zero */
  unsigned char *intensity;   /**< intensity values */
  double timestamp;           /**< timestamp */
  char host[10];              /**< hostname associated with timestamp */
} LaserLaser;

#define  DGC_LASER_LASER1_NAME   "dgc_laser_laser1"
#define  DGC_LASER_LASER1_FMT    "{int,float,int,<float:3>,int,<char:5>,double,[char:10]}"

const IpcMessageID LaserLaser1ID = { DGC_LASER_LASER1_NAME, 
				     DGC_LASER_LASER1_FMT };

#define  DGC_LASER_LASER2_NAME   "dgc_laser_laser2"
#define  DGC_LASER_LASER2_FMT    "{int,float,int,<float:3>,int,<char:5>,double,[char:10]}"

const IpcMessageID LaserLaser2ID = { DGC_LASER_LASER2_NAME, 
				     DGC_LASER_LASER2_FMT };

#define  DGC_LASER_LASER3_NAME   "dgc_laser_laser3"
#define  DGC_LASER_LASER3_FMT    "{int,float,int,<float:3>,int,<char:5>,double,[char:10]}"

const IpcMessageID LaserLaser3ID = { DGC_LASER_LASER3_NAME, 
				     DGC_LASER_LASER3_FMT };

#define  DGC_LASER_LASER4_NAME   "dgc_laser_laser4"
#define  DGC_LASER_LASER4_FMT    "{int,float,int,<float:3>,int,<char:5>,double,[char:10]}"

const IpcMessageID LaserLaser4ID = { DGC_LASER_LASER4_NAME, 
				     DGC_LASER_LASER4_FMT };

#define  DGC_LASER_LASER5_NAME   "dgc_laser_laser5"
#define  DGC_LASER_LASER5_FMT    "{int,float,int,<float:3>,int,<char:5>,double,[char:10]}"

const IpcMessageID LaserLaser5ID = { DGC_LASER_LASER5_NAME, 
				     DGC_LASER_LASER5_FMT };

#define  DGC_LASER_LASER6_NAME   "dgc_laser_laser6"
#define  DGC_LASER_LASER6_FMT    "{int,float,int,<float:3>,int,<char:5>,double,[char:10]}"

const IpcMessageID LaserLaser6ID = { DGC_LASER_LASER6_NAME, 
				     DGC_LASER_LASER6_FMT };

#define  DGC_LASER_LASER7_NAME   "dgc_laser_laser7"
#define  DGC_LASER_LASER7_FMT    "{int,float,int,<float:3>,int,<char:5>,double,[char:10]}"

const IpcMessageID LaserLaser7ID = { DGC_LASER_LASER7_NAME, 
				     DGC_LASER_LASER7_FMT };

#define  DGC_LASER_LASER8_NAME   "dgc_laser_laser8"
#define  DGC_LASER_LASER8_FMT    "{int,float,int,<float:3>,int,<char:5>,double,[char:10]}"

const IpcMessageID LaserLaser8ID = { DGC_LASER_LASER8_NAME, 
				     DGC_LASER_LASER8_FMT };

}

#endif

