#ifndef DGC_RIEGL_MESSAGES_H
#define DGC_RIEGL_MESSAGES_H

#include <ipc_interface.h>

namespace dgc {

typedef struct {
  float start_angle;          /**< angle of first beam relative to top of laser, in radians */
  float fov;                  /**< field of view - in radians */

  int num_range;              /**< number of range readings */
  float *range;               /**< range readings - in meters */
  
  int num_intensity;          /**< number of intensity readings */
  unsigned char *intensity;   /**< intensity readings - 0 to 255 */
  
  int num_angle;              /**< number of angle readings */
  float *angle;               /**< angle of each beam relative to top of laser, in radians */
   
  int num_quality;            /**< number of quality readings */
  unsigned char *quality;     /**< quality readings, 0 for bad, 100 for OK */
  
  int num_shot_timestamp;     /**< number of timestamp readings */
  float *shot_timestamp;      /**< precise timestamp of each reading relative to last sync pulse, in seconds */

  double line_timestamp;      /**< timestamp of line relative to sync pulse */

  double timestamp;           /**< timestamp */
  char host[10];              /**< hostname associated with timestamp */
} RieglLaser;

#define  DGC_RIEGL_LASER1_NAME   "dgc_riegl_laser1"
#define  DGC_RIEGL_LASER1_FMT    "{float,float,int,<float:3>,int,<char:5>,int,<float:7>,int,<char:9>,int,<float:11>,double,double,[char:10]}"

const IpcMessageID RieglLaser1ID = { DGC_RIEGL_LASER1_NAME, 
				     DGC_RIEGL_LASER1_FMT };

#define  DGC_RIEGL_LASER2_NAME   "dgc_riegl_laser2"
#define  DGC_RIEGL_LASER2_FMT    "{float,float,int,<float:3>,int,<char:5>,int,<float:7>,int,<char:9>,int,<float:11>,double,double,[char:10]}"

const IpcMessageID RieglLaser2ID = { DGC_RIEGL_LASER2_NAME, 
				     DGC_RIEGL_LASER2_FMT };

}

#endif

