#ifndef DGC_LOCALIZE_MESSAGES_H
#define DGC_LOCALIZE_MESSAGES_H

#include <ipc_interface.h>

namespace dgc {

#define DGC_LOCALIZE_SOURCE_LASER     1
#define DGC_LOCALIZE_SOURCE_RNDF      2

typedef struct {
  int source;
  double corrected_x, corrected_y;
  char utmzone[5];
  double x_offset, y_offset;
  float std_x, std_y, std_f, std_s;
  double timestamp;
  char host[10];
} LocalizePose;

#define      DGC_LOCALIZE_POSE_NAME     "dgc_localize_pose"
#define      DGC_LOCALIZE_POSE_FMT      "{int,double,double,[char:5],double,double,float,float,float,float,double,[char:10]}"

const IpcMessageID LocalizePoseID = { DGC_LOCALIZE_POSE_NAME, 
				      DGC_LOCALIZE_POSE_FMT };

typedef struct {
  float x, y;
} LocalizeParticle;

typedef struct {
  double mean_x, mean_y;
  char utmzone[5];
  int num_particles;
  LocalizeParticle *particle;
  double timestamp;
  char host[10];
} LocalizeParticles;

#define      DGC_LOCALIZE_PARTICLE_NAME   "dgc_localize_particle"
#define      DGC_LOCALIZE_PARTICLE_FMT    "{double,double,[char:5],int,<{double,double}:4>,double,[char:10]}"

const IpcMessageID LocalizeParticlesID = { DGC_LOCALIZE_PARTICLE_NAME, 
					   DGC_LOCALIZE_PARTICLE_FMT };

}

#endif
