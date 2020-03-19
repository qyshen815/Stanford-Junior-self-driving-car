#ifndef DGC_PASSAT_CONSTANTS_H
#define DGC_PASSAT_CONSTANTS_H

#ifdef __cplusplus
extern "C" {
#endif

  /* all units in meters */

#define       DGC_PASSAT_LENGTH            4.78
#define       DGC_PASSAT_WIDTH             2.10
#define       DGC_PASSAT_HEIGHT            1.51        // car only
#define       DGC_PASSAT_WHEEL_BASE        2.71
#define       DGC_PASSAT_STEERING_RATIO    14.3        // FROM WEB
#define       DGC_PASSAT_IMU_TO_FA_DIST    3.15        // DON'T KNOW YET
#define       DGC_PASSAT_IMU_TO_CG_DIST    1.37
#define       DGC_PASSAT_IMU_TO_R_BUMPER   1.65    // DISTANCE TO REAR BUMPER

#define       DGC_PASSAT_TRACK_WIDTH       1.65    // DON'T KNOW
#define       DGC_PASSAT_WHEEL_RADIUS      0.40
#define       DGC_PASSAT_FA_TO_BUMPER_DIST 1.02    // DON'T KNOW

#define       DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT    1.7

#ifdef __cplusplus
}
#endif

#endif
