#ifndef DGC_PERCEPTION_DEFINES_H
#define DGC_PERCEPTION_DEFINES_H

#define    MAX_NUM_IPC_OBSTACLES       40000
#define    MAX_NUM_POINTS              2000000
#define    MAX_NUM_TRACKER_OBSTACLES   5000
#define    NUM_LINE_POINTS             10
#define    NUM_SAMPLE_POINTS           (3*NUM_LINE_POINTS)

#define    NUM_LASER_BEAMS       64
#define    NUM_VIRTUAL_SCANS     15
#define    NUM_LDLRS_LASERS      2
#define    LDLRS_GROUND_HEIGHT   0.6
#define    NUM_LRR2_RADARS       4
#define    NUM_LRR3_RADARS       2
#define    NUM_RADARS            (NUM_LRR2_RADARS + NUM_LRR3_RADARS)
#define    RADAR3_INDEX          2   // RADAR3 is an LRR3. This value is used in the radar for-loops.
#define    RADAR6_INDEX          5   // RADAR6 is an LRR3. This value is used in the radar for-loops.

#define    PERCEPTION_RAY_TRACING_MIN_DIST    3.0
#define    PERCEPTION_RAY_TRACING_MAX         40.0
#define    PERCEPTION_RAY_TRACING_FREESPACE   20.0
#define    PERCEPTION_RAY_TRACING_INNER       15.0
#define    PERCEPTION_RAY_TRACING_SHORTEN     0.35
#define    PERCEPTION_RAY_TRACING_RES         1.00

#define    PERCEPTION_MAP_LOW_HEIGHT          0.4
#define    PERCEPTION_MAP_HIGH_HEIGHT         2.2

#define    CM_TO_METER_FACTOR       0.01

#endif
