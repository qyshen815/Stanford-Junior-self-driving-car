#ifndef DGC_APPLANIX_MESSAGES_H
#define DGC_APPLANIX_MESSAGES_H

#include <ipc_interface.h>

namespace dgc {

  /** Primary applanix pose message */

typedef struct {
  double smooth_x;     /** Velocity-integrated, smooth position (East).  0=where applanix was started */
  double smooth_y;     /** Velocity-integrated, smooth position (North). 0=where applanix was started */
  double smooth_z;     /** Velocity-integrated, smooth position (Up).    0=where applanix was started */
  double latitude;     /**< Latitude, in degrees */
  double longitude;    /**< Longitude, in degrees */
  double altitude;     /**< Altitude, in meters */
  float v_north;       /**< north velocity, in m/s */
  float v_east;        /**< east velocity, in m/s */
  float v_up;          /**< up velocity, in m/s */
  float speed;         /**< Magitude of velocity vector, in m/s */
  float track;         /**< planar heading of velocity vector, in radians */
  double roll;         /**< roll, in radians */
  double pitch;        /**< pitch, in radians */
  double yaw;          /**< yaw, in radians */
  double ar_roll;      /**< roll rate, in rad/sec */
  double ar_pitch;     /**< pitch rate, in rad/sec */
  double ar_yaw;       /**< yaw rate, in rad/sec */
  double a_x;          /**< acceleration in x, in m/s/s (body frame) */
  double a_y;          /**< acceleration in y, in m/s/s (body frame) */
  double a_z;          /**< acceleration in z, in m/s/s (body frame) */
  double wander;       /**< wander angle, in radians */
  unsigned int ID;     /**< unique ID for internal tracking */
  int postprocess_code;  /**< 0 = Real Time.  1 = Post Processed.  2 = Post Processed with Base Station. */
  double hardware_timestamp;    /**< Timestamp from the Applanix hardware, in UTC seconds of the week. */
  int hardware_time_mode;        /**< Mode of timestamp from the Applanix hardware.  0 = None.  1 = Acquire.  2 = Locked. */
  double timestamp;    /**< DGC timestamp */
  char host[10];       /**< hostname associated with timestamp */
} ApplanixPose;

#define        DGC_APPLANIX_POSE_NAME        "dgc_applanix_pose"
#define        DGC_APPLANIX_POSE_FMT         "{double,double,double,double,double,double,float,float,float,float,float,double,double,double,double,double,double,double,double,double,double,int,int,double,int,double,[char:10]}"

const IpcMessageID ApplanixPoseID = { DGC_APPLANIX_POSE_NAME,
				      DGC_APPLANIX_POSE_FMT };

  /** Applanix pose error message */

typedef struct {
  float rms_north;    /**< North position error, in meters (global frame) */
  float rms_east;     /**< East position error, in meters (global frame) */
  float rms_up;       /**< Up position error, in meters (global frame) */
  float rms_v_north;  /**< North velocity error, in m/s (global frame) */
  float rms_v_east;   /**< East velocity error, in m/s (global frame) */ 
  float rms_v_up;     /**< Up velocity error, in m/s (global frame) */
  float rms_roll;     /**< roll error, in radians */
  float rms_pitch;    /**< pitch error, in radians */
  float rms_yaw;      /**< yaw error, in radians */
  float semi_major;   /**< length of semi major axis of error ellipse, in m */
  float semi_minor;   /**< length of semi minor axis of error ellipse, in m */
  float orientation;  /**< orientation of error ellipse, radians */
  unsigned int ID;    /**< unique ID for internal tracking */
  int postprocess_code;   /**< 0 = Real Time.  1 = Post Processed.  2 = Post Processed with Base Station. */
  double hardware_timestamp;  /**< Timestamp from the Applanix hardware, in UTC seconds of the week. */
  int hardware_time_mode;     /**< Mode of timestamp from the Applanix hardware.  0 = None.  1 = Acquire.  2 = Locked. */
  double timestamp;   /**< DGC timestamp */
  char host[10];      /**< hostname associated with timestamp */
} ApplanixRms;

#define        DGC_APPLANIX_RMS_NAME        "dgc_applanix_rms"
#define        DGC_APPLANIX_RMS_FMT         "{float,float,float,float,float,float,float,float,float,float,float,float,int,int,double,int,double,[char:10]}"

const IpcMessageID ApplanixRmsID = { DGC_APPLANIX_RMS_NAME,
				     DGC_APPLANIX_RMS_FMT };

  /** Applanix GPS message */

typedef struct {
  int primary_sats;         /**< primary GPS satellite constellation. */
  unsigned int primary_ID;  /**< primary GPS unique ID, for internal tracking. */
  double primary_timestamp; /**< primary GPS timestamp from machine receiving Applanix message, in seconds */
  int secondary_sats;       /**< secondary GPS satellite constellation. */  
  unsigned int secondary_ID; /**< secondary GPS unique ID, for internal tracking. */
  double secondary_timestamp; /**< secondary GPS timestamp from machine receiving Applanix message, in seconds */
  int gams_solution_code;     /**< GAMS status code.  Range is 0-7.  7 is full solution. */
  unsigned int gams_ID;       /**< GAMS unique ID, for internal tracking. */
  double gams_timestamp;      /**< GAMS timestamp from machine receiving Applanix message, in seconds */
  double timestamp;           /**< DGC timestamp */
  char host[10];              /**< hostname associated with timestamp */
} ApplanixGps;

#define        DGC_APPLANIX_GPS_NAME        "dgc_applanix_gps"
#define        DGC_APPLANIX_GPS_FMT         "{int,int,double,int,int,double,int,int,double,double,[char:10]}"

const IpcMessageID ApplanixGpsID = { DGC_APPLANIX_GPS_NAME,
				     DGC_APPLANIX_GPS_FMT };

  /** Applanix DMI message */

typedef struct {
  double signed_odometer;        /**< DMI distance odometer, in meters, signed value */
  double unsigned_odometer;        /**< DMI distance odometer, in meters, unsigned value */
  unsigned int ID;                /**< unique ID for internal tracking */
  double hardware_timestamp;        /**< Timestamp from the Applanix hardware, in UTC seconds of the week. */
  int hardware_time_mode;        /**< Mode of timestamp from the Applanix hardware.  0 = None.  1 = Acquire.  2 = Locked. */
  double timestamp;                /**< DGC timestamp */
  char host[10];                /**< hostname associated with timestamp */
} ApplanixDmi;

#define                DGC_APPLANIX_DMI_NAME       "dgc_applanix_dmi"
#define                DGC_APPLANIX_DMI_FMT        "{double,double,int,double,int,double,[char:10]}"

const IpcMessageID ApplanixDmiID = { DGC_APPLANIX_DMI_NAME,
				     DGC_APPLANIX_DMI_FMT };

}

#endif
