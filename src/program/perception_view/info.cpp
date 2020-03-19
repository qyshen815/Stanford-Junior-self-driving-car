#include "view.h"

char gams_solution[][80] = {
    "no solution",
    "solution from nav. & install",
    "solution without install data",
    "degraded floated ambiguity",
    "floated ambiguity",
    "degraded fixed integer",
    "fixed integer test install data",
    "fixed integer",
};

char *
numf(float v, char *unit) {
  static char nstr[20];
  int val = (int) (v * 100);
  if (v >= 0) {
    snprintf(nstr, 20, "%s%02.2f %s", val < 1000 ? "0" : "", val / 100.0, unit);
    return (nstr);
  }
  return ("-");
}

char *
numf2(float v, char *unit) {
  static char nstr[20];
  int val = (int) fabs(v * 100);
  snprintf(nstr, 20, "%c%s%02.2f %s", v < 0 ? '-' : '+', val < 1000 ? "0" : "", val / 100.0, unit);
  return (nstr);
}

char *
numi(int v, char *unit) {
  static char nstr[20];
  if (v >= 0) {
    snprintf(nstr, 20, "%d %s", v, unit);
    return (nstr);
  }
  return ("-");
}

char *
can_gear(char gear_position) {
  static char str[20];
  switch (gear_position) {
    case CAN_GEAR_POSITION_INTERMEDIATE:
      strncpy(str, "-", 20);
      break;
    case CAN_GEAR_POSITION_1:
      strncpy(str, "1", 20);
      break;
    case CAN_GEAR_POSITION_2:
      strncpy(str, "2", 20);
      break;
    case CAN_GEAR_POSITION_3:
      strncpy(str, "3", 20);
      break;
    case CAN_GEAR_POSITION_4:
      strncpy(str, "4", 20);
      break;
    case CAN_GEAR_POSITION_D:
      strncpy(str, "D", 20);
      break;
    case CAN_GEAR_POSITION_N:
      strncpy(str, "N", 20);
      break;
    case CAN_GEAR_POSITION_R:
      strncpy(str, "R", 20);
      break;
    case CAN_GEAR_POSITION_P:
      strncpy(str, "P", 20);
      break;
    case CAN_GEAR_POSITION_RSP:
      strncpy(str, "RSP", 20);
      break;
    case CAN_GEAR_POSITION_Z1:
      strncpy(str, "Z1", 20);
      break;
    case CAN_GEAR_POSITION_Z2:
      strncpy(str, "Z2", 20);
      break;
    case CAN_GEAR_POSITION_S:
      strncpy(str, "S", 20);
      break;
    case CAN_GEAR_POSITION_L:
      strncpy(str, "L", 20);
      break;
    case CAN_GEAR_POSITION_TIPTRONIC:
      strncpy(str, "TIP", 20);
      break;
    case CAN_GEAR_POSITION_ERROR:
      strncpy(str, "ERR", 20);
      break;
  }
  return (str);
}

char *
internal_gear(char gear_position) {
  static char str[20];
  switch (gear_position) {
    case CAN_TARGET_GEAR_PARK_NEUTRAL:
      strncpy(str, "N", 20);
      break;
    case CAN_TARGET_GEAR_1ST:
      strncpy(str, "1.", 20);
      break;
    case CAN_TARGET_GEAR_2ND:
      strncpy(str, "2.", 20);
      break;
    case CAN_TARGET_GEAR_3RD:
      strncpy(str, "3.", 20);
      break;
    case CAN_TARGET_GEAR_4TH:
      strncpy(str, "4.", 20);
      break;
    case CAN_TARGET_GEAR_5TH:
      strncpy(str, "5.", 20);
      break;
    case CAN_TARGET_GEAR_1M:
      strncpy(str, "1M", 20);
      break;
    case CAN_TARGET_GEAR_REVERSE:
      strncpy(str, "Rev", 20);
      break;
    case CAN_TARGET_GEAR_6TH:
      strncpy(str, "6.", 20);
      break;
    case CAN_TARGET_GEAR_7TH:
      strncpy(str, "7.", 20);
      break;
    case CAN_TARGET_GEAR_ERROR:
      strncpy(str, "Err", 20);
      break;
  }
  return (str);
}

void draw_info_window(ApplanixPose *pose) {
  int n = 0;
  snprintf(sensor_info[n++], NAME_LENGTH, "Perception:               %s", numf(perception_hz, "hz"));
  snprintf(sensor_info[n++], NAME_LENGTH, "Obstacles:                %s", numi(obstacles.num_points, "pts"));
  snprintf(sensor_info[n++], NAME_LENGTH, "Occ. Map Cells         %s", numi(occupied_map_cells, "cells"));
  occupied_map_cells = -1;
  snprintf(sensor_info[n++], NAME_LENGTH, " ");
  snprintf(sensor_info[n++], NAME_LENGTH, "CAN:                       %s", numf(can_hz, "hz"));
  snprintf(sensor_info[n++], NAME_LENGTH, "Steering:                   %.2f deg", can.steering_angle);
  snprintf(sensor_info[n++], NAME_LENGTH, "Throttle:                    %s", numf(can.throttle_position, "%"));
  snprintf(sensor_info[n++], NAME_LENGTH, "Brake:                      %s", numf(can.brake_pressure, ""));
  snprintf(sensor_info[n++], NAME_LENGTH, "RPM:                       %d", (int) (can.engine_rpm));
  snprintf(sensor_info[n++], NAME_LENGTH, "Gear:                       %s (%s Gear)", can_gear(can.gear_position),
      internal_gear(can.target_gear));
  snprintf(sensor_info[n++], NAME_LENGTH, "Speed:                     %.2f m/h [%.2f km/h]", (can.wheel_speed_fl
      + can.wheel_speed_fr) / (1.609344 * 2.0), (can.wheel_speed_fl + can.wheel_speed_fr) / 2.0);
  snprintf(sensor_info[n++], NAME_LENGTH, "Speed (Applanix):      %.2f m/h [%.2f km/h]",
      pose->speed * (3.6 / 1.609344), pose->speed * 3.6);
  snprintf(sensor_info[n++], NAME_LENGTH, " ");
  snprintf(sensor_info[n++], NAME_LENGTH, "Applanix:                  %s", numf(applanix_hz, "hz"));
  snprintf(sensor_info[n++], NAME_LENGTH, "Applanix RMS:          %.2f   %.2f   %.2f", applanix_rms.rms_north,
      applanix_rms.rms_east, applanix_rms.rms_up);
  snprintf(sensor_info[n++], NAME_LENGTH, " ");
  snprintf(sensor_info[n++], NAME_LENGTH, "Latitude:                   %.7f deg", pose->latitude);
  snprintf(sensor_info[n++], NAME_LENGTH, "Longitude:                %.7f deg", pose->longitude);
  snprintf(sensor_info[n++], NAME_LENGTH, "Altitude:                   %.4f m", pose->altitude);
  snprintf(sensor_info[n++], NAME_LENGTH, " ");
  snprintf(sensor_info[n++], NAME_LENGTH, "Yaw:                        %.4f deg", dgc_r2d(pose->yaw));
  snprintf(sensor_info[n++], NAME_LENGTH, "Pitch:                       %.4f deg", dgc_r2d(pose->pitch));
  snprintf(sensor_info[n++], NAME_LENGTH, "Roll:                         %.4f deg", dgc_r2d(pose->roll));

  snprintf(sensor_info[n++], NAME_LENGTH, " ");
  snprintf(sensor_info[n++], NAME_LENGTH, "Smooth-X:               %.4f m", pose->smooth_x);
  snprintf(sensor_info[n++], NAME_LENGTH, "Smooth-Y:               %.4f m", pose->smooth_y);
  snprintf(sensor_info[n++], NAME_LENGTH, "Smooth-Z:               %.4f m", pose->smooth_z);
  snprintf(sensor_info[n++], NAME_LENGTH, " ");
  snprintf(sensor_info[n++], NAME_LENGTH, "Correction-X:           %.4f m", gps_offset_x);
  snprintf(sensor_info[n++], NAME_LENGTH, "Correction-Y:           %.4f m", gps_offset_y);
  snprintf(sensor_info[n++], NAME_LENGTH, "Correction-D:           %.4f m", hypot(gps_offset_x, gps_offset_y));
  snprintf(sensor_info[n++], NAME_LENGTH, " ");
  snprintf(sensor_info[n++], NAME_LENGTH, "Ldlrs 1:                    %s", numf(ldlrs1_hz, "hz"));
  snprintf(sensor_info[n++], NAME_LENGTH, "Ldlrs 2:                    %s", numf(ldlrs2_hz, "hz"));
  snprintf(sensor_info[n++], NAME_LENGTH, " ");
  snprintf(sensor_info[n++], NAME_LENGTH, "LMS 1:                    %s", numf(lms1_hz, "hz"));
  snprintf(sensor_info[n++], NAME_LENGTH, "LMS 3:                    %s", numf(lms3_hz, "hz"));
  snprintf(sensor_info[n++], NAME_LENGTH, " ");
  snprintf(sensor_info[n++], NAME_LENGTH, "Acceleration-X:      %.3f G", fabs(pose->a_x / 9.81));
  snprintf(sensor_info[n++], NAME_LENGTH, "Acceleration-Y:      %.3f G", fabs(pose->a_y / 9.81));
  snprintf(sensor_info[n++], NAME_LENGTH, "Acceleration-Z:      %.3f G", fabs(pose->a_z / 9.81));

  snprintf(sensor_info[n++], NAME_LENGTH, " ");
  snprintf(sensor_info[n++], NAME_LENGTH, "GLS:                       %s", numf(gls_hz, "hz"));
  snprintf(sensor_info[n++], NAME_LENGTH, " ");
  snprintf(sensor_info[n++], NAME_LENGTH, "Date:                      %s", createDateString(pose->timestamp));
  snprintf(sensor_info[n++], NAME_LENGTH, "Time:                      %s", createTimeString(pose->timestamp));

  vlr::gui3D_help_window(n, sensor_info, GLUT_BITMAP_HELVETICA_12, show_sensor_info_layout);
}

void draw_applanix_window(ApplanixPose *pose) {
  int n = 0;
  char speed_mi_h[NAME_LENGTH];
  char speed_km_h[NAME_LENGTH];

  snprintf(sensor_info[n++], NAME_LENGTH, "Applanix:                  %s", numf(applanix_hz, "hz"));
  snprintf(sensor_info[n++], NAME_LENGTH, " ");
  snprintf(sensor_info[n++], NAME_LENGTH, "RMS Pos [m]:              %.5f   %.5f   %.5f", applanix_rms.rms_north,
      applanix_rms.rms_east, applanix_rms.rms_up);
  snprintf(sensor_info[n++], NAME_LENGTH, "RMS Angle [deg]:        %.5f   %.5f   %.5f", dgc_r2d(applanix_rms.rms_roll),
      dgc_r2d(applanix_rms.rms_pitch), dgc_r2d(applanix_rms.rms_yaw));
  snprintf(sensor_info[n++], NAME_LENGTH, "RMS Velocity [m/s]:     %.5f   %.5f   %.5f", applanix_rms.rms_v_north,
      applanix_rms.rms_v_east, applanix_rms.rms_v_up);

  snprintf(sensor_info[n++], NAME_LENGTH, " ");
  snprintf(sensor_info[n++], NAME_LENGTH, "Error ellipse:              %s (semi major axis)", numf2(
      applanix_rms.semi_major, "m"));
  snprintf(sensor_info[n++], NAME_LENGTH, "Error ellipse:              %s (semi minor axis)", numf2(
      applanix_rms.semi_minor, "m"));
  snprintf(sensor_info[n++], NAME_LENGTH, "Error ellipse:              %.2f deg (orientation)", dgc_r2d(
      applanix_rms.orientation));

  snprintf(sensor_info[n++], NAME_LENGTH, " ");
  strncpy(speed_mi_h, numf2(pose->speed * (3.6 / 1.609344), "m/h"), NAME_LENGTH);
  strncpy(speed_km_h, numf2(pose->speed * 3.6, "km/h"), NAME_LENGTH);
  snprintf(sensor_info[n++], NAME_LENGTH, "Speed:                     %s [%s]", speed_mi_h, speed_km_h);

  snprintf(sensor_info[n++], NAME_LENGTH, " ");
  strncpy(speed_mi_h, numf2(pose->v_north * (3.6 / 1.609344), "m/h"), NAME_LENGTH);
  strncpy(speed_km_h, numf2(pose->v_north * 3.6, "km/h"), NAME_LENGTH);
  snprintf(sensor_info[n++], NAME_LENGTH, "North velocity:           %s [%s]", speed_mi_h, speed_km_h);
  strncpy(speed_mi_h, numf2(pose->v_east * (3.6 / 1.609344), "m/h"), NAME_LENGTH);
  strncpy(speed_km_h, numf2(pose->v_east * 3.6, "km/h"), NAME_LENGTH);
  snprintf(sensor_info[n++], NAME_LENGTH, "East velocity:            %s [%s]", speed_mi_h, speed_km_h);
  strncpy(speed_mi_h, numf2(pose->v_up * (3.6 / 1.609344), "m/h"), NAME_LENGTH);
  strncpy(speed_km_h, numf2(pose->v_up * 3.6, "km/h"), NAME_LENGTH);
  snprintf(sensor_info[n++], NAME_LENGTH, "Up velocity:              %s [%s]", speed_mi_h, speed_km_h);

  snprintf(sensor_info[n++], NAME_LENGTH, " ");
  snprintf(sensor_info[n++], NAME_LENGTH, "DMI:                        %s", numf2(applanix_dmi.signed_odometer, "m"));

  snprintf(sensor_info[n++], NAME_LENGTH, " ");
  snprintf(sensor_info[n++], NAME_LENGTH, "Primimary gps:           %02d satellites", applanix_gps.primary_sats);
  snprintf(sensor_info[n++], NAME_LENGTH, "Secondary gps:         %02d satellite", applanix_gps.secondary_sats);
  snprintf(sensor_info[n++], NAME_LENGTH, "GAMS solution:         %d (%s)", applanix_gps.gams_solution_code,
      gams_solution[applanix_gps.gams_solution_code]);

  snprintf(sensor_info[n++], NAME_LENGTH, " ");
  snprintf(sensor_info[n++], NAME_LENGTH, "Yaw:                        %s", numf2(dgc_r2d(pose->yaw), "deg"));
  snprintf(sensor_info[n++], NAME_LENGTH, "Pitch:                       %s", numf2(dgc_r2d(pose->pitch), "deg"));
  snprintf(sensor_info[n++], NAME_LENGTH, "Roll:                         %s", numf2(dgc_r2d(pose->roll), "deg"));

  snprintf(sensor_info[n++], NAME_LENGTH, " ");
  snprintf(sensor_info[n++], NAME_LENGTH, "Yaw rate:                 %s", numf2(dgc_r2d(pose->ar_yaw), "deg/sec"));
  snprintf(sensor_info[n++], NAME_LENGTH, "Pitch rate:                %s", numf2(dgc_r2d(pose->ar_pitch), "deg/sec"));
  snprintf(sensor_info[n++], NAME_LENGTH, "Roll rate:                  %s", numf2(dgc_r2d(pose->ar_roll), "deg/sec"));

  snprintf(sensor_info[n++], NAME_LENGTH, " ");
  snprintf(sensor_info[n++], NAME_LENGTH, "Acceleration-X:       %.3f G", fabs(pose->a_x / 9.81));
  snprintf(sensor_info[n++], NAME_LENGTH, "Acceleration-Y:       %.3f G", fabs(pose->a_y / 9.81));
  snprintf(sensor_info[n++], NAME_LENGTH, "Acceleration-Z:       %.3f G", fabs(pose->a_z / 9.81));

  snprintf(sensor_info[n++], NAME_LENGTH, " ");
  snprintf(sensor_info[n++], NAME_LENGTH, "Smooth-X:               %.4f m", pose->smooth_x);
  snprintf(sensor_info[n++], NAME_LENGTH, "Smooth-Y:               %.4f m", pose->smooth_y);
  snprintf(sensor_info[n++], NAME_LENGTH, "Smooth-Z:               %.4f m", pose->smooth_z);
  snprintf(sensor_info[n++], NAME_LENGTH, " ");
  snprintf(sensor_info[n++], NAME_LENGTH, "Latitude:                   %.7f deg", pose->latitude);
  snprintf(sensor_info[n++], NAME_LENGTH, "Longitude:                %.7f deg", pose->longitude);
  snprintf(sensor_info[n++], NAME_LENGTH, "Altitude:                   %.4f m", pose->altitude);

  vlr::gui3D_help_window(n, sensor_info, GLUT_BITMAP_HELVETICA_12, show_sensor_info_layout);
}
