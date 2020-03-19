#ifndef DGC_RADAR_INTERFACE_H
#define DGC_RADAR_INTERFACE_H

#include <logio.h>
#include <ipc_interface.h>
#include <radar_messages.h>

namespace dgc {

char *StringToRadarSensor(char *string, RadarSensor *sensor);

char *StringToRadarLRR3Sensor(char *string, RadarLRR3Sensor *sensor);

void RadarAddLogReaderCallbacks(LogReaderCallbackList *callbacks);

void RadarLRR3AddLogReaderCallbacks(LogReaderCallbackList *callbacks);

void RadarSensor1Write(RadarSensor *sensor, double logger_timestamp, 
		       dgc_FILE *outfile);

void RadarSensor2Write(RadarSensor *sensor, double logger_timestamp, 
		       dgc_FILE *outfile);

void RadarSensor3Write(RadarSensor *sensor, double logger_timestamp, 
		       dgc_FILE *outfile);

void RadarSensor4Write(RadarSensor *sensor, double logger_timestamp, 
		       dgc_FILE *outfile);

void RadarSensor5Write(RadarSensor *sensor, double logger_timestamp, 
		       dgc_FILE *outfile);

void RadarSensor6Write(RadarSensor *sensor, double logger_timestamp, 
		       dgc_FILE *outfile);

void RadarSensor7Write(RadarSensor *sensor, double logger_timestamp, 
		       dgc_FILE *outfile);

void RadarSensor8Write(RadarSensor *sensor, double logger_timestamp, 
		       dgc_FILE *outfile);

void RadarSensor9Write(RadarSensor *sensor, double logger_timestamp, 
		       dgc_FILE *outfile);

void RadarSensor10Write(RadarSensor *sensor, double logger_timestamp, 
			dgc_FILE *outfile);

void RadarLRR3Sensor1Write(RadarLRR3Sensor *sensor, double logger_timestamp,
                           dgc_FILE *outfile);

void RadarLRR3Sensor2Write(RadarLRR3Sensor *sensor, double logger_timestamp,
                           dgc_FILE *outfile);

void RadarLRR3Sensor3Write(RadarLRR3Sensor *sensor, double logger_timestamp,
                           dgc_FILE *outfile);

void RadarLRR3Sensor4Write(RadarLRR3Sensor *sensor, double logger_timestamp,
                           dgc_FILE *outfile);

void RadarLRR3Sensor5Write(RadarLRR3Sensor *sensor, double logger_timestamp,
                           dgc_FILE *outfile);

void RadarLRR3Sensor6Write(RadarLRR3Sensor *sensor, double logger_timestamp,
                           dgc_FILE *outfile);

void RadarAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
				dgc_FILE *logfile,
				dgc_subscribe_t subscribe_how);

void RadarLRR3AddLogWriterCallbacks(IpcInterface *ipc, double start_time,
                                    dgc_FILE *logfile,
                                    dgc_subscribe_t subscribe_how);
  
}

#endif
