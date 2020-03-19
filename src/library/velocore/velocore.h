#ifndef DGC_VELO_H
#define DGC_VELO_H

#include <transform.h>
#include <dgc_stdio.h>

typedef enum { UNKNOWN, PCAP, VLF } VELODYNE_FILE_TYPE;

#define VELODYNE_TICKS_TO_METER     0.002

#define VELO_NUM_LASERS          64
#define VELO_NUM_TICKS           36000
#define VELO_SPIN_START          18000

#define VELO_SCANS_IN_PACKET     12
#define VELO_BEAMS_IN_SCAN       32

#define VELO_PACKET_SIZE         1206

#define VLF_START_BYTE           0x1b
#define VELO_PACKET_FSIZE        1226         /* 1 + 16 + 2 + 1206 + 1 */

/* 
   one packet is 

   one scan =  VELO_BEAMS_IN_SCAN * 3 + 4 [enc+block+n*3] = 100 bytes
   packet = VELO_SCANS_IN_PACKET * 100 + status (6 bytes) = 1206 bytes
   
   file:
   
   startbyte  timestamp length  [... DATA ...] checksum
   
   => additional 1+16+2+1 = 20 bytes
   
     startbyte = 0x1b
     timestamp = 2 x <unsigned long>  [tv_sec and tv_usec]
     length    = <unsigned short>
     ...
     checksum  = <unsigned char>
*/

/************************************************************************
 *
 *  STRUCTURE WITH RAW DATA FROM SCANNER
 *
 ************************************************************************/

typedef struct {

  unsigned short                encoder;
  unsigned short                block;
  unsigned short                range[VELO_BEAMS_IN_SCAN];
  unsigned char                 intensity[VELO_BEAMS_IN_SCAN];

} dgc_velodyne_measurement_t, *dgc_velodyne_measurement_p;

typedef struct {

  double                        timestamp;
  dgc_velodyne_measurement_t    scan[VELO_SCANS_IN_PACKET];
  unsigned char                 status[6];

} dgc_velodyne_packet_t, *dgc_velodyne_packet_p;


/************************************************************************
 *
 *  STRUCTURE WITH LOG FILE DATA
 *
 ************************************************************************/

typedef struct {

  VELODYNE_FILE_TYPE           format;

  dgc_FILE                    *fp;
  char                        *filename;

  int                          buffer_len;
  unsigned char               *msg_buffer;

  // variables that change infrequently
  int                           sweep_number;

} dgc_velodyne_file_t, * dgc_velodyne_file_p;

/************************************************************************
 *
 ************************************************************************/

typedef struct {

  // variables that never change 
  dgc_transform_t           offset;
  double                    range_offset[VELO_NUM_LASERS];
  double                    range_offsetX[VELO_NUM_LASERS];
  double                    range_offsetY[VELO_NUM_LASERS];
  char                      laser_enabled[VELO_NUM_LASERS];
  double                    global_range_offset;
  double                    vert_angle[VELO_NUM_LASERS];
  double                    rot_angle[VELO_NUM_LASERS];
  double                    h_offset[VELO_NUM_LASERS];
  double                    v_offset[VELO_NUM_LASERS];
  double                    sin_vert_angle[VELO_NUM_LASERS];
  double                    cos_vert_angle[VELO_NUM_LASERS];
  double                    enc_rot_angle[VELO_NUM_TICKS][VELO_NUM_LASERS];
  double                    sin_rot_angle[VELO_NUM_TICKS][VELO_NUM_LASERS];
  double                    cos_rot_angle[VELO_NUM_TICKS][VELO_NUM_LASERS];
  double                    sin_enc_angle[VELO_NUM_TICKS];
  double                    cos_enc_angle[VELO_NUM_TICKS];
  double                    enc_angle[VELO_NUM_TICKS];
  double                    range_multiplier;
  int                       min_intensity;
  int                       max_intensity;
  double                    intensity_map[VELO_NUM_LASERS][256];
  int                       beam_order[VELO_NUM_LASERS];
  int                       inv_beam_order[VELO_NUM_LASERS];
  int                       spin_start;

} dgc_velodyne_config_t, *dgc_velodyne_config_p;

/************************************************************************
 *
 ************************************************************************/


typedef struct {

  /* the unit here is 10mm(!). Do x * 0.01 to convert
     to m.  */
  short                     x;
  short                     y;
  short                     z;
  /* use the units here are 10mm (1). Do x * 0.01 to convert to m */
  unsigned short            range;
  unsigned char             intensity;

} dgc_velodyne_point_t, * dgc_velodyne_point_p;

typedef struct {

  // global position of the measurement
  dgc_velodyne_point_t      p[VELO_BEAMS_IN_SCAN];
  //
  double                    timestamp;
  //
  dgc_pose_t                robot;
  // which block (upper/lower lasers) is firing
  unsigned char             block;
  unsigned short            encoder;
  // counter that indicates which scans belongs to the same revolution
  unsigned short            counter;

} dgc_velodyne_scan_t, *dgc_velodyne_scan_p;

/************************************************************************
 *
 ************************************************************************/

int 
dgc_velodyne_ethereal_read_packet(dgc_velodyne_file_p velodyne,
				  dgc_velodyne_packet_p pkt);

int 
dgc_velodyne_vlf_read_packet(dgc_velodyne_file_p velodyne, 
			     dgc_velodyne_packet_p pkt);

  /* only call functions below this line */

void 
dgc_velodyne_integrate_offset(dgc_transform_t offset,
                              dgc_velodyne_config_p config);

void
dgc_velodyne_get_config(dgc_velodyne_config_p *config);

int 
dgc_velodyne_read_calibration(char *filename,
                              dgc_velodyne_config_p config);

int
dgc_velodyne_read_intensity(char *filename,
                              dgc_velodyne_config_p config);

void 
dgc_velodyne_print_calibration_data(dgc_velodyne_config_p config);

dgc_velodyne_packet_p
dgc_velodyne_allocate_packet(void);

int
dgc_velodyne_parse_packet(unsigned char *pkt, unsigned short len,
			  dgc_velodyne_packet_p  p);

dgc_velodyne_file_p
dgc_velodyne_open_file(char *filename);

inline int dgc_velodyne_read_packet(dgc_velodyne_file_t* velodyne, dgc_velodyne_packet_t* pkt) {
  if(velodyne->format == PCAP) {
    return dgc_velodyne_ethereal_read_packet(velodyne, pkt);
  }
  else {
    return dgc_velodyne_vlf_read_packet(velodyne, pkt);
  }
}
  
void 
dgc_velodyne_project_measurement(dgc_velodyne_config_p config,
				 dgc_velodyne_measurement_p msrm,
				 dgc_velodyne_scan_p scan,
				 dgc_pose_t robot);

#endif
