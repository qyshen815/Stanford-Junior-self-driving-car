#include <roadrunner.h> 
#include <passat_constants.h>
#include <fnmatch.h>
#include <transform.h>
#include "velocore.h"

dgc_velodyne_packet_p
dgc_velodyne_allocate_packet(void)
{
  dgc_velodyne_packet_p p;

  p = (dgc_velodyne_packet_p)malloc(sizeof(dgc_velodyne_packet_t));
  dgc_test_alloc(p);
  return p;
}

int
dgc_velodyne_parse_packet(unsigned char *pkt, unsigned short len,
			  dgc_velodyne_packet_p  p)
{
  int i, j, ptr = 0;
  
  if(len != VELO_PACKET_SIZE) {
    return 0;
  } 
  else {
    for (i=0; i<VELO_SCANS_IN_PACKET; i++) {
      memcpy(&(p->scan[i].block), &(pkt[ptr]), sizeof(unsigned short));
      ptr += sizeof(unsigned short);
      switch(p->scan[i].block) {
      case 0xeeff: // upper camera
	p->scan[i].block = 0;
	break;
      case 0xddff: // lower camera
	p->scan[i].block = 1;
	break;
      default: 
	dgc_warning("unknown camera id in the file: %4x\n", 
		    p->scan[i].block);
	return 0;
      }

      memcpy(&(p->scan[i].encoder), &(pkt[ptr]), sizeof(unsigned short));
      ptr += sizeof(unsigned short);

      for(j = 0; j < VELO_BEAMS_IN_SCAN; j++) {
	memcpy(&(p->scan[i].range[j]), &(pkt[ptr]), sizeof(unsigned short));
	ptr += sizeof(unsigned short);
	p->scan[i].intensity[j] = pkt[ptr];
	ptr++;
      }
    }
    
    memcpy(p->status,  &(pkt[ptr]), 6);
    ptr += 6;
  }
  return 1;
}

int
dgc_velodyne_ethereal(char *filename, 
		      dgc_velodyne_file_p *velodyne)
{
  unsigned char      header[24];
  unsigned int       magic;
  unsigned short int major, minor;
  unsigned int       sigfigs, snaplen, network;
  int                zone, n;

  *velodyne = 
    (dgc_velodyne_file_p) realloc( *velodyne, 
				   sizeof(dgc_velodyne_file_t) );
  dgc_test_alloc(*velodyne);
  
  (*velodyne)->filename = strdup(filename);
  
  (*velodyne)->fp = dgc_fopen(filename, "r");
  if((*velodyne)->fp == NULL) {
    free((*velodyne));
    fprintf(stderr, "Error: could not open file %s for reading.\n", filename);
    return FALSE;
  }
  
  n = dgc_fread(header, 24, 1, (*velodyne)->fp);
  if(n != 1) {
    fprintf(stderr, "Error: could not read header.\n");
    goto error;
  }
  
  magic = *((unsigned int *)header);
  if(magic != 0xa1b2c3d4) {
    fprintf(stderr, "Error: not a ethereal file\n");
    goto error;
  }

  major   = *((unsigned short int *)(header + 4));
  minor   = *((unsigned short int *)(header + 6));
  zone    = *((int *)(header + 8));
  sigfigs = *((unsigned int *)(header + 12));
  snaplen = *((unsigned int *)(header + 16));
  network = *((unsigned int *)(header + 20));

  (*velodyne)->msg_buffer = (unsigned char *)calloc(snaplen + 1, 1);
  dgc_test_alloc((*velodyne)->msg_buffer);

  (*velodyne)->sweep_number = 0;

  return TRUE;

 error:
  dgc_fclose((*velodyne)->fp);
  return FALSE;
}

int 
dgc_velodyne_ethereal_read_packet(dgc_velodyne_file_p velodyne,
				  dgc_velodyne_packet_p pkt)
{
  unsigned char   header[16];
  unsigned int    sec, usec;
  unsigned int    n, len1, len2;
  unsigned char * data;
  
  /* read pcap packet header */
  n = dgc_fread(header, 16, 1, velodyne->fp);
  if(n != 1) {
    fprintf(stderr, "Error: could not read log message.\n");
    return -1;
  }
  sec  = *((unsigned int *)header);
  usec = *((unsigned int *)(header + 4));
  pkt->timestamp = sec + usec / 1e6;
  len1 = *((unsigned int *)(header + 8));
  len2 = *((unsigned int *)(header + 12)); 
  
  /* read pcap packet data */
  n = dgc_fread(velodyne->msg_buffer, len1, 1, velodyne->fp);
  if(n != 1) {
    fprintf(stderr, "Error: could not read log message.\n");   
    return -1;
  }

  /* skip over the IP and UDP header and checksum*/
  data = velodyne->msg_buffer + 40 + 4;
  
  dgc_velodyne_parse_packet( data, n-44, pkt );

  return 0;
}


int
dgc_velodyne_vlf( char *filename, 
		  dgc_velodyne_file_p *velodyne )
{
  *velodyne = 
    (dgc_velodyne_file_p) realloc( *velodyne, 
				   sizeof(dgc_velodyne_file_t) );
  dgc_test_alloc(*velodyne);

  (*velodyne)->filename = strdup(filename);

  (*velodyne)->fp       = dgc_fopen(filename, "r");
  if((*velodyne)->fp == NULL) {
    fprintf(stderr, "Error: could not open file %s for reading.\n", filename);
    goto error;
  }
  
  (*velodyne)->buffer_len = 2*VELO_PACKET_SIZE;
  (*velodyne)->msg_buffer = 
    (unsigned char *)malloc((*velodyne)->buffer_len * sizeof(unsigned char));
  dgc_test_alloc((*velodyne)->msg_buffer);

  (*velodyne)->sweep_number = 0;

  return TRUE;

 error:
  free((*velodyne));
  return FALSE;
}

int 
dgc_velodyne_vlf_read_packet(dgc_velodyne_file_p velodyne, 
			     dgc_velodyne_packet_p pkt)
{
  unsigned char   data[16];
  int             n;
  unsigned short  len;
  
  n = dgc_fgetc(velodyne->fp);
  if(n != VLF_START_BYTE) {
    fprintf(stderr, "# ERROR: wrong start byte.\n");
    return -1;
  }
  
  n = dgc_fread(data, 8, 1, velodyne->fp);
  if(n != 1) {
    fprintf(stderr, "Error: could not read time stamp.\n");
    return -1;
  }
  memcpy( &(pkt->timestamp), data, 8 );

  n = dgc_fread(data, 2, 1, velodyne->fp);
  if(n != 1) {
    fprintf(stderr, "Error: could not read packet length.\n");
    return -1;
  }
  memcpy( &(len), data, 2 );

  if (len!=VELO_PACKET_SIZE) {
    fprintf(stderr, "# ERROR: packet has wrong size.\n");
    return -1;
  }

  n = dgc_fread(velodyne->msg_buffer, len+1, 1, velodyne->fp);
  if(n != 1) {
    return -1;
  }

  dgc_velodyne_parse_packet( velodyne->msg_buffer, len, pkt );
  return 0;
}

#define FILE_VLF_EXT            ".vlf"
#define FILE_PCAP_EXT           ".pcap"

#define MAX_NAME_LENGTH       256

dgc_velodyne_file_p dgc_velodyne_open_file(char *filename)
{
  VELODYNE_FILE_TYPE inp_type = UNKNOWN;
  char fname[MAX_NAME_LENGTH];
  char *completed_filename = NULL;
  dgc_velodyne_file_p velodyne = NULL;

  //  dgc_complete_filename(filename, &completed_filename);
  if(!fnmatch( "pcap:*", filename, 0)) {
    fprintf(stderr, "# INFO: use pcap file type!\n");
    strncpy(fname, &(filename[5]), MAX_NAME_LENGTH);
    inp_type = PCAP;
  } 
  else if(!fnmatch("vlf:*", filename, 0)) {
    fprintf(stderr, "# INFO: read vlf file type!\n");
    strncpy(fname, &(filename[4]), MAX_NAME_LENGTH);
    inp_type = VLF;
  } 
  else if(!fnmatch("*" FILE_PCAP_EXT, filename, 0)) {
    fprintf(stderr, "# INFO: read pcap file type!\n");
    strncpy(fname, filename, MAX_NAME_LENGTH);
    inp_type = PCAP;
  } 
  else if(!fnmatch("*" FILE_VLF_EXT, filename, 0)) {
    fprintf(stderr, "# INFO: read vlf file type!\n");
    strncpy(fname, filename, MAX_NAME_LENGTH);
    inp_type = VLF;
  } 
  else if(dgc_complete_filename(filename, FILE_VLF_EXT, &completed_filename)) {
    strncpy(fname, completed_filename, MAX_NAME_LENGTH);
    free(completed_filename);
    inp_type = VLF;
  } 
  else if(dgc_complete_filename(filename, FILE_PCAP_EXT, &completed_filename)) {
    strncpy(fname, completed_filename, MAX_NAME_LENGTH);
    free(completed_filename);
    inp_type = PCAP;
  } 

  switch(inp_type) {
  case PCAP:
    if(!dgc_velodyne_ethereal(fname, &velodyne)) {
      fprintf(stderr, "# ERROR: could not open VELO logfile.\n");
      return NULL;
    }
    velodyne->format = PCAP;
    return velodyne;
    break;
  case VLF:
    if(!dgc_velodyne_vlf(fname, &velodyne)) {
      fprintf(stderr, "# ERROR: could not open VELO logfile.\n");
      return NULL;
    }
    velodyne->format = VLF;
    return velodyne;
    break;
  default:
    fprintf(stderr, "# ERROR: unknown file type!\n");
    return NULL;
  }
}

short
beam_inside_car(double x, double y)
{
  if(fabs(y) < ((DGC_PASSAT_WIDTH / 2) + 1.2) &&
     x > -(DGC_PASSAT_IMU_TO_R_BUMPER + 1.0) &&
     x < DGC_PASSAT_LENGTH - DGC_PASSAT_IMU_TO_R_BUMPER) {
    return(1);
  } else {
    return(0);
  }

//  if ( fabs(y) < ((DGC_PASSAT_WIDTH+0.3)/2)    &&
//      x >  -DGC_PASSAT_IMU_TO_R_BUMPER        &&
//      x <  0.5 + DGC_PASSAT_LENGTH - DGC_PASSAT_IMU_TO_R_BUMPER ) {
//    return(1);
//  } else {
//    return(0);
//  }
}


void 
dgc_velodyne_project_measurement(dgc_velodyne_config_p       config,
				 dgc_velodyne_measurement_p  msrm,
				 dgc_velodyne_scan_p         scan,
				 dgc_pose_t                  robot)
{
  double x, y, z;
  dgc_transform_t t;
  int j, n;

  float distance, distance1, cosVertAngle, sinVertAngle, cosRotAngle, sinRotAngle, hOffsetCorr, vOffsetCorr;
  float xyDistance, distanceCorr, shortOffset, longOffset;

  dgc_transform_rpy(t, config->offset, robot.roll, robot.pitch, robot.yaw);

  for(j = 0; j < VELO_BEAMS_IN_SCAN; j++) {
    n = j + VELO_BEAMS_IN_SCAN * msrm->block;
	// use calibrated intensity!
    scan->p[j].intensity = config->intensity_map[config->inv_beam_order[n]][msrm->intensity[j]];
    if((msrm->range[j] == 0 || msrm->range[j] * VELODYNE_TICKS_TO_METER > 110 ||
	isnan(config->sin_rot_angle[msrm->encoder][n]))) {
      scan->p[j].x       = 0;
      scan->p[j].y       = 0;
      scan->p[j].z       = 0;
      scan->p[j].range   = 0;
    } 
    else {
	distance1 = msrm->range[j] * VELODYNE_TICKS_TO_METER;
	distance  = config->range_offsetX[n] * distance1 + config->range_offset[n];

	cosVertAngle = config->cos_vert_angle[n];
	sinVertAngle = config->sin_vert_angle[n];
	cosRotAngle = config->cos_rot_angle[msrm->encoder][n];
	sinRotAngle = config->sin_rot_angle[msrm->encoder][n];
	hOffsetCorr = config->h_offset[n];
	vOffsetCorr = config->v_offset[n];

	xyDistance = distance * cosVertAngle;
	
	x = xyDistance * cosRotAngle - hOffsetCorr * sinRotAngle;
	y = xyDistance * sinRotAngle + hOffsetCorr * cosRotAngle;
	z = (xyDistance / cosVertAngle) * sinVertAngle + vOffsetCorr;

#ifdef CHECK_INSIDE_CAR
      if(beam_inside_car(x, y)) {
	scan->p[j].x       = 0;
	scan->p[j].y       = 0;
	scan->p[j].z       = 0;
	scan->p[j].range   = 0;
      } 
      else {
#endif
	dgc_transform_point(&x, &y, &z, t);
	scan->p[j].x     = (short) (x*100 + .0);
	scan->p[j].y     = (short) (y*100 + .0);
	scan->p[j].z     = (short) (z*100 + .0);
	scan->p[j].range = (unsigned short) (xyDistance*100);
#ifdef CHECK_INSIDE_CAR
      }
#endif
    }
  }
  scan->robot       = robot;
  scan->block       = msrm->block;
  scan->encoder     = msrm->encoder;
  //  scan->timestamp   = velodyne->ts;
}

