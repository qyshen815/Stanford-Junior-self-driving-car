#include <roadrunner.h> 
#include "velocore.h"
#include "defaultcal.h"

typedef struct {
  double    angle;
  int       idx;
} beam_angle_t;

static int
beamCompare(const void *a, const void *b)
{
  static beam_angle_t v1, v2;
  v1 = *(beam_angle_t *)a; 
  v2 = *(beam_angle_t *)b;
  if(v1.angle > v2.angle)
    return 1;
  else if(v1.angle == v2.angle)
    return 0;
  else
    return -1;
}

static void 
dgc_velodyne_find_beam_order(dgc_velodyne_config_p config)
{
  beam_angle_t beams[VELO_NUM_LASERS];
  int i;
  
  for(i = 0; i < VELO_NUM_LASERS; i++) {
    beams[i].angle = config->vert_angle[i];
    beams[i].idx   = i;
  }
 
  qsort(beams, VELO_NUM_LASERS, sizeof(beam_angle_t), beamCompare);

  for(i = 0; i < VELO_NUM_LASERS; i++) {
    config->beam_order[i] = beams[i].idx;
    config->inv_beam_order[beams[i].idx] = i;
  }
}

void 
dgc_velodyne_integrate_offset(dgc_transform_t offset,
			      dgc_velodyne_config_p config)
{
  dgc_transform_copy(config->offset, offset);
}

void 
dgc_velodyne_recompute_angles(dgc_velodyne_config_p config) 
{
  int i, j;
  double angle = 0;
  double laser_offset_yaw = 0.0;

  for(i = 0; i < 64; i++) { 
    config->cos_vert_angle[i] = cos(config->vert_angle[i]);
    config->sin_vert_angle[i] = sin(config->vert_angle[i]);

    for(j = 0; j < VELO_NUM_TICKS; j++) {
      angle = dgc_normalize_theta(laser_offset_yaw -
				  j / (float)VELO_NUM_TICKS * 2.0 * M_PI + 
				  config->rot_angle[i]);
      config->cos_rot_angle[j][i] = cos(angle);
      config->sin_rot_angle[j][i] = sin(angle);
    }
  }
  for(j = 0; j < VELO_NUM_TICKS; j++) {
    angle = dgc_normalize_theta(laser_offset_yaw - j / (float)VELO_NUM_TICKS * 2.0 * M_PI);
    config->cos_enc_angle[j] = cos(angle);
    config->sin_enc_angle[j] = sin(angle);
  }
}

void 
dgc_velodyne_autoconfig(dgc_velodyne_config_p config) 
{
  int i;

  config->global_range_offset = 0;
  config->range_multiplier    = 1.0;
  
  for(i = 0; i < 64; i++ ) {
    config->rot_angle[i] = dgc_d2r(DEFAULT_VELO_ROT_ANGLE[i]);
    config->vert_angle[i] = dgc_d2r(DEFAULT_VELO_VERT_ANGLE[i]);
    config->range_offset[i] = 0; 
    config->laser_enabled[i] = 1;
  }
}

void
dgc_velodyne_get_config(dgc_velodyne_config_p *config)
{
  *config = 
    (dgc_velodyne_config_p)calloc(1, sizeof(dgc_velodyne_config_t));
  dgc_test_alloc(*config);
  dgc_velodyne_autoconfig(*config);
  dgc_velodyne_recompute_angles(*config);
  dgc_velodyne_find_beam_order(*config);
}

#define MAX_LINE_LENGTH    512

int 
dgc_velodyne_read_intensity(char *filename, dgc_velodyne_config_p config)
{
  FILE *iop;
  int i,j;
	
  if(!filename) { // not using calibrated intensities so skip this
	return 0;
  }

  char *expanded_filename = NULL;
  expanded_filename = dgc_expand_filename(filename);
  if(expanded_filename == NULL) {
    fprintf(stderr, 
	    "[31;1m# ERROR: could not expand filename %s[0m\n", 
	    filename);
    return 1;
  }
  else if ((iop = fopen(expanded_filename, "r")) == 0){
    fprintf(stderr, 
	    "[31;1m# ERROR: could not open velodyne intensity calibration file %s[0m\n", 
	    filename );
    return 1;
  }
  fprintf(stderr, "# INFO: read velodyne intensity calibration file %s\n", filename); 
  free(expanded_filename);

  int dummy = fscanf(iop, "%d %d\n", &config->min_intensity, &config->max_intensity);
  for(i = 0; i < 64; i++) {
    for(j = 0; j < 256; j++) {
      dummy = fscanf(iop, "%lf ", &config->intensity_map[i][j]);
      float expanded = (config->intensity_map[i][j] - config->min_intensity) / (config->max_intensity - config->min_intensity);
      if(expanded < 0)
	expanded = 0;
      if(expanded > 1)
	expanded = 1;
      config->intensity_map[i][j] = (unsigned char) (255 * expanded);
    }
  }
  fclose(iop);
  //printf("New min:%d    New max: %d\n", config->min_intensity, config->max_intensity);
  return 0;
}


int 
dgc_velodyne_read_calibration(char *filename, dgc_velodyne_config_p config)
{
  FILE   * iop;

  int      FEnd;
  int      linectr = 0;
  int      n, id, enabled;
  int      i, j;
  double   rcf, hcf, hoff, voff, dist, distX, distY;

  char     command[MAX_LINE_LENGTH];
  char     line[MAX_LINE_LENGTH];
  char     str1[MAX_LINE_LENGTH];
  char     str2[MAX_LINE_LENGTH];
  char     str3[MAX_LINE_LENGTH];
  char     str4[MAX_LINE_LENGTH];
  char     str5[MAX_LINE_LENGTH];
  char     str6[MAX_LINE_LENGTH];
  char     str7[MAX_LINE_LENGTH];
  char     str8[MAX_LINE_LENGTH];
  char     str9[MAX_LINE_LENGTH];
  float    range[64];
  char *expanded_filename = NULL;

  config->min_intensity = 0;
  config->max_intensity = 255;
  for(i = 0; i < 64; i++) {
	for(j = 0; j < 256; j++) {
		config->intensity_map[i][j] = j;
	}
  }
  for (n=0; n<64; n++) {
    range[n] = 0.0;
  }
  config->range_multiplier = 1.0;
  config->spin_start = VELO_SPIN_START;
 
  expanded_filename = dgc_expand_filename(filename);
  if(expanded_filename == NULL) {
    fprintf(stderr, 
	    "[31;1m# ERROR: could not expand filename %s[0m\n", 
	    filename);
    return 1;
  }
  else if ((iop = fopen(expanded_filename, "r")) == 0){
    fprintf(stderr, 
	    "[31;1m# ERROR: could not open velodyne calibration file %s[0m\n", 
	    filename );
    return 1;
  }
  fprintf(stderr, "# INFO: read velodyne calibration file %s\n", filename); 
  free(expanded_filename);

  FEnd=0;
  do{
    if (fgets( line, MAX_LINE_LENGTH, iop) == NULL)
      FEnd=1;
    else{
      linectr++;
      if (sscanf(line, "%s", command) == 0) {
	fclose(iop);
	return 1;
      } else {
	if (command[0]!='#'){
	  n = sscanf(line, "%s %s %s %s %s %s %s %s %s",
		     str1, str2, str3, str4, str5, str6, str7, str8, str9);
	  if (n==9) {
	    id      = atoi(str1);
	    rcf     = atof(str2);
	    hcf     = atof(str3);
	    dist    = atof(str4);
	    distX   = atof(str5);
	    distY   = atof(str6);
	    voff    = atof(str7);
	    hoff    = atof(str8);
	    enabled = atoi(str9);
	    if (id<0 || id>63) {
	      fprintf( stderr, "[31;1m# ERROR: wrong id '%d' in line %d[0m\n", 
		       id, linectr );
	      fclose(iop);
	      return 1;
	    } else {
	      config->rot_angle[id]     = dgc_d2r( rcf );
	      config->vert_angle[id]    = dgc_d2r( hcf );
	      config->range_offset[id]  = dist;
	      config->range_offsetX[id]  = distX;
	      config->range_offsetY[id]  = distY;
	      config->laser_enabled[id] = enabled;
	      config->v_offset[id]      = voff;
	      config->h_offset[id]      = hoff;
	    }
	  } else if (n==2) {
	    if (!strcasecmp(str1,"RANGE_MULTIPLIER")) {
	      config->range_multiplier = atof(str2);
      } else if (!strcasecmp(str1,"SPIN_START")) {
         config->spin_start = atoi(str2);
	    } else {
	      fprintf( stderr, "[31;1m# ERROR: unknown keyword '%s' in line %d[0m\n", 
		       str1, linectr );
	      fclose(iop);
	      return 1;
	    }
	  } else {
	    fprintf( stderr, "[31;1m# ERROR: error in line %d: %s[0m\n", 
		     linectr, line );
	    fclose(iop);
	    return 1;
	  }
	}
      }
    }
  } while (!FEnd);
  
  fclose(iop);

  dgc_velodyne_recompute_angles(config);
  dgc_velodyne_find_beam_order(config);

  return 0;
}

void 
dgc_velodyne_print_calibration_data(dgc_velodyne_config_p config) 
{
  int i;

  printf("\ndouble VELO_ROT_ANGLE2[64] = { \n");  
  for( i = 0; i < 16; i++ ) {
    printf("                                    %12.9f,%12.9f,%12.9f,%12.9f,\n", 
           dgc_r2d(config->rot_angle[4*i+0]), 
           dgc_r2d(config->rot_angle[4*i+1]), 
           dgc_r2d(config->rot_angle[4*i+2]), 
           dgc_r2d(config->rot_angle[4*i+3]) );
  }
  printf("                                  };\n");  

  printf("double VELO_VERT_ANGLE2[64] = { \n");  
  for( i = 0; i < 16; i++ ) {
    printf("                                    %12.9f,%12.9f,%12.9f,%12.9f,\n", 
           dgc_r2d(config->vert_angle[4*i+0]), 
           dgc_r2d(config->vert_angle[4*i+1]), 
           dgc_r2d(config->vert_angle[4*i+2]), 
           dgc_r2d(config->vert_angle[4*i+3]) );
  }
  printf("                                  };\n");  

  printf("int VELO_RANGE_OFFSET2[64] = { \n");  
  for( i = 0; i < 16; i++ ) {
    printf("                                    %3f,%3f,%3f,%3f,\n", 
           config->range_offset[4*i+0], 
           config->range_offset[4*i+1], 
           config->range_offset[4*i+2], 
           config->range_offset[4*i+3] );
  }
  printf("                                  };\n");  

  printf("char VELO_LASER_ENABLED2[64] = { \n");
  for( i = 0; i < 16; i++ ) {
    printf("                                    %2d,%2d,%2d,%2d,\n", 
           config->laser_enabled[4*i+0], 
           config->laser_enabled[4*i+1], 
           config->laser_enabled[4*i+2], 
           config->laser_enabled[4*i+3] );
  }
  printf("                                  };\n");  
}


