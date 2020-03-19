#ifndef DGC_TERRASERVER_H
#define DGC_TERRASERVER_H

#define      BW_IMAGE        1
#define      TOPO_IMAGE      2
#define      COLOR_IMAGE     4

void terraserver_url(int terra_res_code, int terra_easting, 
		     int terra_northing, int zone_num, 
		     int image_type, char *url);

void terra_coords(double easting, double northing, char *zone,
		  double image_resolution, int *terra_res_code, 
		  int *terra_easting, int *terra_northing, int *zone_num);

#endif
