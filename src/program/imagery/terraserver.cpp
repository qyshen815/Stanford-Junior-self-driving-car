#include <roadrunner.h>
#include <lltransform.h>

void terraserver_url(int terra_res_code, int terra_easting, 
		     int terra_northing, int zone_num, 
		     int image_type, char *url)
{
  sprintf(url,
	  //	  "http://terraserver.microsoft.com/tile.ashx?T=%d&S=%d&X=%d&Y=%d&Z=%d", 
	  "http://terraserver-usa.com/tile.ashx?T=%d&S=%d&X=%d&Y=%d&Z=%d", 
	  image_type, terra_res_code, terra_easting, terra_northing, zone_num);
}

void terra_coords(double easting, double northing, char *zone,
		  double image_resolution, int *terra_res_code, 
		  int *terra_easting, int *terra_northing, int *zone_num)
{
  char zone_str[10];
  int utm_multiplier;
  
  if(image_resolution == 0.25)
    *terra_res_code = 8;
  else if(image_resolution == 0.5)
    *terra_res_code = 9;
  else if(image_resolution == 1)
    *terra_res_code = 10;
  else if(image_resolution == 2)
    *terra_res_code = 11;
  else if(image_resolution == 4)
    *terra_res_code = 12;
  else if(image_resolution == 8)
    *terra_res_code = 13;
  else if(image_resolution == 16)
    *terra_res_code = 14;
  else if(image_resolution == 32)
    *terra_res_code = 15;
  else if(image_resolution == 64)
    *terra_res_code = 16;
  else if(image_resolution == 128)
    *terra_res_code = 17;
  else if(image_resolution == 256)
    *terra_res_code = 18;
  else if(image_resolution == 512)
    *terra_res_code = 19;
  else
    dgc_die("Error: invalid resolution (%f)\n", image_resolution);
  utm_multiplier = image_resolution * 200;
  
  strncpy(zone_str, zone, strlen(zone) - 1);
  zone_str[strlen(zone) - 1] = '\0';
  *zone_num = atoi(zone_str);
  *terra_easting = (int)floor(easting / utm_multiplier);
  *terra_northing = (int)floor(northing / utm_multiplier);
}

