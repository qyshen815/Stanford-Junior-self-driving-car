#include <roadrunner.h>
#include <sys/dir.h>
#include <lltransform.h>

int image_count = 0;
double min_lon = 1e6, max_lon = -1e6, min_lat = 1e6, max_lat = -1e6;

int is_jpg(de_const_ struct direct *entry)
{
  if(strlen(entry->d_name) > 4 &&
     strcmp(entry->d_name + strlen(entry->d_name) - 4, ".jpg") == 0)
    return TRUE;
  else
    return FALSE;
}

void check_images(char *dir_name)
{
  struct dirent **namelist;
  int n, i = 0, j, dash_count;
  char field[4][100], utmzone[10];
  char *mark, *mark2;
  double x, y, resolution, image_lat1, image_lon1, image_lat2, image_lon2;

  n = scandir(dir_name, &namelist, is_jpg, NULL);
  i = 0;
  if(n < 0)
    perror("scandir");
  else {
    while(n--) {
      dash_count = 0;
      for(j = 0; j < (int)strlen(namelist[n]->d_name); j++)
	if(namelist[n]->d_name[j] == '-')
	  dash_count++;
      
      if(dash_count == 4) {
	mark = strchr(namelist[n]->d_name, '-') + 1;
	for(j = 0; j < 4; j++) {
	  if(j < 3) 
	    mark2 = strchr(mark, '-');
	  else
	    mark2 = strchr(mark, '.');
	  strncpy(field[j], mark, mark2 - mark);
	  field[j][mark2 - mark] = '\0';
	  mark = mark2 + 1;
	}
	
	resolution = atof(field[0]);
	if(resolution <= 4) {
	  x = atof(field[1]) * 200 * resolution;
	  y = atof(field[2]) * 200 * resolution;
	  sprintf(utmzone, "%s%s", field[3], "S");
	  
	  vlr::utmToLatLong(x, y, utmzone, &image_lat1, &image_lon1);
	  vlr::utmToLatLong(x + 200 * resolution, y + 200 * resolution,
		      utmzone, &image_lat2, &image_lon2);
	  
	  if(image_lat1 < min_lat)
	    min_lat = image_lat1;
	  if(image_lon1 < min_lon)
	    min_lon = image_lon1;
	  
	  if(image_lat2 > max_lat)
	    max_lat = image_lat2;
	  if(image_lon2 > max_lon)
	    max_lon = image_lon2;
	}
      }
      image_count++;
      free(namelist[n]);
      i++;
    }
    free(namelist);
  }  
}

int main(int argc, char **argv)
{
  struct dirent **namelist;
  struct stat file_stat;
  char imagery_dir[200], filename[200];
  int n, i = 0;
  FILE *fp;

  if(argc < 2)
    dgc_die("Error: not enough arguments.\n"
	    "Usage: %s imagery-dir\n", argv[0]);

  /* exit if bound.txt already exists */
  sprintf(filename, "%s/bound.txt", argv[1]);
  if(dgc_file_exists(filename)) 
    dgc_die("Error: file %s already exists!\n", filename);

  /* make sure 200/color dir exists */
  sprintf(imagery_dir, "%s/200/color", argv[1]);
  if(!dgc_file_exists(imagery_dir)) 
    dgc_die("Error: could not find USGS 200 subdir.\n");
  
  /* scan through all sub-directories */
  n = scandir(imagery_dir, &namelist, NULL, NULL);
  i = 0;
  if(n < 0)
    perror("scandir");
  else {
    while(n--) {
      if(strcmp(namelist[n]->d_name, ".") != 0 &&
	 strcmp(namelist[n]->d_name, "..") != 0) {
	sprintf(filename, "%s/%s", imagery_dir, namelist[n]->d_name);
	stat(filename, &file_stat);
	if(S_ISDIR(file_stat.st_mode)) 
	  check_images(filename);
      }

      free(namelist[n]);
      i++;
    }
    free(namelist);
  }  

  fprintf(stderr, "Found %d images : %f %f -> %f %f\n", image_count,
	  min_lat, min_lon, max_lat, max_lon);
  
  char utmzone[10];
  double min_x, min_y, max_x, max_y;

  vlr::latLongToUtm(min_lat, min_lon, &min_x, &min_y, utmzone);
  vlr::latLongToUtm(max_lat, max_lon, &max_x, &max_y, utmzone);

  sprintf(filename, "%s/bound.txt", argv[1]);
  fprintf(stderr, "Writing file %s\n", filename);
  fp = fopen(filename, "w");
  if(fp == NULL)
    dgc_die("Error: could not open file %s for writing.\n", filename);
  fprintf(fp, "%f\n%f\n%f\n%f\n", min_lat, min_lon, max_lat, max_lon);
  fclose(fp);
  return 0;
}
