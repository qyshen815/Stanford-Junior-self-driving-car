#include <roadrunner.h>
#include <sys/dir.h>

char imagery_src_dir[500];
char imagery_dest_dir[500];

int is_img(de_const_ direct *entry)
{
  if(strlen(entry->d_name) > 4 &&
     (strcmp(entry->d_name + strlen(entry->d_name) - 4, ".jpg") == 0 ||
      strcmp(entry->d_name + strlen(entry->d_name) - 4, ".gif") == 0))
    return TRUE;
  else
    return FALSE;
}

void parse_fields(char *str, char field[20][100], char extension[100],
		  int *num_fields)
{
  char filename[500], *mark, *mark2;
  int j;

  strcpy(filename, str);
  if(dgc_file_extension(filename) != NULL)
    strcpy(extension, dgc_file_extension(filename));
  else
    extension[0] = '\0';
  filename[strlen(filename) - strlen(extension)] = '\0';
  
  *num_fields = 1;
  for(j = 0; j < (int)strlen(filename); j++)
    if(filename[j] == '-')
      (*num_fields)++;
  
  if(*num_fields > 0) {
    mark = filename;
    for(j = 0; j < *num_fields; j++) {
      mark2 = strchr(mark, '-');
      if(mark2 == NULL)
	strcpy(field[j], mark);
      else {
	strncpy(field[j], mark, mark2 - mark);
	field[j][mark2 - mark] = '\0';
	mark = mark2 + 1;
      }
    }
  }
}

void process_usgs_color_dir(char *dirname)
{
  int terra_res_code = 0, terra_x, terra_y, terra_zone;
  char filename[500], new_filename[500], cmd[1000];
  struct dirent **namelist;
  double image_resolution;
  int i, n, num_fields;
  char extension[100];
  char field[20][100];

  sprintf(new_filename, "%s/usgs", imagery_dest_dir);
  if(!dgc_file_exists(new_filename)) 
    mkdir(new_filename, 0755);
  sprintf(new_filename, "%s/usgs/4", imagery_dest_dir);
  if(!dgc_file_exists(new_filename)) 
    mkdir(new_filename, 0755);

  n = scandir(dirname, &namelist, is_img, NULL);
  i = 0;
  if(n < 0)
    perror("scandir");
  else {
    while(n--) {
      parse_fields(namelist[n]->d_name, field, extension, &num_fields);
      if(num_fields != 5)
	dgc_die("Error: filename %s has %d fields.\n", namelist[n]->d_name,
		num_fields);
      if(strcmp(field[0], "usgs") != 0)
	dgc_die("Error: filename %s has wrong first field %s.\n", 
		namelist[n]->d_name, field[0]);

      image_resolution = atof(field[1]);
      if(image_resolution == 0.25)
	terra_res_code = 8;
      else if(image_resolution == 0.5)
	terra_res_code = 9;
      else if(image_resolution == 1)
	terra_res_code = 10;
      else if(image_resolution == 2)
	terra_res_code = 11;
      else if(image_resolution == 4)
	terra_res_code = 12;
      else if(image_resolution == 8)
	terra_res_code = 13;
      else if(image_resolution == 16)
	terra_res_code = 14;
      else if(image_resolution == 32)
	terra_res_code = 15;
      else if(image_resolution == 64)
	terra_res_code = 16;
      else if(image_resolution == 128)
	terra_res_code = 17;
      else if(image_resolution == 256)
	terra_res_code = 18;
      else if(image_resolution == 512)
	terra_res_code = 19;
      else
	dgc_die("Error: invalid resolution (%f)\n", image_resolution);

      terra_x = atoi(field[2]);
      terra_y = atoi(field[3]);
      terra_zone = atoi(field[4]);

      sprintf(new_filename, "%s/usgs/4/%d/", imagery_dest_dir, terra_res_code);
      if(!dgc_file_exists(new_filename)) 
	mkdir(new_filename, 0755);

      sprintf(new_filename, "%s/usgs/4/%d/%d/", imagery_dest_dir, 
	      terra_res_code, terra_x);
      if(!dgc_file_exists(new_filename)) 
	mkdir(new_filename, 0755);

      sprintf(new_filename, "%s/usgs/4/%d/%d/usgs-%d-%d-%d-%d%s",
	      imagery_dest_dir, terra_res_code, terra_x, terra_res_code,
	      terra_x, terra_y, terra_zone, extension);

      /*      fprintf(stderr, "FIELDS : ");
      for(int j = 0; j < num_fields; j++) {
	fprintf(stderr, "\"%s\" ", field[j]);
      }
      fprintf(stderr, "extension \"%s\"\n", extension);*/

      sprintf(filename, "%s/%s", dirname, namelist[n]->d_name);
      fprintf(stderr, "\r      Image file %s            ", filename);
      
      /* actually copy the image */
      sprintf(cmd, "cp %s %s", filename, new_filename);
      if(system(cmd) == -1)
        dgc_error("Failed to run command %s", cmd);

      free(namelist[n]);
      i++;
    }
    free(namelist);
  }  
}

void process_usgs_topo_dir(char *dirname)
{
  int terra_res_code = 0, terra_x, terra_y, terra_zone;
  char filename[500], new_filename[500], cmd[1000];
  struct dirent **namelist;
  double image_resolution;
  int i, n, num_fields;
  char extension[100];
  char field[20][100];

  sprintf(new_filename, "%s/usgs", imagery_dest_dir);
  if(!dgc_file_exists(new_filename)) 
    mkdir(new_filename, 0755);
  sprintf(new_filename, "%s/usgs/2", imagery_dest_dir);
  if(!dgc_file_exists(new_filename)) 
    mkdir(new_filename, 0755);

  n = scandir(dirname, &namelist, is_img, NULL);
  i = 0;
  if(n < 0)
    perror("scandir");
  else {
    while(n--) {
      parse_fields(namelist[n]->d_name, field, extension, &num_fields);
      if(num_fields != 5)
	dgc_die("Error: filename %s has %d fields.\n", namelist[n]->d_name,
		num_fields);
      if(strcmp(field[0], "usgs") != 0)
	dgc_die("Error: filename %s has wrong first field %s.\n", 
		namelist[n]->d_name, field[0]);

      image_resolution = atof(field[1]);
      if(image_resolution == 0.25)
	terra_res_code = 8;
      else if(image_resolution == 0.5)
	terra_res_code = 9;
      else if(image_resolution == 1)
	terra_res_code = 10;
      else if(image_resolution == 2)
	terra_res_code = 11;
      else if(image_resolution == 4)
	terra_res_code = 12;
      else if(image_resolution == 8)
	terra_res_code = 13;
      else if(image_resolution == 16)
	terra_res_code = 14;
      else if(image_resolution == 32)
	terra_res_code = 15;
      else if(image_resolution == 64)
	terra_res_code = 16;
      else if(image_resolution == 128)
	terra_res_code = 17;
      else if(image_resolution == 256)
	terra_res_code = 18;
      else if(image_resolution == 512)
	terra_res_code = 19;
      else
	dgc_die("Error: invalid resolution (%f)\n", image_resolution);

      terra_x = atoi(field[2]);
      terra_y = atoi(field[3]);
      terra_zone = atoi(field[4]);

      sprintf(new_filename, "%s/usgs/2/%d/", imagery_dest_dir, terra_res_code);
      if(!dgc_file_exists(new_filename)) 
	mkdir(new_filename, 0755);

      sprintf(new_filename, "%s/usgs/2/%d/%d/", imagery_dest_dir, 
	      terra_res_code, terra_x);
      if(!dgc_file_exists(new_filename)) 
	mkdir(new_filename, 0755);

      sprintf(new_filename, "%s/usgs/2/%d/%d/usgs-%d-%d-%d-%d%s",
	      imagery_dest_dir, terra_res_code, terra_x, terra_res_code,
	      terra_x, terra_y, terra_zone, extension);

      /*      fprintf(stderr, "FIELDS : ");
      for(int j = 0; j < num_fields; j++) {
	fprintf(stderr, "\"%s\" ", field[j]);
      }
      fprintf(stderr, "extension \"%s\"\n", extension);*/

      sprintf(filename, "%s/%s", dirname, namelist[n]->d_name);
      fprintf(stderr, "\r      Image file %s            ", filename);
      
      /* actually copy the image */
      sprintf(cmd, "cp %s %s", filename, new_filename);
      if(system(cmd) == -1)
        dgc_error("Failed to run command %s", cmd);

      free(namelist[n]);
      i++;
    }
    free(namelist);
  }  
}

void process_laser_dir(char *dirname)
{
  char filename[500], new_filename[500], cmd[1000];
  struct dirent **namelist;
  int laser_resolution, laser_x;
  int i, n, num_fields;
  char extension[100];
  char field[20][100];

  sprintf(new_filename, "%s/laser/", imagery_dest_dir);
  if(!dgc_file_exists(new_filename)) 
    mkdir(new_filename, 0755);

  n = scandir(dirname, &namelist, is_img, NULL);
  i = 0;
  if(n < 0)
    perror("scandir");
  else {
    while(n--) {
      parse_fields(namelist[n]->d_name, field, extension, &num_fields);
      if(num_fields != 6)
	dgc_die("Error: filename %s has %d fields.\n", namelist[n]->d_name,
		num_fields);
      if(strcmp(field[0], "lmap") != 0)
	dgc_die("Error: filename %s has wrong first field %s.\n", 
		namelist[n]->d_name, field[0]);

      laser_resolution = atoi(field[2]);
      laser_x = atoi(field[4]);
      
      sprintf(new_filename, "%s/laser/%d/", imagery_dest_dir, laser_resolution);
      if(!dgc_file_exists(new_filename)) 
	mkdir(new_filename, 0755);

      sprintf(new_filename, "%s/laser/%d/%d/", imagery_dest_dir, 
	      laser_resolution, laser_x);
      if(!dgc_file_exists(new_filename)) 
	mkdir(new_filename, 0755);

      sprintf(new_filename, "%s/laser/%d/%d/%s", imagery_dest_dir, 
	      laser_resolution, laser_x, namelist[n]->d_name);

      /*      fprintf(stderr, "FIELDS : ");
      for(int j = 0; j < num_fields; j++) {
	fprintf(stderr, "\"%s\" ", field[j]);
      }
      fprintf(stderr, "extension \"%s\"\n", extension);*/

      sprintf(filename, "%s/%s", dirname, namelist[n]->d_name);
      fprintf(stderr, "\r    Image file %s            ", filename);
      
      /* actually copy the image */
      sprintf(cmd, "cp %s %s", filename, new_filename);
      if(system(cmd) == -1)
        dgc_error("Failed to run command %s", cmd);

      free(namelist[n]);
      i++;
    }
    free(namelist);
  }  
}

void process_darpa_dir(char *dirname)
{
  char filename[500], new_filename[500], cmd[1000];
  struct dirent **namelist;
  int darpa_resolution, darpa_x;
  int i, n, num_fields;
  char extension[100];
  char field[20][100];

  sprintf(new_filename, "%s/darpa/", imagery_dest_dir);
  if(!dgc_file_exists(new_filename)) 
    mkdir(new_filename, 0755);

  n = scandir(dirname, &namelist, is_img, NULL);
  i = 0;
  if(n < 0)
    perror("scandir");
  else {
    while(n--) {
      parse_fields(namelist[n]->d_name, field, extension, &num_fields);
      if(num_fields != 5)
	dgc_die("Error: filename %s has %d fields.\n", namelist[n]->d_name,
		num_fields);
      if(strcmp(field[0], "darpa") != 0)
	dgc_die("Error: filename %s has wrong first field %s.\n", 
		namelist[n]->d_name, field[0]);

      darpa_resolution = atoi(field[2]);
      darpa_x = atoi(field[4]);
      
      sprintf(new_filename, "%s/darpa/%d/", imagery_dest_dir, darpa_resolution);
      if(!dgc_file_exists(new_filename)) 
	mkdir(new_filename, 0755);

      sprintf(new_filename, "%s/darpa/%d/%d/", imagery_dest_dir, 
	      darpa_resolution, darpa_x);
      if(!dgc_file_exists(new_filename)) 
	mkdir(new_filename, 0755);

      sprintf(new_filename, "%s/darpa/%d/%d/%s", imagery_dest_dir, 
	      darpa_resolution, darpa_x, namelist[n]->d_name);

      /*      fprintf(stderr, "FIELDS : ");
      for(int j = 0; j < num_fields; j++) {
	fprintf(stderr, "\"%s\" ", field[j]);
      }
      fprintf(stderr, "extension \"%s\"\n", extension);*/

      sprintf(filename, "%s/%s", dirname, namelist[n]->d_name);
      fprintf(stderr, "\r    Image file %s            ", filename);
      
      /* actually copy the image */
      sprintf(cmd, "cp %s %s", filename, new_filename);
      if(system(cmd) == -1)
        dgc_error("Failed to run command %s", cmd);

      free(namelist[n]);
      i++;
    }
    free(namelist);
  }  
}

void process_gsat_dir(char *dirname)
{
  char cmd[1000];

  fprintf(stderr, "    Copying %s      ", dirname);
  sprintf(cmd, "cp -R %s %s", dirname, imagery_dest_dir);
  if(system(cmd) == -1)
    dgc_error("Failed to run command %s", cmd);
}

void process_subdir(char *dirname)
{
  struct dirent **namelist;
  struct stat file_stat;
  char filename[500], filename2[500];
  int i, n;

  fprintf(stderr, "Found %s\n", dirname);

  /* do USGS imagery */
  sprintf(filename, "%s/200", dirname);
  if(dgc_file_exists(filename)) {
    fprintf(stderr, "  Found %s\n", filename)
;
    /* urban area USGS */
    sprintf(filename, "%s/200/color", dirname);
    if(dgc_file_exists(filename)) {
      fprintf(stderr, "    Found %s\n", filename);

      n = scandir(filename, &namelist, NULL, NULL);
      i = 0;
      if(n < 0)
	perror("scandir");
      else {
	while(n--) {
	  if(strcmp(namelist[n]->d_name, ".") != 0 &&
	     strcmp(namelist[n]->d_name, "..") != 0) {
	    sprintf(filename2, "%s/%s", filename, namelist[n]->d_name);
	    stat(filename2, &file_stat);
	    if(S_ISDIR(file_stat.st_mode)) 
	      process_usgs_color_dir(filename2);
	  }
	  free(namelist[n]);
	  i++;
	}
	free(namelist);
      }  
      fprintf(stderr, "\n");
    }

    /* topo USGS */
    sprintf(filename, "%s/200/topo", dirname);
    if(dgc_file_exists(filename)) {
      fprintf(stderr, "    Found %s\n", filename);

      n = scandir(filename, &namelist, NULL, NULL);
      i = 0;
      if(n < 0)
	perror("scandir");
      else {
	while(n--) {
	  if(strcmp(namelist[n]->d_name, ".") != 0 &&
	     strcmp(namelist[n]->d_name, "..") != 0) {
	    sprintf(filename2, "%s/%s", filename, namelist[n]->d_name);
	    stat(filename2, &file_stat);
	    if(S_ISDIR(file_stat.st_mode)) 
	      process_usgs_topo_dir(filename2);
	  }
	  free(namelist[n]);
	  i++;
	}
	free(namelist);
      }  
      fprintf(stderr, "\n");
    }
  }

  /* laser imagery */
  sprintf(filename, "%s/laser", dirname);
  if(dgc_file_exists(filename)) {
    fprintf(stderr, "  Found %s\n", filename);
    process_laser_dir(filename);
    fprintf(stderr, "\n");
  }

  /* darpa imagery */
  sprintf(filename, "%s/darpa", dirname);
  if(dgc_file_exists(filename)) {
    fprintf(stderr, "  Found %s\n", filename);

    n = scandir(filename, &namelist, NULL, NULL);
    i = 0;
    if(n < 0)
      perror("scandir");
    else {
      while(n--) {
	if(strcmp(namelist[n]->d_name, ".") != 0 &&
	   strcmp(namelist[n]->d_name, "..") != 0) {
	  sprintf(filename2, "%s/%s", filename, namelist[n]->d_name);
	  stat(filename2, &file_stat);
	  if(S_ISDIR(file_stat.st_mode)) 
	    process_darpa_dir(filename2);
	}
	free(namelist[n]);
	i++;
      }
      free(namelist);
    }  
    fprintf(stderr, "\n");
  }

  /* google imagery */
  sprintf(filename, "%s/gsat", dirname);
  if(dgc_file_exists(filename)) {
    fprintf(stderr, "  Found %s\n", filename);
    process_gsat_dir(filename);
    fprintf(stderr, "\n");
  }

}

int main(int argc, char **argv)
{
  struct dirent **namelist;
  struct stat file_stat;
  char filename[500];
  int i, n;

  if(argc < 3)
    dgc_die("Error: not enough arguments.\n"
	    "Usage: %s imagery-src imagery-dest\n", argv[0]);

  strcpy(imagery_src_dir, argv[1]);
  strcpy(imagery_dest_dir, argv[2]);

  /* check each imagery sub directory */
  n = scandir(imagery_src_dir, &namelist, NULL, NULL);
  i = 0;
  if(n < 0)
    perror("scandir");
  else {
    while(n--) {
      if(strcmp(namelist[n]->d_name, ".") != 0 &&
	 strcmp(namelist[n]->d_name, "..") != 0) {
	sprintf(filename, "%s/%s", imagery_src_dir, namelist[n]->d_name);
	stat(filename, &file_stat);
	if(S_ISDIR(file_stat.st_mode)) 
	  process_subdir(filename);
      }
      free(namelist[n]);
      i++;
    }
    free(namelist);
  }  


  return 0;
}
