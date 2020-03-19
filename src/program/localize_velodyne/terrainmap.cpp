#include <roadrunner.h>
#include <grid.h>
#include <lltransform.h>
#include "terrainmap.h"
#include <image.h>

using namespace vlr;

static unsigned char *rgb = NULL;

int terrain_tile::load(char *filename) {
  dgc_FILE *fp;
  int i, temp_cols, temp_rows;

  fp = dgc_fopen(filename, "r");
  if(fp == NULL)
    return -1;
  
  strcpy(this->filename, filename);
  
  dgc_fread(&temp_cols, sizeof(int), 1, fp);
  dgc_fread(&temp_rows, sizeof(int), 1, fp);
  if(temp_cols != cols || temp_rows != rows)
    dgc_die("Error: tile is not the right size\n");

  dgc_fread(&utm_x0, sizeof(double), 1, fp);
  dgc_fread(&utm_y0, sizeof(double), 1, fp);
  dgc_fread(&utmzone, 3, 1, fp);
  utmzone[4] = '\0';
  dgc_fread(&resolution, sizeof(double), 1, fp);

  for(i = 0; i < cols; i++)
    dgc_fread(cell[i], rows * sizeof(terrain_tile_cell), 1, fp);
  
  dgc_fclose(fp);

  return 0;
}

int vision_tile::load(char *filename) {
  dgc_FILE *fp;
  int i, temp_cols, temp_rows;

  fp = dgc_fopen(filename, "r");
  if(fp == NULL)
    return -1;
  
  strcpy(this->filename, filename);
  
  dgc_fread(&temp_cols, sizeof(int), 1, fp);
  dgc_fread(&temp_rows, sizeof(int), 1, fp);
  if(temp_cols != cols || temp_rows != rows)
    dgc_die("Error: vision tile is not the right size\n");

  dgc_fread(&utm_x0, sizeof(double), 1, fp);
  dgc_fread(&utm_y0, sizeof(double), 1, fp);
  dgc_fread(&utmzone, 3, 1, fp);
  utmzone[4] = '\0';
  dgc_fread(&resolution, sizeof(double), 1, fp);

  for(i = 0; i < cols; i++)
    dgc_fread(cell[i], rows * sizeof(vision_tile_cell), 1, fp);
  
  dgc_fclose(fp);

  return 0;
}

void terrain_tile::do_average(void)
{
  int x, y;
  
  min_z = 1e9;
  for(x = 0; x < cols; x++)
    for(y = 0; y < rows; y++) {
      if(cell[x][y].z_count > 0) {
	cell[x][y].z /= (double)cell[x][y].z_count;
	if(cell[x][y].z < min_z)
	  min_z = cell[x][y].z;
      }
      if(cell[x][y].i_count > 0)
	cell[x][y].intensity /= (double)cell[x][y].i_count;
    }
}

int vision_tile::save(char *filename) {
  dgc_FILE *fp;
  int i, j;

  fp = dgc_fopen(filename, "w");
  if(fp == NULL)
    return -1;
  
  dgc_fwrite(&cols, sizeof(int), 1, fp);
  dgc_fwrite(&rows, sizeof(int), 1, fp);

  dgc_fwrite(&utm_x0, sizeof(double), 1, fp);
  dgc_fwrite(&utm_y0, sizeof(double), 1, fp);
  dgc_fwrite(&utmzone, 3, 1, fp);
  dgc_fwrite(&resolution, sizeof(double), 1, fp);

  for(i = 0; i < cols; i++)
    dgc_fwrite(cell[i], rows * sizeof(vision_tile_cell), 1, fp);
  dgc_fclose(fp);

  int numPixels = TERRAIN_TILE_SIZE * TERRAIN_TILE_SIZE;
  if(!rgb)
  	rgb = (unsigned char *)calloc(numPixels*3, 1);

  for(i = 0; i < numPixels * 3; i++)
	rgb[i] = 0;
  for(i = 0; i < cols; i++) {
	for(j = 0; j < rows; j++) {
		int intensity = (int) (255 * cell[i][j].blur);
		if(intensity > 255) intensity = 255;
		int lane = (int) (1000 * cell[i][j].line_lat);
		if(lane > 255) lane = 255;
		rgb[3 * (j * TERRAIN_TILE_SIZE + i)] = (unsigned char) intensity;
		rgb[3 * (j * TERRAIN_TILE_SIZE + i) + 1] = (unsigned char) (lane);
		//rgb[3 * (j * TERRAIN_TILE_SIZE + i) + 2] = (unsigned char) (.2 * intensity);
	}
  }
  
  char imagename[100];
  strncpy(imagename, filename, 29);
  strcpy(imagename + 29, ".png");
  printf("NEW FILE NAME: %s\n", imagename);
  dgc_image_write_raw(rgb, TERRAIN_TILE_SIZE, TERRAIN_TILE_SIZE, imagename);
  return 0;
}

int terrain_tile::save(char *filename) {
  dgc_FILE *fp;
  int i, j;

  fp = dgc_fopen(filename, "w");
  if(fp == NULL)
    return -1;
  
  dgc_fwrite(&cols, sizeof(int), 1, fp);
  dgc_fwrite(&rows, sizeof(int), 1, fp);

  dgc_fwrite(&utm_x0, sizeof(double), 1, fp);
  dgc_fwrite(&utm_y0, sizeof(double), 1, fp);
  dgc_fwrite(&utmzone, 3, 1, fp);
  dgc_fwrite(&resolution, sizeof(double), 1, fp);

  for(i = 0; i < cols; i++)
    dgc_fwrite(cell[i], rows * sizeof(terrain_tile_cell), 1, fp);
  dgc_fclose(fp);

  int numPixels = TERRAIN_TILE_SIZE * TERRAIN_TILE_SIZE;
  if(!rgb)
  	rgb = (unsigned char *)calloc(numPixels*3, 1);

  for(i = 0; i < numPixels * 3; i++)
	rgb[i] = 0;
  for(i = 0; i < cols; i++) {
	for(j = 0; j < rows; j++) {
		int count = cell[i][j].i_count;
		if(count > 0) {
			int intensity = (int) (cell[i][j].intensity / count);
			intensity *= 1.0;
			if(intensity > 255) intensity = 255;
			if(intensity < 0) intensity = 0;
			rgb[3 * (j * TERRAIN_TILE_SIZE + i)] = (unsigned char) intensity;
			rgb[3 * (j * TERRAIN_TILE_SIZE + i) + 1] = (unsigned char) (.5 * intensity);
			rgb[3 * (j * TERRAIN_TILE_SIZE + i) + 2] = (unsigned char) (.2 * intensity);
		}
		/* int z_ness = (int) (30 * pow((cell[i][j].z / cell[i][j].z_count), 2));
		if(cell[i][j].z_count > 1)
			z_ness = (int) (100 * (cell[i][j].current_highest - cell[i][j].z));
		else
			z_ness = 0;
		z_ness = (int) (2 * cell[i][j].stdev);
		if(z_ness > 255) z_ness = 255;
		//rgb[3 * (j * TERRAIN_TILE_SIZE + i) + 1] = (unsigned char) z_ness;
		z_ness = (int) (1.0 * cell[i][j].z_response);
		if(z_ness > 255) z_ness = 255;
		//rgb[3 * (j * TERRAIN_TILE_SIZE + i) + 2] = (unsigned char) z_ness;

		z_ness = .01 * pow(cell[i][j].intensity / cell[i][j].i_count, 2) / cell[i][j].stdev;
		if(z_ness > 255) z_ness = 255;
		//rgb[3 * (j * TERRAIN_TILE_SIZE + i) + 2] = (unsigned char) z_ness;

		if(cell[i][j].z_count > 0) {
			float height = cell[i][j].current_highest - cell[i][j].z;
			if(height > 1) height = 1;
			rgb[3 * (j * TERRAIN_TILE_SIZE + i) + 1] = 0 + 1 * ((unsigned char) (255 * height));

			int z_ness = (int) (10 * pow(cell[i][j].z_response / (.000001 + cell[i][j].current_highest - cell[i][j].z), -1));
			z_ness = 10 * cell[i][j].z_response;
			z_ness = 2 * (cell[i][j].z_response / (height + .05));
			if(z_ness > 255) z_ness = 255;

			rgb[3 * (j * TERRAIN_TILE_SIZE + i) + 2] = 0 + 1 * ((unsigned char) z_ness);
		}
    */
	}
  }
  
  char imagename[100];
  strncpy(imagename, filename, 29);
  strcpy(imagename + 29, ".png");
  printf("NEW FILE NAME: %s\n", imagename);
  dgc_image_write_raw(rgb, TERRAIN_TILE_SIZE, TERRAIN_TILE_SIZE, imagename);
  return 0;
}

vision_tile::vision_tile(int rows, int cols) {
  int x, y;

  this->rows = rows;
  this->cols = cols;
  cell = (vision_tile_cell **)calloc(cols, sizeof(vision_tile_cell *));
  dgc_test_alloc(cell);
  for(x = 0; x < cols; x++) {
    cell[x] = (vision_tile_cell *)calloc(rows, sizeof(vision_tile_cell));
    dgc_test_alloc(cell);
    for(y = 0; y < rows; y++) {
      cell[x][y].curb = 0;
      cell[x][y].blur = 0;
    }
  }
}

terrain_tile::terrain_tile(int rows, int cols) {
  int x, y;

  this->rows = rows;
  this->cols = cols;
  cell = (terrain_tile_cell **)calloc(cols, sizeof(terrain_tile_cell *));
  dgc_test_alloc(cell);
  for(x = 0; x < cols; x++) {
    cell[x] = (terrain_tile_cell *)calloc(rows, sizeof(terrain_tile_cell));
    dgc_test_alloc(cell);
    for(y = 0; y < rows; y++) {
      cell[x][y].z = 1e6;
      cell[x][y].current_highest = -1e6;
    }
  }
}

vision_tile::~vision_tile() {
  int x;
  for(x = 0; x < cols; x++)
    free(cell[x]);
  free(cell);
}

terrain_tile::~terrain_tile() {
  int x, y;
  
  //printf("hi!\n");
  double stdev_sum = 0.0;
  int stdev_count = 0;
  double stdev_max = 0.0;
  for(x = 0; x < cols; x++) {
    for(y = 0; y < rows; y++) {
	//stdev_sum += cell[x][y].stdev;
	if(cell[x][y].stdev > 0) {
		stdev_count++;
		stdev_sum += cell[x][y].stdev;
		if(cell[x][y].stdev > stdev_max)
			stdev_max = cell[x][y].stdev;
	}
    }
  }
  //printf("SUM: %f\n", stdev_sum);
  //printf("%d cells had an average stdev of %f, max of %f\n", stdev_count, 1.0 * stdev_sum / stdev_count, stdev_max);
  for(x = 0; x < cols; x++)
    free(cell[x]);
  free(cell);
}

