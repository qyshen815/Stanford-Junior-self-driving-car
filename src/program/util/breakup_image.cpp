#include <roadrunner.h>
#include <image.h>
#include <libgeotiff/xtiffio.h>
#include <libgeotiff/geotiffio.h>
 
#include <libgeotiff/geotiff.h>
#include <libgeotiff/geo_normalize.h>
#include <libgeotiff/geovalues.h>

#include <lltransform.h>
#include <proj_api.h>

#define      DO_DATUM_SHIFT       0
#define      AT_SWRI              0
#define      MIN_RES_MULT         1

enum {
  VERSION = 0,
  MAJOR,
  MINOR
};

void create_dir(char *dirname)
{
  if(!dgc_file_exists(dirname)) {
    //    fprintf(stderr, "Making directory %s\n", dirname);
    mkdir(dirname, 0755);
  }
}

projPJ pj1, pj2;

void setup_datum_shift(void)
{
  pj1 = pj_init_plus("+proj=latlong +datum=NAD27");
  pj2 = pj_init_plus("+proj=latlong +datum=NAD83");
}

void nad_datum_shift(double *latitude, double *longitude)
{
  double x, y, z = 0;
  
  x = dgc_d2r(*longitude);
  y = dgc_d2r(*latitude);
  pj_transform(pj1, pj2, 1, 1, &x, &y, &z);
  *latitude = dgc_r2d(y);
  *longitude = dgc_r2d(x);
}

int main(int argc, char **argv)
{
  TIFF *tif=(TIFF*)0;  /* TIFF-level descriptor */
  GTIF *gtif=(GTIF*)0; /* GeoKey-level descriptor */
  int versions[3];
  int cit_length;
  geocode_t model;    /* all key-codes are of this type */
  char *citation;
  int size;
  tagtype_t type;
  GTIFDefn defn;

  int width, height;
  double left, right, bottom, top;
  double resolution = dgc_surveyor_feet2meters(1.0) / 2.0;

  dgc_image_t* image_out;
  dgc_image_t* image;

  int xi, yi, x1i, y1i, x2i, y2i;
  int x1p, y1p;
  int xp, yp, xp2, yp2;
  char dirname[200], filename[200];

  int mult, count = 0, total_count;

  if(argc < 2)
    dgc_die("Error: not enough arguments.\n"
	    "Usage: %s filename\n", argv[0]);
  
  /* Open TIFF descriptor to read GeoTIFF tags */
  tif = XTIFFOpen(argv[1], "r");  
  if(!tif) 
    goto failure;

  /* Open GTIF Key parser; keys will be read at this time. */
  gtif = GTIFNew(tif);
  if(!gtif)
    goto failure;
  
  /* Get the GeoTIFF directory info */
  GTIFDirectoryInfo(gtif,versions,0);
  if(versions[MAJOR] > 1) {
    printf("this file is too new for me\n"); 
    goto failure;
  }
  if(!GTIFKeyGet(gtif, GTModelTypeGeoKey, &model, 0, 1)) {
    printf("Yikes! no Model Type\n");
    goto failure;
  }
  
  /* ASCII keys are variable-length; compute size */
  cit_length = GTIFKeyInfo(gtif, GTCitationGeoKey, &size, &type);
  if(cit_length > 0) {
    citation = (char *)malloc(size * cit_length);
    if(!citation) 
      goto failure;
    GTIFKeyGet(gtif, GTCitationGeoKey, citation, 0, cit_length);
  }

  TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &width);
  TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &height);

  left = 0; bottom = height;
  if(!GTIFImageToPCS(gtif, &left, &bottom))
    dgc_die("Error: could not transform coordinate to PCS\n");
  
  right = width; top = 0;
  if(!GTIFImageToPCS(gtif, &right, &top))
    dgc_die("Error: could not transform coordinate to PCS\n");

  /* make sure units are in meters */
  if(!GTIFGetDefn(gtif, &defn))
    dgc_die("Error: could not get image definition.\n");
  if(defn.Model != ModelTypeProjected)
    dgc_die("Error: geotiff image is not in projection coordinates\n");
  left *= defn.UOMLengthInMeters;
  top *= defn.UOMLengthInMeters;
  right *= defn.UOMLengthInMeters;
  bottom *= defn.UOMLengthInMeters;

  setup_datum_shift();

  if(AT_SWRI)
    spcsInit(4204, 1, 0);

  left = 0; top = 0;
  if(!GTIFImageToPCS(gtif, &left, &top))
    dgc_die("Error: could not transform coordinate to PCS\n");
  GTIFProj4ToLatLong(&defn, 1, &left, &top);
  if(DO_DATUM_SHIFT)
    nad_datum_shift(&top, &left);

  /* do we need this? */
  double top2, left2;
  fprintf(stderr, "input lat lon %f %f\n", top, left);
  latLongToSpcs(left, top, &top2, &left2);
  fprintf(stderr, "output  SPCS x y %f %f\n", left2, top2);
  top = top2;
  left = left2;

  right = width; bottom = height;
  if(!GTIFImageToPCS(gtif, &right, &bottom))
    dgc_die("Error: could not transform coordinate to PCS\n");
  GTIFProj4ToLatLong(&defn, 1, &right, &bottom);
  if(DO_DATUM_SHIFT)
    nad_datum_shift(&bottom, &right);

  /* do we need this? */
  double bottom2, right2;
  latLongToSpcs(right, bottom, &bottom2, &right2);
  bottom = bottom2;
  right = right2;

  /* get rid of the key parser */
  GTIFFree(gtif);
  
  /* close the TIFF file descriptor */
  XTIFFClose(tif);

  fprintf(stderr, "pos after %f %f -> %f %f  res %f\n", left, top, right, bottom,
	  resolution * MIN_RES_MULT);

  image_out = dgc_image_initialize(256, 256);

  fprintf(stderr, "Reading image %s... ", argv[1]);
  image = dgc_image_read(argv[1]);
  fprintf(stderr, "done.\n");

  create_dir("darpa");
  
  for(mult = 1; mult <= 512; mult *= 2) {
    x1i = (int)floor(left / (resolution * MIN_RES_MULT * mult) / 256.0);
    y1i = (int)floor(bottom / (resolution * MIN_RES_MULT * mult) / 256.0);
    x2i = (int)floor(right / (resolution * MIN_RES_MULT * mult) / 256.0);
    y2i = (int)floor(top / (resolution * MIN_RES_MULT * mult) / 256.0);

    total_count = (x2i - x1i + 1) * (y2i - y1i + 1);
    
    count = 0;
    for(xi = x1i; xi <= x2i; xi++) {
      sprintf(dirname, "darpa/%d", xi);
      create_dir(dirname);
      
      for(yi = y1i; yi <= y2i; yi++) {
	x1p = (int)rint(xi * 256 * mult - left / (resolution * MIN_RES_MULT));
	y1p = (int)rint(yi * 256 * mult - bottom / (resolution * MIN_RES_MULT));
	
	for(xp = x1p; xp < x1p + 256 * mult; xp += mult)
	  for(yp = y1p; yp < y1p + 256 * mult; yp += mult) {
	    xp2 = (xp - x1p) / mult;
	    yp2 = (yp - y1p) / mult;
	    if(xp < 0 || xp >= width || yp < 0 || yp >= height) {
	      image_out->pix[(yp2 * 256 + xp2) * 3] = 0;
	      image_out->pix[(yp2 * 256 + xp2) * 3 + 1] = 0;
	      image_out->pix[(yp2 * 256 + xp2) * 3 + 2] = 0;
	    }
	    else {
	      memcpy(image_out->pix + (yp2 * 256 + xp2) * 3, 
		     image->pix + ((height - yp - 1) * width + xp) * 3, 3);
	    }
	  }
	
	sprintf(filename, "darpa/%d/darpa-%d-%d-%d-%d.jpg", xi, 405,
		mult * MIN_RES_MULT, xi, yi);
	
	count++;
	fprintf(stderr, 
		"\rMult %d : Writing image %d of %d (%.2f%%) : %s      ", mult,
		count, total_count, count / (double)total_count * 100, 
		filename);
	
	dgc_image_write_raw(image_out->pix, image_out->width, 
			    image_out->height, filename);
      }
    }

    
  }


  exit (0);
 failure:
  exit (-1);
  return 0;
}

