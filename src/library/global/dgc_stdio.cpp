#include <roadrunner.h>
#include <dgc_stdio.h>

dgc_FILE *dgc_fopen(const char *filename, const char *mode)
{
  dgc_FILE *fp;
  
  /* allocate a new file pointer */
  fp = (dgc_FILE *)calloc(1, sizeof(dgc_FILE));
  dgc_test_alloc(fp);

  /* copy the filename & mode */
  fp->filename = (char *)calloc(strlen(filename) + 1, 1);
  dgc_test_alloc(fp->filename);
  strcpy(fp->filename, filename);

  fp->mode = (char *)calloc(strlen(mode) + 1, 1);
  dgc_test_alloc(fp->mode);
  strcpy(fp->mode, mode);

  /* look at filename extension to determine if file is compressed */
  if(strcmp(filename + strlen(filename) - 3, ".gz") == 0)
    fp->compressed = 1;
  else
    fp->compressed = 0;

  if(!fp->compressed) {
    fp->fp = (FILE*)fopen64(filename, mode);
    if(fp->fp == NULL) {
      free(fp);
      return NULL;
    }
  }
  else {
    fp->fp = (FILE*)fopen64(filename, mode);
    if(fp->fp == NULL) {
      free(fp);
      return NULL;
    }
    fp->comp_fp = (void**)gzdopen(fileno(fp->fp), mode);
    if(fp->comp_fp == NULL) {
      fclose(fp->fp);
      free(fp);
      return NULL;
    }
  }
  return fp;
}

int dgc_fgetc(dgc_FILE *fp)
{
  if(!fp->compressed)
    return fgetc(fp->fp);
  else
    return gzgetc(fp->comp_fp);
}

int dgc_feof(dgc_FILE *fp)
{
  if(!fp->compressed)
    return feof(fp->fp);
  else
    return gzeof(fp->comp_fp);
}

int dgc_fseek(dgc_FILE *fp, off64_t offset, int whence)
{
  int err;

  if(!fp->compressed)
    return fseeko64(fp->fp, offset, whence);
  else {
    err = gzseek(fp->comp_fp, offset, whence);

    if(err < 0 && whence == SEEK_SET && offset == 0) {
      gzclose(fp->comp_fp);

      fp->fp = (FILE*)fopen64(fp->filename, fp->mode);
      if(fp->fp == NULL) {
	free(fp);
	return -1;
      }
      fp->comp_fp = (void**)gzdopen(fileno(fp->fp), fp->mode);
      if(fp->comp_fp == NULL) {
	fclose(fp->fp);
	free(fp);
	return -1;
      }

      return 0;
    }
    else
      return err;
  }
}

off64_t dgc_ftell(dgc_FILE *fp)
{
  if(!fp->compressed)
    return ftello64(fp->fp);
  else
    return gztell(fp->comp_fp);
}

int dgc_fclose(dgc_FILE *fp)
{
  if(!fp->compressed)
    return fclose(fp->fp);
  else
    return gzclose(fp->comp_fp);
}

size_t dgc_fread(void *ptr, size_t size, size_t nmemb, dgc_FILE *fp)
{
  if(!fp->compressed)
    return fread(ptr, size, nmemb, fp->fp);
  else
    return gzread(fp->comp_fp, ptr, size * nmemb) / size;
}

size_t dgc_fwrite(const void *ptr, size_t size, size_t nmemb, dgc_FILE *fp)
{
  if(!fp->compressed)
    return fwrite(ptr, size, nmemb, fp->fp);
  else
    return gzwrite(fp->comp_fp, (void *)ptr, size * nmemb) / size;
}

char *dgc_fgets(char *s, int size, dgc_FILE *fp)
{
  if(!fp->compressed)
    return fgets(s, size, fp->fp);
  else
    return gzgets(fp->comp_fp, s, size);
}

int dgc_fputc(int c, dgc_FILE *fp)
{
  if(!fp->compressed)
    return fputc(c, fp->fp);
  else
    return gzputc(fp->comp_fp, c);
}

void dgc_fprintf(dgc_FILE *fp, const char *fmt, ...)
{
  /* Guess we need no more than 100 bytes. */
  int n, size = 100;
  char *p;
  va_list ap;

  if((p = (char *)malloc(size)) == NULL)
    return;
  while(1) {
    /* Try to print in the allocated space. */
    va_start(ap, fmt);
    n = vsnprintf(p, size, fmt, ap);
    va_end(ap);
    /* If that worked, return the string. */
    if(n > -1 && n < size) {
      dgc_fwrite(p, strlen(p), 1, fp);
      free(p);
      return;
    }
    /* Else try again with more space. */
    if(n > -1)    /* glibc 2.1 */
      size = n + 1; /* precisely what is needed */
    else           /* glibc 2.0 */
      size *= 2;  /* twice the old size */
    if((p = (char *)realloc(p, size)) == NULL)
      return;
  }
}

int dgc_fflush(dgc_FILE *fp)
{
  if(!fp->compressed)
    return fflush(fp->fp);
  else
    return gzflush(fp->comp_fp, Z_SYNC_FLUSH);
}
