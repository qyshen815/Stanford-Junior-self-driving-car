#ifndef VLR_STDIO_H_
#define VLR_STDIO_H_

#include <stdio.h>
#include <stdlib.h>
#include <zlib.h>
#ifdef __DARWIN__
#include <sys/types.h>
#ifndef OFF64T_
#define OFF64T_
typedef off_t off64_t;
#endif
#endif

typedef struct {
  int compressed;
  char *filename, *mode;
  FILE *fp;
  gzFile *comp_fp;
} dgc_FILE;

dgc_FILE *dgc_fopen(const char *filename, const char *mode);

int dgc_fgetc(dgc_FILE *fp);

int dgc_feof(dgc_FILE *fp);

int dgc_fseek(dgc_FILE *fp, off64_t offset, int whence);

off64_t dgc_ftell(dgc_FILE *fp);

int dgc_fclose(dgc_FILE *fp);

size_t dgc_fread(void *ptr, size_t size, size_t nmemb, dgc_FILE *fp);

size_t dgc_fwrite(const void *ptr, size_t size, size_t nmemb, dgc_FILE *fp);

char *dgc_fgets(char *s, int size, dgc_FILE *fp);

int dgc_fputc(int c, dgc_FILE *fp);

void dgc_fprintf(dgc_FILE *fp, const char *fmt, ...);

int dgc_fflush(dgc_FILE *fp);
#endif
