#ifndef _GLOBAL_MACADDONS_H_
#define _GLOBAL_MACADDONS_H_

#ifdef __DARWIN__
#include <stdio.h>
#include <unistd.h>
#include <sched.h>
#include <mach/mach.h>

#define SIGCLD SIGCHLD

#ifndef OFF64T_
#define OFF64T_
typedef off_t off64_t; // it's 64bit already
typedef off64_t __off64_t;
#endif

	// standard functions are 64bit already
#define fopen64 fopen
//extern inline FILE* fopen64(const char* filename, const char* mode) {return fopen(filename, mode);}
#define fseeko64 fseek
//extern inline int fseeko64(FILE* stream, off_t offset, int whence) {return fseeko(stream, offset, whence);}
#define lseek64 lseek
//extern inline off64_t lseek64(int fildes, off64_t offset, int whence) {return lseek(fildes, offset, whence);}
#define ftello64 ftello
//extern inline off64_t ftello64(FILE* stream) {return ftello(stream);}

#define open64 open
#define O_LARGEFILE 0
#endif

int sched_setscheduler(__attribute__ ((unused)) pid_t pid, int policy, __attribute__ ((unused)) const struct sched_param* param);

void* memmem(const void* haystack, size_t haystack_len, const void* needle, size_t needle_len);

#endif
