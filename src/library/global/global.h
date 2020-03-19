/********************************************************
 This source code is part of the Carnegie Mellon Robot
 Navigation Toolkit (CARMEN).

 CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 and Jared Glover
 All rights reserved.

 Redistribution and use in source and binary forms, with
 or without modification, are permitted provided that the
 following conditions are met:

 * Redistributions of source code must retain the above
 copyright notice, this list of conditions and the
 following disclaimer.
 * Redistributions in binary form must reproduce the above
 copyright notice, this list of conditions and the
 following disclaimer in the documentation and/or other
 materials provided with the distribution.
 * The names of the contributors may not be used to endorse
 or promote products derived from this software
 without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 DAMAGE.
 ********************************************************/

#ifndef VLR_GLOBAL_H_
#define VLR_GLOBAL_H_

#include <roadrunner.h>
#include <dgc_stdio.h>
#include <scaledTime.h>
#include <pose.h>

#define      DGC_MAJOR_VERSION          1
#define      DGC_MINOR_VERSION          2
#define      DGC_REVISION               2

#ifndef      TRUE
#define      TRUE                       1
#endif
#ifndef      FALSE
#define      FALSE                      0
#endif

#define      dgc_red_code            "[31;1m"
#define      dgc_blue_code           "[34;1m"
#define      dgc_normal_code         "[0m"

typedef void (*dgc_usage_func)(char *fmt, ...);

extern int dgc_carp_verbose;

#define DGC_PF __PRETTY_FUNCTION__

extern "C" {
float strtof(const char *nptr, char **endptr);
int strcasecmp(const char *s1, const char *s2);
int strncasecmp(const char *s1, const char *s2, size_t n);
}

#ifndef va_copy
#define va_copy __va_copy
#endif

void dgc_fatal_error(char *fmt, ...) __attribute__ ((format (printf, 1, 2)));
void dgc_error(char *fmt, ...) __attribute__ ((format (printf, 1, 2)));
void dgc_warning(char *fmt, ...) __attribute__ ((format (printf, 1, 2)));
void dgc_info(char *fmt, ...) __attribute__ ((format (printf, 1, 2)));

void dgc_fatal_ferror(const char *fname, char *fmt, ...) __attribute__ ((format (printf, 2, 3)));
void dgc_ferror(const char *fname, char *fmt, ...) __attribute__ ((format (printf, 2, 3)));
void dgc_fwarning(const char *fname, char *fmt, ...) __attribute__ ((format (printf, 2, 3)));
void dgc_finfo(const char *fname, char *fmt, ...) __attribute__ ((format (printf, 2, 3)));

void dgc_fprintf_blue(FILE *fp, char *fmt, ...) __attribute__ ((format (printf, 2, 3)));
void dgc_fprintf_red(FILE *fp, char *fmt, ...) __attribute__ ((format (printf, 2, 3)));
void dgc_fprintf_green(FILE *fp, char *fmt, ...) __attribute__ ((format (printf, 2, 3)));

void dgc_verbose(const char *fmt, ...) __attribute__ ((format (printf, 1, 2)));
void dgc_die(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));

void dgc_carp_set_verbose(int verbosity);
int dgc_carp_get_verbose(void);
void dgc_carp_set_output(FILE *output);
char *dgc_extract_filename(char *path);
char *dgc_hostname(void);
int dgc_sign(double num);
void dgc_rect_to_polar(double x, double y, double *r, double *theta);

unsigned int dgc_randomize(int *argc, char ***argv);
unsigned int dgc_generate_random_seed();
void dgc_set_random_seed(unsigned int seed);
int dgc_int_random(int max);
double dgc_uniform_random(double min, double max);
double dgc_gaussian_random(double mean, double std);

int dgc_file_exists(char *filename);
char *dgc_file_extension(char *filename);
int dgc_strcasecmp(const char *s1, const char *s2);
int dgc_strncasecmp(const char *s1, const char *s2, size_t n);
char *dgc_new_string(const char *fmt, ...);
char *dgc_new_stringv(const char *fmt, va_list ap);
void dgc_print_version(void);

typedef enum {
    DGC_PRIORITY_MAX, // Shouldn't be necessary except for central
    DGC_PRIORITY_LOWLATENCY, // For CPU-light sensor apps, like laser, applanix, can, riegl
    DGC_PRIORITY_REALTIME, // Use real-time scheduling
    DGC_PRIORITY_DEFAULT, // Above average priority (nice-level -5) but no real-time
    DGC_PRIORITY_NORMAL, // Unchanged priority (nice-level 0)
    DGC_PRIORITY_LOW,
// For batch processes
} dgc_priority_t;

int dgc_set_priority(dgc_priority_t priority, int allow_swapping_out);

typedef struct {
    int argc;
    char **argv;
} param_struct_t, *param_struct_p;

typedef struct {

    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;

} dgc_pose_t, *dgc_pose_p;

#define dgc_time_code(code, str) { double time_code_t1, time_code_t2; time_code_t1 = dgc_get_time(); code; time_code_t2 = dgc_get_time(); fprintf(stderr, "%-20s :%7.2f ms.\n", str, (time_code_t2 - time_code_t1) * 1000.0); }

template <class T>
inline void dgc_test_alloc(T* X) {
    if (!X) {
        dgc_die("Out of memory in %s, (%s, line %d).\n", __FUNCTION__, __FILE__, __LINE__);
    }
}

/*
 The function is global.c is a backup function as the function in library/timesync/timesync.c does a better job
 */

extern inline int dgc_round(double X) {
    if (X >= 0) return (int) (X + 0.5);
    else return (int) (X - 0.5);
}

inline double dgc_clamp(double X, double Y, double Z) {
    if (Y < X) return X;
    else if (Y > Z) return Z;
    return Y;
}

inline int dgc_clamp_int(int X, int Y, int Z) {
    if (Y < X) return X;
    else if (Y > Z) return Z;
    return Y;
}

inline int dgc_trunc(double X) {
    return (int) (X);
}

double dgc_normalize_theta(double theta);

inline double dgc_hypot3(double a, double b, double c) {
    return sqrt(a * a + b * b + c * c);
}

inline float dgc_hypot3f(float a, float b, float c) {
    return sqrtf(a * a + b * b + c * c);
}

inline double dgc_r2d(double theta) {
    return (theta * 180.0 / M_PI);
}

inline double dgc_d2r(double theta) {
    return (theta * M_PI / 180.0);
}

inline float dgc_r2df(float theta) {
    return (theta * 180.0 / M_PI);
}

inline float dgc_d2rf(float theta) {
    return (theta * M_PI / 180.0);
}

inline double dgc_mph2ms(double mph) {
    return (mph * 0.44704);
}

inline double dgc_ms2mph(double ms) {
    return (ms * 2.23693629);
}

inline double dgc_kph2ms(double kph) {
    return (kph * 0.277777778);
}

inline double dgc_ms2kph(double ms) {
    return (ms * 3.6);
}

inline double dgc_meters2feet(double meters) {
    return (meters * 3.2808399);
}

inline double dgc_feet2meters(double feet) {
    return (feet * 0.3048);
}

inline double dgc_meters2miles(double meters) {
    return (meters / 1609.344);
}

inline double dgc_miles2meters(double miles) {
    return (miles * 1609.344);
}

inline double dgc_mph2kph(double mph) {
    return (mph * 1.609344);
}

inline double dgc_kph2mph(double kph) {
    return (kph * 0.621371192);
}

inline double dgc_surveyor_feet2meters(double x) {
    return x * 0.304800609601219;
}

inline double dgc_meters2surveyor_feet(double x) {
    return x / 0.304800609601219;
}

inline double dgc_fmin(double val1, double val2) {
    if (val2 < val1) return val2;
    return val1;
}

inline double dgc_fmax(double val1, double val2) {
    if (val2 > val1) return val2;
    return val1;
}

inline double dgc_square(double val) {
    return (val * val);
}

inline int my_isblank(char c) {
    if (c == ' ' || c == '\t') return 1;
    else return 0;
}

char* dgc_next_word(char *str);
char *dgc_next_n_words(char *str, int n);

void dgc_point_to_line_distance(double x, double y, double x1, double y1, double x2, double y2, double *parallel_dist,
        double *perp_dist, double *x_match, double *y_match);

void dgc_point_to_segment_distance(double x, double y, double x1, double y1, double x2, double y2, double *perp_dist,
        double *parallel_dist, double *x_match, double *y_match);

void dgc_point_to_segment_distance2(double x, double y, double yaw, double x1, double y1, double x2, double y2,
        double xfar, double yfar, double *perp_dist, double *parallel_dist, double *x_match, double *y_match);

double dgc_average_angle(double theta1, double theta2);

off64_t dgc_file_size(char *filename);

char *dgc_run_program(char *command_line, char *input, double timeout);

int dgc_complete_filename(char *filename, char *extension, char **new_filename);
char* dgc_expand_filename(char *filename);
char* dgc_unique_filename(char *name);
char dgc_ascii_rotor();

double dgc_get_time();
float dgc_avg_time(float t, int cnt);
float dgc_avg_times(float t, int cnt, int num);

char* find_matching_logfile(char *ipc_filename, char *extension, double max_dt);
char* dgc_timestamped_filename(char *basename, char *extension);

#endif
