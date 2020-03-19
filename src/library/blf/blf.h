#ifndef DGC_BLF_H
#define DGC_BLF_H

#include <async_writer.h>
#include <stdio.h>

#define          BLF_OK                               0
#define          BLF_INVALID                         -1
#define          BLF_EOF                             -2
#define          BLF_TIMESTAMP                       -3
#define          BLF_CHKSUM                          -4

#define          BLF_START_BYTE                    0x27

typedef struct {
  unsigned short       id;
  double               timestamp;
  unsigned int         len, max_len;
  unsigned char       *data;
} blf_pkt_t, *blf_pkt_p;

class blf_t {
 public:
  blf_t();
  blf_t(bool async_write_mode);

  ~blf_t();
  
  int open(char *filename, char *mode);

  int close(void);

  /* reading */

  int read_data(unsigned short *id, double *timestamp,
		unsigned char **data, unsigned int *len, 
		unsigned int *max_size);

  int read_pkt(blf_pkt_t *pkt);

  int start_partial_read(unsigned short *id, double *timestamp,
			 unsigned int *len);

  int partial_read(unsigned char *data, unsigned int len);

  int finish_partial_read(void);

  int abort_partial_read(void);

  int reset_partial_read(void);

  int seek(long long offset, int whence);

  int seek_timestamp_nearest(double t);
  
  int seek_timestamp(double t);

  long long tell(void);

  int rewind(void);

  /* writing */

  int write_data(unsigned int len, unsigned char *data,
		 double timestamp, unsigned short id);

  int write_pkt(blf_pkt_t *pkt);

  int start_partial_write(unsigned int len, double timestamp, 
			  unsigned short id);

  int partial_write(unsigned int len, unsigned char *data);

  int finish_partial_write(void);

  int std_write_data(unsigned int len, unsigned char *data,
		     double timestamp, unsigned short id);

  int std_write_pkt(blf_pkt_t *pkt);

 private:
  enum blf_status_t     { UNINITIALIZED, READ, WRITE };
  bool                  async_mode;
  dgc::AsyncWriter      async_writer;

  FILE                * fp;

  blf_status_t          status;
  unsigned int          partial_bytes_left;
  unsigned int          partial_bytes_read;
  bool                  partial_read_in_progress;
  unsigned int          partial_bytes_written;
  bool                  partial_write_in_progress;
  unsigned char         partial_chksum;
  
  bool                  found_first_timestamp;
  double                first_timestamp;
};

/* BLF Index files */

typedef struct {
  int                   id;
  __off64_t             offset;
  double                timestamp;
} blf_index_entry_t, *blf_index_entry_p;

typedef struct {
  int                   num_blocks, max_blocks;
  blf_index_entry_p     block;
} blf_index_t, *blf_index_p;

blf_index_p blf_index_load(char *filename);

#endif
