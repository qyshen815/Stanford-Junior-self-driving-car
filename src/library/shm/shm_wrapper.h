#ifndef DGC_SHM_WRAPPER_H
#define DGC_SHM_WRAPPER_H

class dgc_shm_t {
 public:
  dgc_shm_t();
  ~dgc_shm_t();
  
  int init_server(int key, int info_size, int data_size);
  int init_client(int key, int info_size, int data_size);

  int clear(void);
  bool bytes_waiting(void);

  int write(unsigned int len, unsigned char *data);
  int start_partial_write(void);
  int partial_write(unsigned int len, unsigned char *data);
  int finish_partial_write(void);
  
  int nread(int maxlen, unsigned char *s);
  int read(unsigned char *s);
  int current_nread(int maxlen, unsigned char *s);
  int current_read(unsigned char *s);

  int start_partial_read(void);
  int start_partial_current_read(void);
  int partial_nread(unsigned int maxlen, unsigned char *s);
  int partial_read(unsigned char *s);
  int finish_partial_read(void);
  int abort_partial_read(void);

  int add_info(unsigned int len, unsigned char *data);
  int get_info(unsigned char *s);

  long long get_max(void);
  int set_max(long long max);

 private:
  enum shm_interface_type { SERVER, CLIENT };

  bool initialized;

  shm_interface_type int_type;
  unsigned int   session_id;

  int            shmid;     /* shm id - needed to delete the segment */
  unsigned char *mem;
  unsigned int   i_start;   /* start of info buffer */
  unsigned int   i_size;    /* size of the (static) info buffer */
  unsigned int   i_off;

  unsigned int   d_start;   /* start of data buffer */
  unsigned int   d_size;    /* size of the (ringbuffer) data buffer */
  unsigned int   w_off;
  unsigned int   r_off;

  unsigned int   t_len;     /* total length */
  unsigned char  use_checksum;

  unsigned char partial_chksum;

  bool partial_write_in_progress;
  unsigned int partial_bytes_written;

  bool partial_read_in_progress;
  unsigned int partial_bytes_read;
  unsigned int partial_bytes_total;
  
  int init(int data_key, shm_interface_type mode, int info_size, int data_size);
  int dwrite_addr(int num);
  void dwrite(unsigned char *data, int len, int offset);
  int dread_addr(int num);
  void dread(unsigned char *data, int len, int offset);
  int nread_internal(int maxlen, unsigned char *s, int current);
  int iwrite_addr(int num);
  void iwrite(unsigned char *data, int len, int offset);
  void test_session_id(void);
  int start_partial_read_internal(int current);
};

#endif
