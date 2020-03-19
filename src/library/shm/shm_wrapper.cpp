#include <roadrunner.h> 
#include <limits.h> 
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include "shm_wrapper.h"

/*

SHARED MEMORY STRUCTURE:

-- ID  --     -------- STATIC  --------       ----------- DATA -------
             INDEX           PACKET PACKET      INDEX           PACKET PACKET
00 01 02 03  00 01 02 .. 39  ...    ,,,         00 01 02 .. 39  ...    ...

             __SHM_INFO + 4                   __SHM_DATA + 4

--------   PACKET ----------
00     01 02    03 04 ... N    N+1   
SBYTE  LEN      DATA           CHK

---------  INDEX ----------
00 01 02 03    04 05 06 07
 END ADDR     CURRENT PKT ADDR

*/

#define PROC_SHM_MAX "/proc/sys/kernel/shmmax"

static const int shm_id_len        = 4;
static const int shm_index_len    = 40;
static const int shm_idtf_len      = 1;
static const int shm_chksum_len    = 1;
static const int shm_addr_len      = 4;

static const unsigned char shm_identifier = 0x0F;

dgc_shm_t::dgc_shm_t()
{
  initialized = false;
  session_id = 0;
  mem = NULL;
}

dgc_shm_t::~dgc_shm_t()
{
  shmid_ds ds;

  if(mem != NULL) {
    /* detatch from shared memory segment */
    shmdt(mem);

    /* if we are the last to detach from the segment, delete it */
    if(shmctl(shmid, IPC_STAT, &ds) >= 0) 
      if(ds.shm_nattch == 0)
	shmctl(shmid, IPC_RMID, NULL);

    mem = NULL;
  }
}

void dgc_shm_t::test_session_id(void)
{
  unsigned int off, shm_session_id;

  /* read the session_id */
  memcpy(&shm_session_id, mem, shm_id_len);

  /* read the current mem pointer of data segement */
  memcpy(&off, mem + d_start, shm_addr_len);

  if(session_id != shm_session_id) {  
    r_off = off;
    session_id = shm_session_id;
  }
}

int dgc_shm_t::init(int data_key, shm_interface_type mode, 
		    int info_size, int data_size)
{
  int m, f;
  key_t shmkey = data_key;
  unsigned int num = 0;

  if(initialized) {
    dgc_fwarning(DGC_PF, "Shm object already initialized.");
    return -1;
  }

  if(info_size < 0) {
    dgc_ferror(DGC_PF, "Size of static shm buffer must be > 0.");
    return -1;
  }
  
  if(data_size < 0) {
    dgc_ferror(DGC_PF, "Size of shm ring buffer must be > 0.");
    return -1;
  }

  int_type = mode;

  i_start = shm_id_len; /* offset is 4 bytes = sizeof(key) */
  i_size = info_size;

  d_start = shm_id_len + shm_index_len + i_size;
  d_size = data_size;

  t_len = shm_id_len + shm_index_len + i_size + shm_index_len + d_size;
  use_checksum = 1;

  partial_write_in_progress = false;
  partial_read_in_progress = false;

  if(int_type == SERVER) {
    m = IPC_CREAT | 0666;
    f = 0;
  } 
  else {
    m = 0666;
    f = SHM_RDONLY;
  }
  
  shmid = shmget(shmkey, t_len, m);
  if(shmid == -1) {
    dgc_ferror(DGC_PF, "Can't get a shared memory segment.");
    return -1;
  }
  
  mem = (unsigned char *)shmat(shmid, NULL, f);
  if(mem == (void *)-1) {
    dgc_ferror(DGC_PF, "Can't attach to shared memory segment.");
    return -1;
  }
  
  r_off = 0;
  w_off = 0;
  if(int_type == SERVER) {
    session_id = (int)remainder(dgc_get_time() * 100, INT_MAX);

    memcpy(mem, &session_id, shm_id_len);
    /* set info index to 0 */
    memcpy(mem + i_start, &num, shm_addr_len); 
    /* set data index to 0 */
    memcpy(mem + d_start, &num, shm_addr_len); 
    /* set data index to 0 */
    memcpy(mem + d_start + shm_addr_len, &num, shm_addr_len);

    dgc_info("Allocated %.2f MB of shared memory", t_len / (1024.0 * 1024.0));
  }
  else 
    test_session_id();

  initialized = true;
  return 0;
}

int dgc_shm_t::init_server(int key, int info_size, int data_size)
{
  return init(key, SERVER, info_size, data_size);
}

int dgc_shm_t::init_client(int key, int info_size, int data_size)
{
  return init(key, CLIENT, info_size, data_size);
}

static inline unsigned char
shm_chksum(unsigned char *bytes, int num)
{
  unsigned char c = 0;
  int i;

  for(i = 0; i < num; i++) 
    c += bytes[i];
  return c;
}

static inline unsigned char
shm_chksum(unsigned char *bytes, int num, unsigned char c)
{
  int i;

  for(i = 0; i < num; i++) 
    c += bytes[i];
  return c;
}

bool dgc_shm_t::bytes_waiting(void)
{
  unsigned int off;
  
  if(!initialized) {
    dgc_ferror(DGC_PF, "Shm client has not been initialized.");
    return false;
  }

  if(int_type != CLIENT) {
    dgc_ferror(DGC_PF, "Can only be called by client interfaces.");
    return false;
  }

  test_session_id();

  memcpy(&off, mem + d_start, shm_addr_len);
  if(off != (unsigned int)r_off) 
    return true;
   else
    return false;
}

int dgc_shm_t::clear(void)
{
  unsigned off;

  if(!initialized) {
    dgc_ferror(DGC_PF, "Shm client has not been initialized.");
    return 0;
  }

  if(int_type != CLIENT) {
    dgc_ferror(DGC_PF, "Can only be called by client interfaces.");
    return 0;
  }
  
  memcpy(&off, mem + d_start, shm_addr_len);
  if(r_off == off) 
    return 0;
  else {
    r_off = off;
    return 1;
  }
}

/* WRITE DATA TO SHM */

inline int dgc_shm_t::dwrite_addr(int num)
{ 
  return d_start + shm_index_len + ((w_off + num) % d_size);
}

inline void dgc_shm_t::dwrite(unsigned char *data, int len, int offset)
{ 
  int part1, part2;
  unsigned int start = w_off + offset;
  if(start < d_size && start + len > d_size) {
    /* the data has to be split in to pieces */
    part1 = d_size - w_off - offset;
    part2 = len - part1;
    memcpy(mem + dwrite_addr(offset), data, part1);
    memcpy(mem + d_start + shm_index_len, data + part1, part2);
  } 
  else 
    /* the data fits in the buffer in one piece */
    memcpy(mem + dwrite_addr(offset), data, len);
}

int dgc_shm_t::start_partial_write(void)
{
  if(!initialized) {
    dgc_ferror(DGC_PF, "Shm interface has not been initialized.");
    return -1;
  }
  if(int_type != SERVER) {
    dgc_ferror(DGC_PF, "Can only be called by server interfaces.");
    return -1;
  }

  partial_write_in_progress = true;
  partial_bytes_written = 0;
  partial_chksum = 0;

  /* write identifier byte */
  mem[dwrite_addr(0)] = shm_identifier;
  return 0;
}

int dgc_shm_t::partial_write(unsigned int len, unsigned char *data)
{
  if(!initialized) {
    dgc_ferror(DGC_PF, "Shm interface has not been initialized.");
    return -1;
  }
  if(int_type != SERVER) {
    dgc_ferror(DGC_PF, "Can only be called by server interfaces.");
    return -1;
  }
  if(!partial_write_in_progress) {
    dgc_ferror(DGC_PF, "Partial write has not been started.");
    return -1;
  }

  /* copy the data of the package to shm */
  dwrite(data, len, shm_idtf_len + shm_addr_len + partial_bytes_written);
  partial_bytes_written += len;
  partial_chksum = shm_chksum(data, len, partial_chksum);
  return 0;
}

int dgc_shm_t::finish_partial_write(void)
{
  unsigned int c_off, len, added_bytes;

  if(!initialized) {
    dgc_ferror(DGC_PF, "Shm interface has not been initialized.");
    return -1;
  }
  if(int_type != SERVER) {
    dgc_ferror(DGC_PF, "Can only be called by server interfaces.");
    return -1;
  }
  if(!partial_write_in_progress) {
    dgc_ferror(DGC_PF, "Partial write has not been started.");
    return -1;
  }

  /* copy the length of the data package to shm */
  len = partial_bytes_written;
  dwrite((unsigned char *)&len, shm_addr_len, shm_idtf_len);

  added_bytes = shm_idtf_len + shm_addr_len + partial_bytes_written;

  /* copy the chksum to shm */
  if(use_checksum) 
    mem[dwrite_addr(added_bytes)] = partial_chksum;
  added_bytes += shm_chksum_len;

  /* copy the current packet address to shm */
  c_off = w_off;
  memcpy(mem + d_start + shm_addr_len, &c_off, shm_addr_len);

  /* calculate the address of the new end of the buffer */
  w_off = (w_off + added_bytes) % d_size;

  /* copy the new address to shm */
  memcpy(mem + d_start, &w_off, shm_addr_len);

  partial_write_in_progress = false;

  return len;
}

int dgc_shm_t::write(unsigned int len, unsigned char *data)
{
  unsigned int c_off;
  int added_bytes = 0;

  if(!initialized) {
    dgc_ferror(DGC_PF, "Shm client has not been initialized.");
    return -1;
  }
  if(int_type != SERVER) {
    dgc_ferror(DGC_PF, "Can only be called by server interfaces.");
    return -1;
  }

  /* first byte is the SHM_IDENTIFIER BYTE */
  mem[dwrite_addr(0)] = shm_identifier;
  added_bytes += shm_idtf_len;

  /* copy the length of the data package to shm */
  dwrite((unsigned char *)&len, shm_addr_len, added_bytes);
  added_bytes += shm_addr_len;

  /* copy the data of the package to shm */
  dwrite(data, len, added_bytes);
  added_bytes += len;

  /* copy the chksum to shm */
  if(use_checksum) 
    mem[dwrite_addr(added_bytes)] = shm_chksum(data, len);
  added_bytes += shm_chksum_len;

  /* copy the current packet address to shm */
  c_off = w_off;
  memcpy(mem + d_start + shm_addr_len, &c_off, shm_addr_len);

  /* calculate the address of the new end of the buffer */
  w_off = (w_off + added_bytes) % d_size;

  /* copy the new address to shm */
  memcpy(mem + d_start, &w_off, shm_addr_len);
  return len;
}

/* READ DATA FROM SHM */

inline int dgc_shm_t::dread_addr(int num)
{ 
  return d_start + shm_index_len + ((r_off + num) % d_size);
}

inline void dgc_shm_t::dread(unsigned char *data, int len, int offset)
{ 
  int part1, part2;
  unsigned int start = r_off + offset;
  if(start < d_size && start + len > d_size) {
    /* the data has to be split in to pieces */
    part1 = d_size - r_off - offset;
    part2 = len - part1;
    memcpy(data, mem + dread_addr(offset), part1);
    memcpy(data + part1, mem + d_start + shm_index_len, part2);
  } 
  else 
    /* the data fits in the buffer in one piece */
    memcpy(data, mem + dread_addr(offset), len);
}

int dgc_shm_t::start_partial_read_internal(int current)
{
  unsigned int off, len, c_off, packet_size;

  if(!initialized) {
    dgc_ferror(DGC_PF, "Shm client has not been initialized.");
    return -1;
  }
  if(int_type != CLIENT) {
    dgc_ferror(DGC_PF, "Can only be called by client interfaces.");
    return -1;
  }
  test_session_id();

  /* read the current mem pointer of data segement */
  memcpy(&off, mem + d_start, shm_addr_len);

  /* check if new packet is waiting */
  if(off == r_off) 
    return 0;

  if(current) {
    memcpy(&c_off, mem + d_start + shm_addr_len, shm_addr_len);
    r_off = c_off;
  }

  /* check if first byte is IDENTFIER */
  if(mem[dread_addr(0)] != shm_identifier) {
    dgc_warning("Corrupt packet, re-initalizing shm queue.");
    r_off = off;
    return 0;
  }

  /* read length of packet */ 
  dread((unsigned char *)&len, shm_addr_len, shm_idtf_len);

  packet_size = shm_idtf_len + shm_addr_len + len + shm_chksum_len;

  partial_bytes_total = len;
  partial_bytes_read = 0;
  partial_read_in_progress = true;
  partial_chksum = 0;
  return len;
}

int dgc_shm_t::start_partial_read(void)
{
  return start_partial_read_internal(0);
}

int dgc_shm_t::start_partial_current_read(void)
{
  return start_partial_read_internal(1);
}

int dgc_shm_t::partial_nread(unsigned int maxlen, unsigned char *s)
{
  unsigned int len;

  if(!initialized) {
    dgc_ferror(DGC_PF, "Shm client has not been initialized.");
    return -1;
  }
  if(int_type != CLIENT) {
    dgc_ferror(DGC_PF, "Can only be called by client interfaces.");
    return -1;
  }
  if(!partial_read_in_progress) {
    dgc_ferror(DGC_PF, "Partial read has not been started.");
    return -1;
  }

  len = maxlen;
  if(len > partial_bytes_total - partial_bytes_read)
    len = partial_bytes_total - partial_bytes_read;

  /* copy data to user memory */
  dread(s, len, shm_idtf_len + shm_addr_len + partial_bytes_read);

  partial_chksum = shm_chksum(s, len, partial_chksum);
  partial_bytes_read += len;
  return len;
}

int dgc_shm_t::partial_read(unsigned char *s)
{
  return partial_nread(INT_MAX, s);
}

int dgc_shm_t::abort_partial_read(void)
{
  unsigned int packet_size;

  if(!initialized) {
    dgc_ferror(DGC_PF, "Shm client has not been initialized.");
    return -1;
  }
  if(int_type != CLIENT) {
    dgc_ferror(DGC_PF, "Can only be called by client interfaces.");
    return -1;
  }
  if(!partial_read_in_progress) {
    dgc_ferror(DGC_PF, "Partial read has not been started.");
    return -1;
  }

  packet_size = shm_idtf_len + shm_addr_len + partial_bytes_total + 
    shm_chksum_len;
  r_off = (r_off + packet_size) % d_size;
  partial_read_in_progress = false;
  return 0;
}

int dgc_shm_t::finish_partial_read(void)
{
  unsigned int packet_size, off;

  if(!initialized) {
    dgc_ferror(DGC_PF, "Shm client has not been initialized.");
    return -1;
  }
  if(int_type != CLIENT) {
    dgc_ferror(DGC_PF, "Can only be called by client interfaces.");
    return -1;
  }
  if(!partial_read_in_progress) {
    dgc_ferror(DGC_PF, "Partial read has not been started.");
    return -1;
  }

  packet_size = shm_idtf_len + shm_addr_len + partial_bytes_total + 
    shm_chksum_len;

  /* test checksum */
  if(partial_bytes_total != partial_bytes_read) {
    dgc_warning("Incomplete partial read %d bytes left over.", 
		partial_bytes_total - partial_bytes_read);
    memcpy(&off, mem + d_start, shm_addr_len);
    r_off = off;
    partial_read_in_progress = false;
    return 0;
  }
  else if(!use_checksum || mem[dread_addr(packet_size - shm_chksum_len)] == 
     partial_chksum) {
    r_off = (r_off + packet_size) % d_size;
    partial_read_in_progress = false;
    return partial_bytes_read;
  } 
  else {
    dgc_warning("Checksum error, re-initalizing shm queue.");
    memcpy(&off, mem + d_start, shm_addr_len);
    r_off = off;
    partial_read_in_progress = false;
    return 0;
  }
}

int dgc_shm_t::nread_internal(int maxlen, unsigned char *s, int current)
{
  unsigned int len, off, c_off, packet_size, bytes_read;

  if(!initialized) {
    dgc_ferror(DGC_PF, "Shm client has not been initialized.");
    return -1;
  }
  if(int_type != CLIENT) {
    dgc_ferror(DGC_PF, "Can only be called by client interfaces.");
    return -1;
  }
  test_session_id();

  /* read the current mem pointer of data segement */
  memcpy(&off, mem + d_start, shm_addr_len);

  /* check if new packet is waiting */
  if(off == r_off) 
    return 0;
  
  if(current) {
    memcpy(&c_off, mem + d_start + shm_addr_len, shm_addr_len);
    r_off = c_off;
  }

  /* check if first byte is IDENTFIER */
  if(mem[dread_addr(0)] != shm_identifier) {
    dgc_warning("Corrupt packet, re-initalizing shm queue.");
    r_off = off;
    return 0;
  }
  bytes_read = shm_idtf_len;
  
  /* read length of packet */ 
  dread((unsigned char *)&len, shm_addr_len, bytes_read);
  bytes_read += shm_addr_len;
  packet_size = shm_idtf_len + shm_addr_len + len + shm_chksum_len;
  
  /* is packet too long ? */
  if(len > (unsigned int)maxlen) {
    dgc_warning("Packet too long, skipping packet.");
    r_off = (r_off + packet_size) % d_size;
    return 0;
  }
  
  /* copy data to user memory */
  dread(s, len, bytes_read);
  bytes_read += len;
  
  /* test checksum */
  if(!use_checksum || mem[dread_addr(packet_size - shm_chksum_len)] == 
     shm_chksum(s, len)) {
    r_off = (r_off + packet_size) % d_size;
    return len;
  } 
  else {
    dgc_warning("Checksum error, re-initalizing shm queue.");
    r_off = off;
    return 0;
  }
}

int dgc_shm_t::nread(int maxlen, unsigned char *s)
{
  return nread_internal(maxlen, s, 0);
}

int dgc_shm_t::read(unsigned char *s)
{
  return nread_internal(INT_MAX, s, 0);
}

int dgc_shm_t::current_nread(int maxlen, unsigned char *s)
{
  return nread_internal(maxlen, s, 1);
}

int dgc_shm_t::current_read(unsigned char *s)
{
  return nread_internal(INT_MAX, s, 1);
}

/* WRITE STATIC (INFO) INFORMATION TO SHM */

int dgc_shm_t::iwrite_addr(int num)
{ 
  return i_start + shm_index_len + num; 
}

void dgc_shm_t::iwrite(unsigned char *data, int len, int offset)
{ 
  /* the data fits in the buffer in one piece */
  memcpy(mem + iwrite_addr(offset), data, len);
}

int dgc_shm_t::add_info(unsigned int len, unsigned char *data)
{
  int added_bytes = 0;

  if(!initialized) {
    dgc_ferror(DGC_PF, "Shm client has not been initialized.");
    return -1;
  }
  if(int_type != SERVER) {
    dgc_ferror(DGC_PF, "Can only be called by server interfaces.");
    return -1;
  }

  /* is there enough space left for new packet? */
  if(i_off + len + shm_addr_len + shm_idtf_len + shm_chksum_len > i_size) 
    return 0;

  /* first byte is the SHM_IDENTIFIER BYTE */
  mem[iwrite_addr(0)] = shm_identifier;
  added_bytes += shm_idtf_len;

  /* copy the length of the data package to shm */
  iwrite((unsigned char *)&len, shm_addr_len, added_bytes);
  added_bytes += shm_addr_len;

  /* copy the data of the package to shm */
  iwrite(data, len, added_bytes);
  added_bytes += len;

  /* copy the chksum to shm */
  if(use_checksum)
    mem[iwrite_addr(added_bytes)] = shm_chksum(data, len);
  added_bytes += shm_chksum_len;

  /* calculate the address of the new end of the buffer */
  i_off = i_off + added_bytes;

  /* copy the new address to shm */
  memcpy(mem + i_start, &i_off, shm_addr_len);

  return len;
}

/* READ STATIC (INFO) INFORMATION FROM SHM */

int dgc_shm_t::get_info(unsigned char *s)
{
  unsigned int len, off, packet_size;

  if(!initialized) {
    dgc_ferror(DGC_PF, "Shm client has not been initialized.");
    return -1;
  }
  if(int_type != SERVER) {
    dgc_ferror(DGC_PF, "Can only be called by client interfaces.");
    return -1;
  }

  /* read the current mem pointer of data segement */
  memcpy(&off, mem + i_start, shm_addr_len);

  /* check if new packet is waiting */
  if(off != (unsigned int)i_off) {
    /* check if first byte is IDENTFIER */
    if(mem[i_start + shm_index_len + i_off] != shm_identifier) {
      dgc_warning("Corrupt packet, re-initalizing shm queue.");
      return -1;
    }

    /* read length of packet */ 
    memcpy(&len, mem + i_start + shm_index_len + i_off + 
	   shm_idtf_len, shm_addr_len);
    packet_size = shm_idtf_len + shm_addr_len + len + shm_chksum_len;
    
    /* copy packet to user mem */
    memcpy(s, mem + i_start + shm_index_len + i_off + 
	   shm_idtf_len + shm_addr_len, len);
    
    /* compute checksum and compare */
    if(!use_checksum ||
       mem[i_start + shm_index_len + i_off + 
	   packet_size - shm_chksum_len] == shm_chksum(s, len)) {
      i_off += packet_size;
      return len;
    } 
    else {
      dgc_warning("Checksum error, re-initalizing shm queue.");
      return -1;
    }
  }
  return 0;
}

/* TOOLS */

long long dgc_shm_t::get_max(void)
{
  FILE    * fp = NULL;
  int       n = 0;
  long long max = -1;
  char      number[100];

  if((fp = fopen(PROC_SHM_MAX, "r")) == 0) {
    dgc_ferror(DGC_PF, "Can't open shm info: %s", PROC_SHM_MAX);
    return -1;
  } 
  else {
    n = fscanf(fp, "%s", number);
    if(n != 1) 
      max = -1;
    else 
      max = atoll(number);
    fclose(fp);
  }
  return max;
}

int dgc_shm_t::set_max(long long max)
{
  FILE    * fp = NULL;

  if((fp = fopen(PROC_SHM_MAX, "w")) == 0) {
    dgc_ferror(DGC_PF, "Can't open shm info: %s", PROC_SHM_MAX);
    return -1;
  } 
  else {
    fprintf(fp, "%lld\n", max);
    fclose(fp);
  }
  return 1;
}

