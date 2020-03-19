#include <roadrunner.h>
#include "blf_id.h"
#include "blf_convert.h"
#include "blf.h"

#define          BLF_INDEX_SIZE_STEP               2000

static inline unsigned char blf_chksum(int numbytes, unsigned char *bytes)
{
  unsigned char chk = 0;
  int i;

  for(i = 0; i < numbytes; i++) 
    chk += bytes[i];
  return chk;
}

blf_index_p blf_index_load(char *filename)
{
  char        *index_filename, line[1001];
  dgc_FILE    *fp;
  blf_index_p index;

  index_filename = (char *)calloc(1, strlen(filename) + 10);
  dgc_test_alloc(index_filename);
  sprintf(index_filename, "%s.idx.gz", filename);

  fp = dgc_fopen(index_filename, "r");
  if(fp == NULL) {
    free(index_filename);
    return NULL;
  } 
  free(index_filename);

  index = (blf_index_p)calloc(1, sizeof(blf_index_t));
  dgc_test_alloc(index);

  index->num_blocks = 0;
  index->max_blocks = 0;
  index->block = NULL;

  while(dgc_fgets(line, 1000, fp)) {
    if(index->num_blocks >= index->max_blocks) {
      index->max_blocks += BLF_INDEX_SIZE_STEP;
      index->block = 
        (blf_index_entry_p)realloc(index->block, index->max_blocks * 
            sizeof(blf_index_entry_t));
      dgc_test_alloc(index->block);
    }
    if(sscanf(line, "%d %lf %lld", 
          &(index->block[index->num_blocks].id),
          &(index->block[index->num_blocks].timestamp),
          (long long int *)&(index->block[index->num_blocks].offset)) == 3) 
      index->num_blocks++;
    else
      dgc_fatal_error("BLF index format has changed.  Please reindex file.");
  }
  dgc_fclose(fp);
  return index;
}

blf_t::blf_t()
{
  status = UNINITIALIZED;
  async_mode = true;
}

blf_t::blf_t(bool async_write_mode) 
{
  async_mode = async_write_mode;
}

blf_t::~blf_t()
{
  if(status != UNINITIALIZED)
    close();
}

int blf_t::open(char *fname, char *mode)
{
  char *filename = NULL;

  if(status != UNINITIALIZED) {
    dgc_ferror(DGC_PF, "BLF file already opened.");
    return BLF_INVALID;
  }

  if(!strcasecmp(fname, "-") ||
      !strcasecmp(fname, "stdout") ||
      !strcasecmp(fname, "/dev/stout")) {
    filename = strdup("stdout");
    fp = stdout;
  } 
  else if(!strcasecmp(fname, "-") ||
      !strcasecmp(fname, "stdin") ||
      !strcasecmp(fname, "/dev/stdin")) {
    filename = strdup("stdin");
    fp = stdin;
  } 
  else {
    filename = strdup(fname);

    if(!strcasecmp(mode, "w")) {
      if(async_writer.Open(filename) < 0) {
        dgc_error("Can't open file %s for writing.", filename);
        free(filename);
        return BLF_INVALID;
      }
      if((fp = fdopen(async_writer.fd(), mode)) == 0) {
        dgc_error("Can't open file %s for writing.", filename);
        free(filename);
        return BLF_INVALID;
      }
      status = WRITE;
    } 
    else if(!strcasecmp(mode, "r")) {
      fp = fopen64(filename, mode);
      if(fp == NULL) {
        dgc_error("Can't open blf file %s.", filename);
        free(filename);
        return BLF_INVALID;
      }
      status = READ;
    } 
    else {
      dgc_error("Unknown file mode %s.", mode);
      free(filename);
      return BLF_INVALID;
    }
  }
  free(filename);
  found_first_timestamp = false;
  partial_read_in_progress = false;
  partial_write_in_progress = false;
  return BLF_OK;
}

int blf_t::close(void)
{
  if(status == UNINITIALIZED) {
    dgc_ferror(DGC_PF, "BLF file is not open.");
    return BLF_INVALID;
  }
  async_writer.Close();
  return BLF_OK;
}

int blf_t::start_partial_read(unsigned short *id, double *timestamp,
    unsigned int *len)
{
  unsigned char buf[32];
  unsigned int  r;

  if(status == UNINITIALIZED) {
    dgc_ferror(DGC_PF, "BLF file is not open.");
    return BLF_INVALID;
  }
  if(status != READ) {
    dgc_ferror(DGC_PF, "Can only be called on files opened for reading.");
    return BLF_INVALID;
  }
  if(partial_read_in_progress) {
    dgc_ferror(DGC_PF, "Partial read already in progress.");
    return BLF_INVALID;
  }

  /* read byte by byte looking for a start byte */
  do {
    if(fread(buf, 1, 1, fp) != 1) 
      return BLF_EOF;
  } while(buf[0] != BLF_START_BYTE);

  /* read timestamp */
  r = fread(buf, 1, 8, fp);
  if(r != 8) 
    return BLF_EOF;
  *timestamp = blf_uchar2double(buf);
  if(*timestamp < 0)
    return BLF_TIMESTAMP;

  /* read id */
  r = fread(buf, 1, 2, fp);
  if(r != 2) 
    return BLF_EOF;
  *id = blf_uchar2ushort(buf);

  /* read length */
  r = fread(buf, 1, 4, fp);
  if(r != 4) 
    return BLF_EOF;
  *len = blf_uchar2uint(buf);

  partial_chksum = 0;
  partial_bytes_left = *len;
  partial_bytes_read = 0;
  partial_read_in_progress = true;

  if(!found_first_timestamp) {
    first_timestamp = *timestamp;
    found_first_timestamp = true;
  }
  return BLF_OK;
}

int blf_t::partial_read(unsigned char *data, unsigned int len)
{
  unsigned int r;

  if(status == UNINITIALIZED) {
    dgc_ferror(DGC_PF, "BLF file is not open.");
    return BLF_INVALID;
  }
  if(status != READ) {
    dgc_ferror(DGC_PF, "Can only be called on files opened for reading.");
    return BLF_INVALID;
  }
  if(!partial_read_in_progress) {
    dgc_ferror(DGC_PF, "Partial read has not been started.");
    return BLF_INVALID;
  }

  if(len > partial_bytes_left) {
    dgc_ferror(DGC_PF, "%d bytes requested : only %d left in partial read.",
        len, partial_bytes_left);
    return BLF_INVALID;
  }

  /* read actual data */
  r = fread(data, 1, len, fp);
  if(r != len) 
    return BLF_EOF;

  partial_bytes_left -= len;
  partial_chksum += blf_chksum(len, data);
  return BLF_OK;
}

int blf_t::finish_partial_read(void)
{
  unsigned int   r;
  unsigned char  chksum;

  if(status == UNINITIALIZED) {
    dgc_ferror(DGC_PF, "BLF file is not open.");
    return BLF_INVALID;
  }
  if(status != READ) {
    dgc_ferror(DGC_PF, "Can only be called on files opened for reading.");
    return BLF_INVALID;
  }
  if(!partial_read_in_progress) {
    dgc_ferror(DGC_PF, "Partial read has not been started.");
    return BLF_INVALID;
  }
  if(partial_bytes_left > 0) 
    dgc_warning("%d bytes leftover in partial read.", partial_bytes_left);

  partial_read_in_progress = false;

  /* read the checksum */
  r = fread(&chksum, 1, 1, fp);
  if(r != 1) 
    return BLF_EOF;

  /* make sure the checksum is correct */
  if(chksum != partial_chksum)
    return BLF_CHKSUM;
  return BLF_OK;
}

int blf_t::abort_partial_read(void)
{
  long long int offset;

  if(status == UNINITIALIZED) {
    dgc_ferror(DGC_PF, "BLF file is not open.");
    return BLF_INVALID;
  }
  if(status != READ) {
    dgc_ferror(DGC_PF, "Can only be called on files opened for reading.");
    return BLF_INVALID;
  }
  if(!partial_read_in_progress) {
    dgc_ferror(DGC_PF, "Partial read has not been started.");
    return BLF_INVALID;
  }

  offset = tell();
  seek(partial_bytes_left + 1, SEEK_CUR);
  partial_read_in_progress = false;
  return BLF_OK;
}

int blf_t::reset_partial_read(void)
{
  long long int offset;

  if(status == UNINITIALIZED) {
    dgc_ferror(DGC_PF, "BLF file is not open.");
    return BLF_INVALID;
  }
  if(status != READ) {
    dgc_ferror(DGC_PF, "Can only be called on files opened for reading.");
    return BLF_INVALID;
  }
  if(!partial_read_in_progress) {
    dgc_ferror(DGC_PF, "Partial read has not been started.");
    return BLF_INVALID;
  }

  offset = tell();
  seek(-partial_bytes_read-15LL, SEEK_CUR);
  partial_read_in_progress = false;
  return BLF_OK;
}

int blf_t::read_data(unsigned short *id, double *timestamp,
    unsigned char **data, unsigned int *len, 
    unsigned int *max_size)
{
  unsigned char  buf[32];
  unsigned int   r;
  unsigned char  chksum;
  bool found = false;

  if(status == UNINITIALIZED) {
    dgc_ferror(DGC_PF, "BLF file is not open.");
    return BLF_INVALID;
  }
  if(status != READ) {
    dgc_ferror(DGC_PF, "Can only be called on files opened for reading.");
    return BLF_INVALID;
  }
  if(partial_read_in_progress) {
    dgc_ferror(DGC_PF, "Cannot be called while partial read in progress.");
    return BLF_INVALID;
  }
  
  /* read byte by byte looking for a start byte */
  do {
    if(fread(buf, 1, 1, fp) != 1) 
      return BLF_EOF;
    if(buf[0] == BLF_START_BYTE) 
      found = true;
  } while(!found);
  
  /* read timestamp */
  r = fread(buf, 1, 8, fp);
  if(r != 8) 
    return BLF_EOF;
  *timestamp = blf_uchar2double(buf);
  if(*timestamp < 0)
    return BLF_TIMESTAMP;

  /* read id */
  r = fread(buf, 1, 2, fp);
  if(r != 2) 
    return BLF_EOF;
  *id = blf_uchar2ushort(buf);

  /* read length */
  r = fread(buf, 1, 4, fp);
  if(r != 4) 
    return BLF_EOF;
  *len = blf_uchar2uint(buf);
  if(*len > *max_size) {
    *max_size = *len;
    *data = (unsigned char *)realloc(data, *max_size);
    dgc_test_alloc(*data);
  }

  /* read actual data */
  r = fread(*data, 1, *len, fp);
  if(r != *len) 
    return BLF_EOF;

  /* read the checksum */
  r = fread(&chksum, 1, 1, fp);
  if(r != 1) 
    return BLF_EOF;

  /* make sure the checksum is correct */
  if(chksum != blf_chksum(*len, *data)) 
    return BLF_CHKSUM;

  if(!found_first_timestamp) {
    first_timestamp = *timestamp;
    found_first_timestamp = true;
  }
  return BLF_OK;
}

int blf_t::read_pkt(blf_pkt_t *pkt)
{
  return read_data(&(pkt->id), &(pkt->timestamp), &(pkt->data),
      &(pkt->len), &(pkt->max_len));
}

int blf_t::start_partial_write(unsigned int len, double timestamp, 
    unsigned short id)
{
  unsigned char bytes[32];
  int           n = 0;

  if(status == UNINITIALIZED) {
    dgc_ferror(DGC_PF, "BLF file is not open.");
    return BLF_INVALID;
  }
  if(status != WRITE) {
    dgc_ferror(DGC_PF, "Can only be called on files opened for writing.");
    return BLF_INVALID;
  }
  if(partial_write_in_progress) {
    dgc_ferror(DGC_PF, "Cannot be called while partial write in progress.");
    return BLF_INVALID;
  }

  /* start byte */
  bytes[0] = BLF_START_BYTE;

  /* timestamp */
  blf_double2uchar(timestamp, bytes + 1);

  /* sensor local id */
  blf_ushort2uchar(id, bytes + 9);

  /* data length */
  blf_uint2uchar(len, bytes + 11);

  if(async_mode)
    n += async_writer.Write(15, bytes);
  else
    n += fwrite(bytes, 1, 15, fp);

  partial_chksum = 0;
  partial_bytes_left = len;
  partial_bytes_written = 0;
  partial_write_in_progress = true;
  return BLF_OK;
}

int blf_t::partial_write(unsigned int len, unsigned char *data)
{
  int n;

  if(status == UNINITIALIZED) {
    dgc_ferror(DGC_PF, "BLF file is not open.");
    return BLF_INVALID;
  }
  if(status != WRITE) {
    dgc_ferror(DGC_PF, "Can only be called on files opened for writing.");
    return BLF_INVALID;
  }
  if(!partial_write_in_progress) {
    dgc_ferror(DGC_PF, "Partial write has not been started.");
    return BLF_INVALID;
  }

  if(len > partial_bytes_left) {
    dgc_ferror(DGC_PF, "%d bytes requested : only %d left in partial write.",
        len, partial_bytes_left);
    return BLF_INVALID;
  }

  if(async_mode)
    n = async_writer.Write(len, data);
  else
    n = fwrite(data, 1, len, fp);

  partial_bytes_left -= len;
  partial_bytes_written += n;
  partial_chksum += blf_chksum(len, data);
  return BLF_OK;
}

int blf_t::finish_partial_write(void)
{
  if(status == UNINITIALIZED) {
    dgc_ferror(DGC_PF, "BLF file is not open.");
    return BLF_INVALID;
  }
  if(status != WRITE) {
    dgc_ferror(DGC_PF, "Can only be called on files opened for writing.");
    return BLF_INVALID;
  }
  if(!partial_write_in_progress) {
    dgc_ferror(DGC_PF, "Partial write has not been started.");
    return BLF_INVALID;
  }
  if(partial_bytes_left > 0) 
    dgc_warning("%d bytes leftover in partial write.", partial_bytes_left);

  partial_write_in_progress = false;
  if(async_mode)
    async_writer.Write(1, &partial_chksum);
  else
    fwrite(&partial_chksum, 1, 1, fp);
  return BLF_OK;
}

int blf_t::write_data(unsigned int len, unsigned char *data,
    double timestamp, unsigned short id)
{
  unsigned char chksum;
  unsigned char bytes[32];
  int           n = 0;

  if(status == UNINITIALIZED) {
    dgc_ferror(DGC_PF, "BLF file is not open.");
    return BLF_INVALID;
  }
  if(status != WRITE) {
    dgc_ferror(DGC_PF, "Can only be called on files opened for writing.");
    return BLF_INVALID;
  }
  if(partial_write_in_progress) {
    dgc_ferror(DGC_PF, "Cannot be called while partial write in progress.");
    return BLF_INVALID;
  }

  /* start byte */
  bytes[0] = BLF_START_BYTE;

  /* timestamp */
  blf_double2uchar(timestamp, bytes + 1);

  /* sensor local id */
  blf_ushort2uchar(id, bytes + 9);

  /* data length */
  blf_uint2uchar(len, bytes + 11);

  /* checksum */
  chksum = blf_chksum(len, data);

  if(async_mode) {
    n += async_writer.Write(15, bytes);
    n += async_writer.Write(len, data);
    n += async_writer.Write(1, &chksum);
  }
  else {
    n += fwrite(bytes, 1, 15, fp);
    n += fwrite(data, 1, len, fp);
    n += fwrite(&chksum, 1, 1, fp);
  }
  return len;
}

int blf_t::write_pkt(blf_pkt_t *pkt)
{
  return write_data(pkt->len, pkt->data, pkt->timestamp, pkt->id);
}

int blf_t::seek_timestamp_nearest(double t)
{
  return 0;
}

int blf_t::seek_timestamp(double t)
{
  unsigned short int pkt_id;
  unsigned int pkt_len;
  double pkt_timestamp;
  int err;

  if(status == UNINITIALIZED) {
    dgc_ferror(DGC_PF, "BLF file is not open.");
    return BLF_INVALID;
  }
  if(status != READ) {
    dgc_ferror(DGC_PF, "Can only be called on files opened for reading.");
    return BLF_INVALID;
  }

  rewind();
  if(found_first_timestamp && t < first_timestamp) 
    return BLF_OK;

  do {
    do {
      err = start_partial_read(&pkt_id, &pkt_timestamp, &pkt_len);
      if(err == BLF_EOF)
        return BLF_EOF;
    } while(err != BLF_OK);

    if(pkt_timestamp > t) {
      err = reset_partial_read();
      break;
    }

    err = abort_partial_read();
    if(err == BLF_EOF)
      return BLF_EOF;
  } while(pkt_timestamp < t);
  return BLF_OK;
}

int blf_t::seek(long long offset, int whence)
{
  if(status == UNINITIALIZED) {
    dgc_ferror(DGC_PF, "BLF file is not open.");
    return BLF_INVALID;
  }
  if(status != READ) {
    dgc_ferror(DGC_PF, "Can only be called on files opened for reading.");
    return BLF_INVALID;
  }
  if(fseeko64(fp, offset, whence) != 0)
    return BLF_INVALID;
  return BLF_OK;
}

long long blf_t::tell(void)
{
  if(status == UNINITIALIZED) {
    dgc_ferror(DGC_PF, "BLF file is not open.");
    return BLF_INVALID;
  }
  if(status != READ) {
    dgc_ferror(DGC_PF, "Can only be called on files opened for reading.");
    return BLF_INVALID;
  }
  return ftello64(fp);
}

int blf_t::rewind(void)
{
  if(status == UNINITIALIZED) {
    dgc_ferror(DGC_PF, "BLF file is not open.");
    return BLF_INVALID;
  }
  if(status != READ) {
    dgc_ferror(DGC_PF, "Can only be called on files opened for reading.");
    return BLF_INVALID;
  }
  ::rewind(fp);
  return BLF_OK;
}



