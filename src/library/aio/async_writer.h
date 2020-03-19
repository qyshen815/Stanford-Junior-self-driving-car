#ifndef DGC_ASYNC_WRITER_H
#define DGC_ASYNC_WRITER_H

#include <aio.h>
#include <vector>
#ifdef __DARWIN__
#ifndef AIO64_DEFINED
#define AIO64_DEFINED
typedef aiocb aiocb64;

extern inline int aio_write64(struct aiocb* aiocbp) {return aio_write(aiocbp);}
extern inline int aio_suspend64(const struct aiocb* const list[], int nent, const struct timespec* timeout) {return aio_suspend(list, nent, timeout);}
extern inline ssize_t aio_return64(struct aiocb *aiocbp) {return aio_return(aiocbp);}
extern inline int aio_error64(const struct aiocb *aiocbp) {return aio_error(aiocbp);}
#endif
#endif

namespace dgc {

// This class implements buffered, asyncronous file output.  When data 
// is written through this interface, the write will return immediately
// unless all bufffers are full.  The writes will block if you are outputting
// data above the maximum rate your file system can support.

// Example usage:
//
//   AsyncWriter writer;
//
//   if (writer.Open("test.dat") < 0) 
//    return -1;      // could not open file 
//   writer.Write(num_data_bytes, data);
//   ...
//   writer.Close();

// Default size of output chunk
const unsigned int kDefaultChunkSize = 65536;

// Default number of output buffers
const int          kDefaultNumBuffers = 100;

class AsyncWriter {
 public:
  AsyncWriter();
  ~AsyncWriter();

  // Opens the file for asyncronous output.  The default number of output
  // buffers and buffer size can be overridden if necessary, however
  // the default parameters work well for most usages.  Returns -1 if
  // the file cannot be opened, 0 otherwise.
  int Open(char *filename, unsigned int num_buffers = kDefaultNumBuffers,
	   unsigned int buffer_size = kDefaultChunkSize);

  // Writes any remaining buffered data to the file, and then closes it.
  void Close(void);

  // Queues up len bytes of data to be written to the file.  If there is
  // enough buffer space left, the write call will return immediately. 
  // If there is not enough buffer space left, this call will block until
  // there is enough space left.
  int Write(unsigned int len, unsigned char *data);

  // Returns the file descriptor for the file.  
  int fd(void) { return fd_; }

 private:
  // Initialiates a write of len bytes from the Nth output buffer.
  int WriteBlock(int n, unsigned int len);

  // File descriptor of output file, -1 if not initialized
  int fd_;

  // Number of output buffers
  unsigned int num_buffers_;

  // Size of each output buffer
  unsigned int buffer_size_;

  // asyncronous buffer parameters for each buffer
  std::vector <aiocb64> aio_;

  // flag marking whether a write is in progress for each buffer
  std::vector <bool> occupied_;

  // Ouput ring buffer.  Data is cached in these buffers until the 
  // asyncronous write is complete.
  std::vector <unsigned char *> buffer_;

  // index of the next buffer to be written to
  unsigned int current_buffer_;

  // Number of bytes in the current buffer leftover from a previous
  // write.
  unsigned int unwritten_bytes_;
  
  // Debugging counts, number of times the writer had to wait for 
  // an available buffer versus the total number of write calls
  unsigned int wait_count_;
  unsigned int write_count_;
};

}

#endif
