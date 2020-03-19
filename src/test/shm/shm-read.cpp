#include <roadrunner.h>
#include <shm_wrapper.h>

#define __SHM_TEST_KEY         12345
#define __SHM_TEST_INFO_SIZE   4000
#define __SHM_TEST_DATA_SIZE   500000

int 
main( void )
{
  dgc_shm_t       shm;
  unsigned char  *mem;
  int             n;

  mem = (unsigned char *) malloc( __SHM_TEST_DATA_SIZE * sizeof(char) );
  dgc_test_alloc(mem);

  shm.init_client( __SHM_TEST_KEY, __SHM_TEST_INFO_SIZE, __SHM_TEST_DATA_SIZE);

  while (1) {
    
    while (shm.bytes_waiting()) {
      n = shm.read(mem);
      fprintf( stderr, "[%d]", n );
    }
    usleep(1);
  }
}
