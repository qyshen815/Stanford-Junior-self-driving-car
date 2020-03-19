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
  double          dt, lt, nt, mb = 0.0;

  mem = (unsigned char *) malloc( __SHM_TEST_DATA_SIZE * sizeof(char) );
  dgc_test_alloc(mem);

  for (n=0; n<__SHM_TEST_DATA_SIZE; n++) {
    mem[n] = (unsigned char) (n%256);
  }

  shm.init_server( __SHM_TEST_KEY, __SHM_TEST_INFO_SIZE, __SHM_TEST_DATA_SIZE);

  lt = dgc_get_time();

  while (1) {

    n = shm.write(100000,mem);
    mb += n / (1024.0*1024.0);

    nt = dgc_get_time();
    dt = nt-lt;
    if (dt>1.0) {
      fprintf( stderr, "write speed: %.2f MB/s\n", mb/dt );
      mb = 0.0;
      lt = nt;
    }
    
    usleep(1000);
  }
}
