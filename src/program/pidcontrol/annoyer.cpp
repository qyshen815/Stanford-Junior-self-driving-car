#include <roadrunner.h>

void sigint_handler(int x)
{
  fprintf(stderr, "Received signal %d\n", x);
}

int main(void)
{
  signal(SIGINT, sigint_handler);
  while(1) 
    sleep(1);
}
