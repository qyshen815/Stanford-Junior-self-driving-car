#include <roadrunner.h>

int main(int argc, char **argv)
{ 
  FILE *fp;
  int size, code;
  
  if(argc != 2)
    dgc_die("Error: not enough arguments.\n"
	    "Usage: %s <file>\n", argv[0]);

  if((fp = fopen(argv[1], "r")) == 0) 
    dgc_die("Error: can't open file %s\n", argv[1]);

  size = 0;
  while(!feof(fp)) {
    fgetc(fp);
    size++;
  }
  size--;
  printf("/* file: %s, size: %d */\n", argv[1], size); 
  printf("unsigned char data[%d] = {", size);
  rewind(fp);
  size = 0;
  while(!feof(fp)) {
    code = fgetc(fp);
    if(code >= 0)
      printf("%s%d", size > 0 ? ", " : " ", code);
    size++;
  }
  printf(" };\n");
  fclose(fp);
  return 0;
}
