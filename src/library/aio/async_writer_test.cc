#include <roadrunner.h>
#include "async_writer.h"

using dgc::AsyncWriter;

void test_chunk_size(char *filename, int chunk_size, int num_chunks)
{
  unsigned char *buffer;
  AsyncWriter writer;
  int i;

  buffer = new unsigned char[chunk_size];
  
  for (i = 0; i < chunk_size; i++)
    buffer[i] = (i % 100);
  
  writer.Open(filename);
  for (i = 0; i < num_chunks; i++) 
    writer.Write(chunk_size, buffer);
  writer.Close();

  delete [] buffer;
}

const int kLadybugPacketSize = 1000000;

void test_ladybug_rate(void)
{
  unsigned char *buffer;
  AsyncWriter writer;
  int i, count = 0;
  double t1, t2, write_time = 0;

  buffer = new unsigned char[kLadybugPacketSize];
  for (i = 0; i < kLadybugPacketSize; i++)
    buffer[i] = (i % 100);

  writer.Open("ladybug.dat");

  do {
    t1 = dgc_get_time();
    writer.Write(kLadybugPacketSize, buffer);
    t2 = dgc_get_time();
    write_time += (t2 - t1);
    usleep(33333);
    count++;
  } while(count < 10 * 30);

  writer.Close();
  delete [] buffer;

  fprintf(stderr, "Write time = %f\n", write_time);
}

void test_ladybug_sync(void)
{
  unsigned char *buffer;
  int i, count = 0;
  FILE *fp;
  double t1, t2, write_time = 0;
  
  buffer = new unsigned char[kLadybugPacketSize];
  for (i = 0; i < kLadybugPacketSize; i++)
    buffer[i] = (i % 100);

  fp = fopen("ladybug.dat", "w");
  if(fp == NULL)
    dgc_die("Error: could not open file.\n");

  do {
    t1 = dgc_get_time();
    fwrite(buffer, kLadybugPacketSize, 1, fp);
    t2 = dgc_get_time();
    write_time += (t2 - t1);
    usleep(33333);
    count++;
  } while(count < 10 * 30);

  fclose(fp);
  delete [] buffer;

  fprintf(stderr, "Write time = %f\n", write_time);
}

int main(void)
{
  fprintf(stderr, "Async:\n");
  test_ladybug_rate();
  fprintf(stderr, "Sync:\n");
  test_ladybug_sync();
  return 0;

  test_chunk_size("test.dat", 10000, 100000);
  test_chunk_size("test2.dat", 65536, 15258);
  test_chunk_size("test3.dat", 100000, 10000);
  return 0;
}
