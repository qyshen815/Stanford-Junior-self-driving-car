#include <roadrunner.h>
#include <async_writer.h>
#include <gtest/gtest.h>

using dgc::AsyncWriter;

// Creates an empty file

TEST(AsyncWriterTest, GeneratesEmptyFile) {
  AsyncWriter writer;

  ASSERT_EQ(0, writer.Open("/tmp/test1.dat"))
    << "Could not open file.";
  writer.Close();
  ASSERT_EQ(1, dgc_file_exists("/tmp/test1.dat")) 
    << "Did not create file.";
  EXPECT_EQ(0, dgc_file_size("/tmp/test1.dat"));
}

// Writes N bytes, and closes immediately.  Makes sure that all of the
// bytes actually get written to the file, and then checks to see
// if the file contains the same data.

const unsigned int kBufferSize = 50000000;

TEST(AsyncWriterTest, WritesAllData) {
  unsigned char *data, *readback;
  AsyncWriter writer;
  unsigned int i;
  bool same;
  FILE *fp;

  data = new unsigned char[kBufferSize];
  readback = new unsigned char[kBufferSize];

  for (i = 0; i < kBufferSize; i++)
    data[i] = i % 256;

  ASSERT_EQ(0, writer.Open("/tmp/test1.dat"))
    << "Could not open file.";
  ASSERT_EQ(0, writer.Write(kBufferSize, data))
    << "Write was unsuccessful.";
  writer.Close();
  ASSERT_EQ(1, dgc_file_exists("/tmp/test1.dat")) 
    << "Did not create file.";
  EXPECT_EQ(kBufferSize, dgc_file_size("/tmp/test1.dat"));

  fp = fopen("/tmp/test1.dat", "r");
  ASSERT_TRUE(fp != NULL);
  ASSERT_EQ(1, (int)fread(readback, kBufferSize, 1, fp));

  same = true;
  for(i = 0; i < kBufferSize; i++)
    if(data[i] != readback[i])
      same = false;
  ASSERT_TRUE(same);
  fclose(fp);
  system("rm /tmp/test1.dat");

  delete [] data;
  delete [] readback;
}

TEST(AsyncWriterTest, WriteWithoutOpen) {
  unsigned char data[100];
  AsyncWriter writer;

  ASSERT_EQ(-1, writer.Write(100, data));
}

TEST(AsyncWriterTest, OpenTwice) {
  AsyncWriter writer;

  ASSERT_EQ(0, writer.Open("/tmp/test1.dat")) << "Could not open file.";
  ASSERT_EQ(-1, writer.Open("/tmp/test1.dat"));
}
