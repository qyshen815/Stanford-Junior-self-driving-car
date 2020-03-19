#ifndef DGC_NEWLOGIO_H
#define DGC_NEWLOGIO_H

#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <global.h>
#include <vector>
#include <ipc_interface.h>

namespace dgc {

typedef char *(*LogConverterFunc)(char *, void *);

typedef struct {
  char *logger_message_name;
  IpcMessageID messageID;
  //  char *output_string;
  LogConverterFunc conv_func;
  void *message_data;
  int interpreted;
} LogReaderCallback;

class LogReaderCallbackList {
 public:
  void AddCallback(char *logger_message_name, const IpcMessageID id, 
		   LogConverterFunc converter, int message_size, 
		   int interpreted);
  void DefineIpcMessages(IpcInterface *ipc);
  LogReaderCallback *FindCallback(char *name);

 private:
  std::vector <LogReaderCallback> callback_;
};

class LineBuffer {
 public:
  LineBuffer();
  ~LineBuffer();
  char *ReadLine(dgc_FILE *fp);
  void Reset(void);

 private:
  inline char *FindExistingLine(void);

  char *buffer_;
  int num_bytes_, max_bytes_;
  int current_position_;
};

off64_t LogfileUncompressedLength(dgc_FILE *infile);

class LogfileIndex {
 public:
  LogfileIndex();
  void IndexFile(dgc_FILE *infile);

  int Load(char *logfile_name);
  void Save(char *logfile_name);
  float PercentRead(void);
  long int MessageLength(int message_num);
  off64_t Offset(int message_num);
  int num_messages();
  inline int numMessages() {return num_messages_;}

 private:
  int num_messages_;
  off64_t *offset_;
  long int current_position_;
  off64_t total_bytes_;
  int corrupted_;
};
 
#define READ_FLOAT(pos) strtof(*(pos), (pos))
#define READ_DOUBLE(pos) strtod(*(pos), (pos))
#define READ_UINT(pos) strtoul(*(pos), (pos), 10)
#define READ_INT(pos) strtol(*(pos), (pos), 10)
#define READ_HOST(host, pos) { if(*(*(pos)) == ' ') (*pos)++; \
                               if(*(*(pos)) == ' ') strcpy(host,"unknown"); \
                               else sscanf(*(pos), "%s", host); \
                               *(pos) = dgc_next_word(*(pos)); }

}

#endif

