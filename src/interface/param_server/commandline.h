#ifndef DGC_COMMANDLINE_H
#define DGC_COMMANDLINE_H

#include <transform.h>
#include <vector>
#include <string>

namespace dgc {

typedef enum {
  NOT_FOUND,
  FOUND_SINGLE,
  FOUND_PAIR
} FindParamResult;

typedef struct {
  std::string lvalue;
  std::string rvalue;
} ini_param;

class CommandLineProcessor {
 public:
  CommandLineProcessor(char *error_buffer);

  int Read(int argc, char **argv);
  bool FindParam(std::string lvalue);
  const char *ParamPair(std::string lvalue);
  bool FindParamPair(std::string lvalue);
  FindParamResult Find(std::string lvalue);

  int CheckIntParam(const char *lvalue, int *return_value);
  int CheckDoubleParam(const char *lvalue, double *return_value);
  int CheckOnOffParam(const char *lvalue, int *return_value);
  int CheckStringParam(const char *lvalue, char **string);
  int CheckFilenameParam(const char *lvalue, char **filename);
  int CheckFileParam(const char *lvalue, char **file);
  int CheckTransformParam(const char *lvalue, dgc_transform_t *t);

 private:
  std::vector<ini_param> param_;
  char *error_buffer_;
};

}

#endif
