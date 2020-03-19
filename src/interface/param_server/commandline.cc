#include <roadrunner.h>
#include <commandline.h>
#include <string>

using std::string;

namespace dgc {

CommandLineProcessor::CommandLineProcessor(char *error_buffer)
{
  error_buffer_ = error_buffer;
}

int CommandLineProcessor::Read(int argc, char **argv)
{
  int i, last_commandline_arg = argc;
  char *tmp_arg1;
  ini_param p;

  if (argc == 1)
    return 1;

  param_.clear();

  for (i = 1; i < last_commandline_arg; i++) {
    if (argv[i][0] != '-') {
      if (i < last_commandline_arg && i + 1 < argc) {
	tmp_arg1 = argv[i];
	memmove(argv + i, argv + i + 1, (argc - i - 1) * sizeof(char *));
	argv[argc - 1] = tmp_arg1;
	i--;
      }
      last_commandline_arg--;
      continue;
    }
    
    /* If it's the last argument, or the next argument is -<letter>,
       then this obviously is not a param pair. 
       
       Note that we test isalpha. and not isalphnum, so that we can pass 
       -<digit> to the the param pair code below. */

    if (i + 1 == argc || 
	(strlen(argv[i + 1]) > 1 && argv[i + 1][0] == '-' && 
	 isalpha(argv[i + 1][1]))) {
      /* single parameter */
      if (strcmp(argv[i], "-v") == 0) {
	dgc_carp_verbose = 1; 
      } else {
	p.lvalue = argv[i] + 1;
	p.rvalue = "";
	param_.push_back(p);
      }          
    } else {
      /* parameter pair */
      p.lvalue = argv[i] + 1;
      p.rvalue = argv[i + 1];
      param_.push_back(p);
      i++;
    }
  }
  return last_commandline_arg;
}

bool CommandLineProcessor::FindParam(string lvalue)
{
  unsigned int i;
  
  for (i = 0; i < param_.size(); i++)
    if (param_[i].lvalue == lvalue)
      return true;
  return false;
}

const char *CommandLineProcessor::ParamPair(string lvalue)
{
  unsigned int i;

  for (i = 0; i < param_.size(); i++)
    if (param_[i].lvalue == lvalue)
      return param_[i].rvalue.c_str();
  return NULL;
}

bool CommandLineProcessor::FindParamPair(string lvalue)
{
  return FindParam(lvalue) && ParamPair(lvalue) != NULL;
}

FindParamResult CommandLineProcessor::Find(string lvalue)
{
  unsigned int i;

  for (i = 0; i < param_.size(); i++)
    if (param_[i].lvalue == lvalue) {
      if (param_[i].rvalue.empty())
	return FOUND_SINGLE;
      else
	return FOUND_PAIR;
    }
  return NOT_FOUND;
}

int CommandLineProcessor::CheckIntParam(const char *lvalue, int *return_value)
{
  const char *arg;
  char *endptr;

  arg = ParamPair(lvalue);
  if (arg == NULL) {
    return 0;
  } else if (strlen(arg) == 0) {
    sprintf(error_buffer_, "Option '%s' requires argument: should be an "
            "integer.", lvalue);
    return -1;
  } 

  *return_value = strtol(arg, &endptr, 0);
  if (*endptr != '\0') {
    sprintf(error_buffer_, "Bad argument to %s: %s, should be an integer.", 
            lvalue, arg);
    return -1;
  }
  return 1;
}

int CommandLineProcessor::CheckDoubleParam(const char *lvalue, 
					   double *return_value)
{
  const char *arg;
  char *endptr;

  arg = ParamPair(lvalue);
  if (arg == NULL) {
    return 0;
  } else if (strlen(arg) == 0) {
    sprintf(error_buffer_, "Option '%s' requires argument: should be a "
            "double.", lvalue);
    return -1;
  } 

  *return_value = strtod(arg, &endptr);
  if (*endptr == '\0') {
    sprintf(error_buffer_, "Bad argument to %s: %s, should be a double.", 
            lvalue, arg);
    return -1;
  }
  return 1;
}

int CommandLineProcessor::CheckOnOffParam(const char *lvalue, int *return_value)
{
  const char *arg;

  arg = ParamPair(lvalue);
  if (arg == NULL) {
    return 0;
  } else if (strlen(arg) == 0) {
    sprintf(error_buffer_, "Option '%s' requires argument: should be "
            "on/off.", lvalue);
    return -1;
  } 

  if (strcmp(arg, "on") == 0) {
    *return_value = 1;
    return 1;
  } else if (strcmp(arg, "off") == 0) {
    *return_value = 0;
    return 1;
  } else {
    sprintf(error_buffer_, "Bad argument to %s: %s, should be on/off.", 
            lvalue, arg);
    return -1;
  }
}

int CommandLineProcessor::CheckStringParam(const char *lvalue, 
					   char **return_value)
{
  const char *arg;
  
  arg = ParamPair(lvalue);
  if (arg == NULL) {
    return 0;
  } else if (strlen(arg) == 0) {
    sprintf(error_buffer_, "Option '%s' requires argument: should be a "
            "string.", lvalue);
    return -1;
  } 
  *return_value = const_cast<char *>(arg);
  return 1;
}

int CommandLineProcessor::CheckFilenameParam(const char *lvalue, 
					     char **filename)
{
  const char *arg;

  arg = ParamPair(lvalue);
  if (arg == NULL) {
    return 0;
  } else if (strlen(arg) == 0) {
    sprintf(error_buffer_, "Option '%s' requires argument: should be a "
            "filename.", lvalue);
    return -1;
  } 
  *filename = const_cast<char *>(arg);
  return 1;
}

int CommandLineProcessor::CheckFileParam(const char *lvalue, char **file)
{
  struct stat buf;
  const char *arg;
  FILE *fp;

  arg = ParamPair(lvalue);
  if (arg == NULL) {
    return 0;
  } else if (strlen(arg) == 0) {
    sprintf(error_buffer_, "Option '%s' requires argument: should be a "
            "file.", lvalue);
    return -1;
  } 

  if (stat(arg, &buf) < 0) {
    sprintf(error_buffer_, "Could not stat file %s.", arg);
    return -1;
  }
  *file = (char *)calloc(buf.st_size + 1, 1);
  dgc_test_alloc(*file);

  fp = fopen(arg, "r");
  if (fp == NULL) {
    sprintf(error_buffer_, "Could not open file %s.", arg);
    return -1;
  }

  if(fread(*file, buf.st_size, 1, fp) != 1) {
    sprintf(error_buffer_, "Could not read from file %s.", arg);
    fclose(fp);
    return -1;
  }
  (*file)[buf.st_size] = '\0';
  fclose(fp);

  return 1;
}

int CommandLineProcessor::CheckTransformParam(const char *lvalue, 
					      dgc_transform_t *t)
{
  const char *arg;
  int err;

  arg = ParamPair(lvalue);
  if (arg == NULL) {
    return 0;
  } else if (strlen(arg) == 0) {
    sprintf(error_buffer_, "Option '%s' requires argument: should be a "
            "transform filename.", lvalue);
    return -1;
  } 
  err = dgc_transform_read(*t, const_cast<char *>(arg));
  if (err < 0) {
    sprintf(error_buffer_, "Could not read transform file %s.", arg);
    return -1;
  }
  return 1;
}

}

