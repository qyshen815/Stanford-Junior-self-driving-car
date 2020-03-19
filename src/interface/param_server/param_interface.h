#ifndef DGC_PARAM_PROCESSOR_H
#define DGC_PARAM_PROCESSOR_H

#include <ipc_interface.h>
#include <param_messages.h>
#include <commandline.h>
#include <vector>
#include <string>
#include <stdarg.h>

namespace dgc {

struct ParamChangeInfo {
  char *module;
  char *variable;
  char *value;
};

IpcCallback *ParamCB(void (*pfun)(void));

IpcCallback *ParamCB(void (*pfun)(ParamChangeInfo *));

template<class ClassType>
IpcCallback *ParamCB(ClassType *pclass, 
		     void (ClassType::*pmemfun)(ParamChangeInfo *)) {
  return new Callback<ParamChangeInfo>(pclass, pmemfun);
}

template<class ClassType>
IpcCallback *ParamCB(ClassType *pclass, void (ClassType::*pmemfun)(void)) {
  return new VoidCallback<ParamChangeInfo>(pclass, pmemfun);
}

const int kParamTimeoutMs = 5000;

typedef enum { DGC_PARAM_INT,
	       DGC_PARAM_DOUBLE,
	       DGC_PARAM_ONOFF,
	       DGC_PARAM_STRING,
	       DGC_PARAM_FILENAME,
	       DGC_PARAM_TRANSFORM } ParamType;

typedef struct {
  std::string module;                /**<The module name of this parameter. */
  std::string variable;              /**<The variable name to be loaded. */
  ParamType type          ;          /**<Type should match user_variable:
                                         e.g., DGC_PARAM_INT if the local
                                         variable storage is an integer. */
  void *user_variable;               /**<A pointer to the local variable
                                        storage. */
  int subscribe;                     /**<If the param_daemon publishes a
                                        change to this variable value (because
                                        someone has changed the variable
                                        value, should the local value be
                                        updated? 1 for yes, 0 for no. */
  IpcCallback *cb;
} Param;

struct ParamInfo {
  std::string variable;
  std::string value;
  bool expert;
};

class ParamInterface {
 public:
  ParamInterface(IpcInterface *ipc);
  ~ParamInterface();

  char *GetRobot(void);
  int GetModules(std::vector <char *> &modules);

  int GetAll(const char *module, std::vector <ParamInfo> &param);
  int GetInt(const char *module, const char *variable, int *return_value, 
	     int *expert);
  int GetDouble(const char *module, const char *variable, double *return_value,
		int *expert);
  int GetOnOff(const char *module, const char *variable, int *return_value, 
	       int *expert);
  int GetString(const char *module, const char *variable, char **return_value, 
		int *expert);
  int GetString(const char *variable, char **return_value, int *expert);
  int GetFilename(const char *module, const char *variable, char **return_value,
		  int *expert);

  int GetFile(const char *module, const char *variable, char **file);
  int GetTransform(const char *module, const char *variable, 
		   dgc_transform_t *return_value);

  int SetVariable(const char *module, const char *variable, char *new_value,
		  char **return_value);
  int SetVariable(const char *variable, char *new_value, char **return_value);
  int SetInt(const char *module, const char *variable, int new_value, 
	     int *return_value);
  int SetDouble(const char *module, const char *variable, double new_value, 
		double *return_value);
  int SetOnOff(const char *module, const char *variable, int new_value, 
	       int *return_value);
  int SetString(const char *module, const char *variable, char *new_value, 
		char **return_value);
  int SetFilename(const char *module, const char *variable, char *new_value, 
		  char **return_value);

  void SubscribeInt(const char *module, const char *variable, 
		    int *variable_address, IpcCallback *callback);
  void SubscribeDouble(const char *module, const char *variable, 
		       double *variable_address, IpcCallback *callback);
  void SubscribeOnOff(const char *module, const char *variable, 
		      int *variable_address, IpcCallback *callback);
  void SubscribeString(const char *module, const char *variable, 
		       char **variable_address, IpcCallback *callback);
  void SubscribeFilename(const char *module, const char *variable, 
			 char **variable_address, IpcCallback *callback);

  char *GetError(void);

  void SetUsage(char *fmt, ... );
  void ParamUsage(char *progname, Param *param_list, int num_items, 
		  char *s);

  void AllowUnfoundVariables(bool allow);
  void InstallParams(int argc, char **argv, Param *param_list, int num_items);

  int CheckVersion(char *prog_name);
  void RereadCommand(void);

 private:
  int GetStringInternal(const char *module, const char *variable, 
			char **return_value, int *expert, bool is_filename);
  void ParamChangeHandler(ParamVariableChange *msg);
  Param *FindParam(const char *module, const char *variable);
  void InstallParam(std::string module, std::string variable, 
		    void *use_variable, ParamType type, int subscribe, 
		    IpcCallback *cb);
  void AddChangeSubscription(void);

  std::vector <Param> param_;
  std::string usage_line_;
  bool allow_unfound_parameters_;
  CommandLineProcessor cmdline_;
  IpcInterface *ipc_;
  char error_buffer_[1024];
  int change_callback_id_;
};

}

#endif
