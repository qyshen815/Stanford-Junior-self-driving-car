#ifndef DGC_PARAM_SERVER_H
#define DGC_PARAM_SERVER_H

#include <ipc_interface.h>
#include <param_interface.h>

namespace dgc {

typedef enum {
  BASIC = 0,
  EXPERT = 1,
  NOCHANGE = 2
} ParamLevel;

typedef struct {
  char *module_name;
  char *variable_name;
  char *lvalue;
  char *rvalue;
  int expert;
} IniParam;

class ParamServer {
public:
  ParamServer(IpcInterface *ipc);
  void Startup(int argc, char **argv);

private:
  int LookupName(char *full_name);
  int LookupParameter(char *module_name, char *parameter_name);
  int LookupModule(char *module_name);

  int ReadCommandline(int argc, char **argv);
  void SetParamIpc(ParamSetCommand *query);
  void RegisterIpcMessages(void);
  void AddParamsFromFile(void);
  void Usage(char *progname, char *fmt, ...);
  void Help(char *progname);

  void AddModule(char *module_name);
  int QueryNumParams(char *module_name);

  void PublishNewParam(int i);
  void SetParam(char *lvalue, char *rvalue, ParamLevel param_level);
  void GetRobot(ParamQuery *query);
  void GetModules(ParamQuery *query);
  void GetParamAll(ParamQuery *query);
  void GetParamInt(ParamQuery *query);
  void GetParamDouble(ParamQuery *query);
  void GetParamFile(ParamQuery *query);
  void GetParamFilename(ParamQuery *query);
  void GetParamOnOff(ParamQuery *query);
  void GetVersion(ParamVersionQuery *query);
  void GetParamString(ParamQuery *query);
  void RereadCommand(ParamReread *reread);

  int alphabetize_;
  char *param_filename_;
  char *selected_robot_;

  std::vector <char *> modules_;
  std::vector <IniParam> param_list_;

  IpcInterface *ipc_;
};

}

#endif
