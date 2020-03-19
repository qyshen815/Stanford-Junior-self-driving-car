#ifndef DGC_PARAMREAD_H
#define DGC_PARAMREAD_H

#include <vector>

typedef struct {
  char *lvalue, *rvalue;
  int expert;
} ParamInput;

int FindValidRobots(char *param_filename, std::vector <char *> &robot_names);

int ReadParamsFromFile(char *param_filename, char *selected_robot,
		       std::vector <ParamInput> &param_list);

void FreeParamInputs(std::vector <ParamInput> &param_list);

#endif
