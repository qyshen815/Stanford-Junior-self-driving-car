#include <roadrunner.h>
#include "paramread.h"
#include <vector>

using std::vector;

#define       MAX_LVALUE_LENGTH           254
#define       MAX_VARIABLE_LENGTH         2048

int FindValidRobots(char *param_filename, vector <char *> &robot_names)
{
  char *err, *line, *mark, *left, *right, *name;
  FILE *fp = NULL;
  int count = 0;

  fp = fopen(param_filename, "r");      
  if(fp == NULL)
    return -1;

  line = (char *)calloc(MAX_VARIABLE_LENGTH, sizeof(char));
  dgc_test_alloc(line);

  do {
    err = fgets(line, MAX_VARIABLE_LENGTH - 1, fp);
    line[MAX_VARIABLE_LENGTH - 1] = '\0';
    if(strlen(line) == MAX_VARIABLE_LENGTH - 1) 
      dgc_die("Line %d of file %s is too long.\n"
	      "Maximum line length is %d. Please correct this line.\n\n"
	      "It is also possible that this file has become corrupted.\n"
	      "Make sure you have an up-to-date version of dgc, and\n"
	      "consult the param_server documentation to make sure the\n"
	      "file format is valid.\n", count, param_filename, 
	      MAX_VARIABLE_LENGTH - 1);
    count++;
    if(err != NULL) {
      mark = strchr(line, '#');    /* strip comments and trailing returns */
      if(mark != NULL)
        mark[0] = '\0';
      mark = strchr(line, '\n');
      if(mark != NULL)
        mark[0] = '\0';
      
      left = strchr(line, '[');
      right = strchr(line, ']');
      if(left != NULL && right != NULL && left < right && 
         left[1] != '*' && dgc_strncasecmp(left + 1, "expert", 6) != 0) {
        name = (char *)calloc(right - left, 1);
        dgc_test_alloc(name);
        strncpy(name, left + 1, right - left - 1);
        name[right - left - 1] = '\0';
	robot_names.push_back(name);
      }
    }
  } while(err != NULL);
  free(line);
  return 0;
}

int ReadParamsFromFile(char *param_filename, char *selected_robot,
		       vector <ParamInput> &param_list)
{
  int token_num, found_matching_robot = 0, found_desired_robot = 0;
  char lvalue[MAX_LVALUE_LENGTH + 1], rvalue[MAX_VARIABLE_LENGTH];
  int expert = 0, line_length, count;
  char *line, *mark, *token;
  ParamInput p;
  FILE *fp = NULL;

  fp = fopen(param_filename, "r");      
  if(fp == NULL)
    return -1;

  line = (char *)calloc(MAX_VARIABLE_LENGTH, sizeof(char));
  dgc_test_alloc(line);

  count = 0;
  while (!feof(fp)) {
    if(fgets(line, MAX_VARIABLE_LENGTH-1, fp) != line)
      if(!feof(fp))
        dgc_die("fgets error: Unable to read line %d of file %s.",
            count, param_filename);
    line[MAX_VARIABLE_LENGTH - 1] = '\0';
    if (strlen(line) == MAX_VARIABLE_LENGTH - 1) 
      dgc_die("Line %d of file %s is too long.\n"
	      "Maximum line length is %d. Please correct this line.\n\n"
	      "It is also possible that this file has become corrupted.\n"
	      "Make sure you have an up-to-date version of dgc, and\n"
	      "consult the param_server documentation to make sure the\n"
	      "file format is valid.\n", count, param_filename, 
	      MAX_VARIABLE_LENGTH - 1);
    count++;
    if (feof(fp))
      break;
    mark = strchr(line, '#');    /* strip comments and trailing returns */
    if (mark != NULL)
      mark[0] = '\0';
    mark = strchr(line, '\n');
    if (mark != NULL)
      mark[0] = '\0';
    
    // Trim off trailing white space 
    
    line_length = strlen(line) - 1;
    while (line_length >= 0 && 
           (line[line_length] == ' ' || line[line_length] == '\t' )) {
      line[line_length--] = '\0';
    }
    line_length++;
    
    if (line_length == 0)
      continue;
    
    // Skip over initial blank space
    
    mark = line + strspn(line, " \t");
    if (strlen(mark) == 0) 
      dgc_die("You have encountered a bug in dgc. Please report it\n"
	      "to the dgc maintainers. \n"
	      "Line %d, function %s, file %s\n", __LINE__, __FUNCTION__,
                 __FILE__);
    
    token_num = 0;
    
    /* tokenize line */
    token = mark;
    
    // Move mark to the first whitespace character.
    mark = strpbrk(mark, " \t");
    // If we found a whitespace character, then turn it into a NULL
    // and move mark to the next non-whitespace.
    if (mark) {
      mark[0] = '\0';
      mark++;
      mark += strspn(mark, " \t");
    }
    
    if (strlen(token) > MAX_LVALUE_LENGTH) {
      dgc_warning("Bad file format of %s on line %d.\n"
		  "The parameter name %s is too long (%d characters).\n"
		  "A parameter name can be no longer than %d "
		  "characters.\nSkipping this line.\n", param_filename, 
		  count, token, (int) strlen(token), MAX_LVALUE_LENGTH);
      continue;
    }
    
    strcpy(lvalue, token);
    token_num++;
    
    // If mark points to a non-whitespace character, then we have a
    // two-token line
    if (mark) {
      if (strlen(mark) > MAX_VARIABLE_LENGTH-1) {
        dgc_warning("Bad file format of %s on line %d.\n"
		    "The parameter value %s is too long (%d "
		    "characters).\nA parameter value can be no longer "
		    "than %u characters.\nSkipping this line.\n", 
		    param_filename, count, mark, (int) strlen(mark),
		    MAX_VARIABLE_LENGTH-1);
        continue;
      }
      strcpy(rvalue, mark);
      token_num++;
    }
    
    if (lvalue[0] == '[') {
      if (strcspn(lvalue+1,"]") == 6 && 
	  !dgc_strncasecmp(lvalue+1, "expert", 6)) {
        found_matching_robot = 1;
        expert = 1;
      }
      else {
        expert = 0;
        if (lvalue[1] == '*')
          found_matching_robot = 1;
        else if (selected_robot) {
          if (strlen(lvalue) < strlen(selected_robot) + 2)
            found_matching_robot = 0;
          else if (lvalue[strlen(selected_robot)+1] != ']') 
            found_matching_robot = 0;
          else if (dgc_strncasecmp
                   (lvalue+1, selected_robot, strlen(selected_robot)) == 0) {
            found_matching_robot = 1;
            found_desired_robot = 1;
          } else
            found_matching_robot = 0;
        }
      }
    }
    else if(token_num == 2 && found_matching_robot == 1) {
      p.lvalue = strdup(lvalue);
      p.rvalue = strdup(rvalue);
      p.expert = expert;
      param_list.push_back(p);
    }
  }
  
  fclose(fp);

  if (selected_robot && !found_desired_robot) 
    dgc_die("Did not find a match for robot %s. Do you have the right\n"
	    "init file? (Reading parameters from %s)\n", selected_robot,
	    param_filename);
  return 0;
}

void FreeParamInputs(vector <ParamInput> &param_list)
{
  unsigned int i;

  for (i = 0; i < param_list.size(); i++) {
    if (param_list[i].lvalue != NULL)
      free(param_list[i].lvalue);
    if (param_list[i].rvalue != NULL)
      free(param_list[i].rvalue);
  }
  param_list.clear();
}
