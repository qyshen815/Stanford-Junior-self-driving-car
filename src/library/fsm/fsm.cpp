#include <roadrunner.h>
#include "fsm.h"

using std::vector;

fsm_t::fsm_t(int start_state)
{
  state_ = start_state;
  default_state_func_ = NULL;
}

void fsm_t::add_transition_func(int state, IntFunctor *transition_func)
{
  fsm_transition_t t;

  t.state = state;
  t.transition_func = transition_func;
  transition_.push_back(t);
}

void fsm_t::add_state_func(int state, VoidFunctor *state_func)
{
  fsm_action_t a;
  
  a.state = state;
  a.state_func = state_func;
  action_.push_back(a);
}

void fsm_t::add_default_state_func(VoidFunctor *state_func)
{
  default_state_func_ = state_func;
}

fsm_t::~fsm_t()
{
  int i;
  
  for(i = 0; i < (signed)transition_.size(); i++) 
    if(transition_[i].transition_func)
      delete transition_[i].transition_func;
  for(i = 0; i < (signed)action_.size(); i++) 
    if(action_[i].state_func)
      delete action_[i].state_func;
  if(default_state_func_)
    delete default_state_func_;
}

bool fsm_t::is_legal(int state, int next_state)
{
  int i;
  
  for(i = 0; i < (signed)legal_transition_.size(); i++)
    if(state == legal_transition_[i].state &&
       next_state == legal_transition_[i].next_state)
      return true;
  return false;
}

int fsm_t::update_state(void)
{
  int i, next_state;
  
  for(i = 0; i < (signed)transition_.size(); i++)
    if(state_ == transition_[i].state) {
      next_state = transition_[i].transition_func->Call();
      if(next_state != FSM_NO_TRANSITION) {
	if(!is_legal(state_, next_state))
	  fprintf(stderr, "ERROR: Illegal state transition!\n");
	state_ = next_state;
	return 1;
      }
    }
  return 0;
}

void fsm_t::do_action(void)
{
  int i, did_action = 0;
  
  for(i = 0; i < (signed)action_.size(); i++)
    if(state_ == action_[i].state) {
      action_[i].state_func->Call();
      did_action = 1;
    }
  if(default_state_func_ && !did_action) {
    default_state_func_->Call();
    return;
  }
}

void fsm_t::legal_transition(int state, int next_state)
{
  fsm_legal_transition_t t;

  t.state = state;
  t.next_state = next_state;
  legal_transition_.push_back(t);
}

void fsm_t::set_state_name(int state, char *str)
{
  state_name_[state] = str;
}

char *fsm_t::state_name(int state)
{
  return (char *)state_name_[state].c_str();
}

int compare_ints(const void *a, const void *b)
{
  const int *ai, *bi;

  ai = (int *)a;
  bi = (int *)b;
  
  if(*ai < *bi)
    return -1;
  else if(*ai == *bi)
    return 0;
  else
    return 1;
}

void append_line(char **data, int *num_chars, int *max_chars, char *line)
{
  int l;

  l = strlen(line);
  if(*num_chars + l + 1 > *max_chars) {
    *max_chars = *num_chars + l + 10000;
    *data = (char *)realloc(*data, *max_chars);
    dgc_test_alloc(data);
  }
  (*data)[*num_chars] = '\0';
  *num_chars += l;
  strcat(*data, line);
}

char *fsm_t::dot_data(void)
{
  char *data = NULL, *name, line[1000];
  int num_chars = 0, max_chars = 0;
  vector <int> node_list;
  int i, j, found;

  sprintf(line, "digraph FSMGraph {\n");
  append_line(&data, &num_chars, &max_chars, line);

  for(i = 0; i < (signed)legal_transition_.size(); i++) {
    found = 0;
    for(j = 0; j < (signed)node_list.size(); j++)
      if(node_list[j] == legal_transition_[i].state) {
	found = 1;
	break;
      }
    if(!found)
      node_list.push_back(legal_transition_[i].state);

    found = 0;
    for(j = 0; j < (signed)node_list.size(); j++)
      if(node_list[j] == legal_transition_[i].next_state) {
	found = 1;
	break;
      }
    if(!found)
      node_list.push_back(legal_transition_[i].next_state);
  }

  qsort(&node_list[0], (int)node_list.size(), sizeof(int), compare_ints);

  for(i = 0; i < (signed)node_list.size(); i++) {
    name = (char *)state_name_[node_list[i]].c_str(); 
    if(strlen(name) == 0)
      sprintf(line, "  \"%d\" [label=\"state %d\"];\n", node_list[i],
	     node_list[i]);
    else
      sprintf(line, "  \"%d\" [label=\"%s\"];\n", node_list[i], name);
    append_line(&data, &num_chars, &max_chars, line);
  }

  for(i = 0; i < (signed)legal_transition_.size(); i++) {
    sprintf(line, "  \"%d\" -> \"%d\";\n", 
	    legal_transition_[i].state, legal_transition_[i].next_state);
    append_line(&data, &num_chars, &max_chars, line);
  }

  sprintf(line, "}\n");
  append_line(&data, &num_chars, &max_chars, line);
  return data;
}

char *fsm_t::dot_drawing_data(void)
{
  char *input, *output;
  
  input = dot_data();
  output = dgc_run_program("/usr/bin/dot -Tplain", input, 0.1);
  free(input);
  return output;
}

void fsm_t::export_pdf(char *filename)
{
  char commandline[200];
  char *input, *output;

  input = dot_data();
  sprintf(commandline, "/usr/bin/dot -Tpdf -o %s", filename);
  output = dgc_run_program(commandline, input, 0.1);
  free(input);
  free(output);
}

void fsm_t::export_dot(char *filename)
{
  char *input;
  FILE *fp;

  input = dot_data();
  fp = fopen(filename, "w");
  if(fp == NULL)
    dgc_die("Error: could not open file %s for writing.\n", filename);
  fprintf(fp, "%s", input);
  fclose(fp);
  free(input);
}
