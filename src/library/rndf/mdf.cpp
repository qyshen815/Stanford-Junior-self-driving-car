#include <roadrunner.h>
#include <lltransform.h>
#include <iostream>
#include <fstream>
#include "rndf.h"
#include "mdf.h"

#include "rndfStringTools.h"

using std::string;

namespace dgc {

typedef enum {
  MDF_START,
  MDF_NAME,
  RNDF,
  OPTIONAL_MDF_HEADER,
  CHECKPOINTS,
  NUM_CHECKPOINTS,
  CHECKPOINT_ID,
  END_CHECKPOINTS,
  SPEED_LIMITS,
  NUM_SPEED_LIMITS,
  SPEED_LIMIT_ID,
  END_SPEED_LIMITS,
  END_MDF,
} mdf_state_t, *mdf_state_p;

char *mdf_state_name[] = {
  "MDF_START",
  "MDF_NAME",
  "RNDF",
  "OPTIONAL_MDF_HEADER",
  "CHECKPOINTS",
  "NUM_CHECKPOINTS",
  "CHECKPOINT_ID",
  "END_CHECKPOINTS",
  "SPEED_LIMITS", 
  "NUM_SPEED_LIMITS", 
  "SPEED_LIMIT_ID",
  "END_SPEED_LIMITS",
  "END_MDF",
};

typedef struct {
  mdf_state_t state1;
  string command;
  mdf_state_t state2;
} legal_mdf_command_t, *legal_mdf_command_p;

legal_mdf_command_t legal_mdf_command[] = {
  {MDF_START, "MDF_name", MDF_NAME},
  {MDF_NAME, "RNDF", RNDF},
  {RNDF, "format_version", OPTIONAL_MDF_HEADER},
  {RNDF, "creation_date", OPTIONAL_MDF_HEADER},
  {OPTIONAL_MDF_HEADER, "format_version", OPTIONAL_MDF_HEADER},
  {OPTIONAL_MDF_HEADER, "creation_date", OPTIONAL_MDF_HEADER},
  {OPTIONAL_MDF_HEADER, "checkpoints", CHECKPOINTS},
  {CHECKPOINTS, "num_checkpoints", NUM_CHECKPOINTS},
  {NUM_CHECKPOINTS, "digit", CHECKPOINT_ID},
  {CHECKPOINT_ID, "digit", CHECKPOINT_ID},
  {CHECKPOINT_ID, "end_checkpoints", END_CHECKPOINTS},
  {END_CHECKPOINTS, "speed_limits", SPEED_LIMITS},
  {SPEED_LIMITS, "num_speed_limits", NUM_SPEED_LIMITS},
  {NUM_SPEED_LIMITS, "digit", SPEED_LIMIT_ID},
  {SPEED_LIMIT_ID, "digit", SPEED_LIMIT_ID},
  {SPEED_LIMIT_ID, "end_speed_limits", END_SPEED_LIMITS},
  {END_SPEED_LIMITS, "end_file", END_MDF},
};

int valid_mdf_checkpoint_command(string str)
{
  unsigned int i;

  for(i = 0; i < str.length(); i++)
    if(!isdigit(str[i]))
      return 0;
  return 1;
}

int valid_mdf_command(mdf_state_p state, string command)
{
  unsigned int i;

  for(i = 0; i < sizeof(legal_mdf_command) / 
        sizeof(legal_mdf_command_t); i++)
    if(legal_mdf_command[i].state1 == *state &&
       ((legal_mdf_command[i].command == command) ||
        (legal_mdf_command[i].command == "digit" &&
         valid_mdf_checkpoint_command(command)))) {
      *state = legal_mdf_command[i].state2;
      return 1;
    }
  return 0;
}

int mdf_file::load(char *filename)
{
  std::ifstream infile;
  string line, command, arguments;
  int i, has_text;
  string::size_type mark;
  mdf_state_t state = MDF_START;
  int current_limit = 0, current_checkpoint = 0;

  /* open file for reading */
  infile.open(filename);
  if(!infile.is_open()) {
    fprintf(stderr, "Could not open file %s for reading.\n", filename);
    return -1;
  }

  /* read the file, line by line */
  while(getline(infile, line, '\n')) {
    /* strip out comments and other annoying stuff */
    strip_comments(&line);
    sanitize_line(&line);

    /* look for blank lines */
    has_text = 0;
    for(i = 0; i < (signed)line.length(); i++)
      if(!isblank(line[i]))
        has_text = 1;
    if(!has_text)
      continue;

    /* extract command and arguments */
    mark = line.find(" ", 0);
    if(mark == string::npos) {
      command = line;
      arguments = "";
    }
    else {
      command = line.substr(0, mark);
      arguments = line.substr(mark);
    }

    /* make sure the command is valid, and if so, update the current        
       file state */
    if(!valid_mdf_command(&state, command))
      dgc_die("Command %s not legal from state %s\n", command.c_str(),
              mdf_state_name[state]);
      
    if(command == "MDF_name") 
      mdf_file::filename_ = first_word(arguments);
    else if(command == "RNDF")
      rndf_filename_ = first_word(arguments);
    else if(command == "format_version")
      format_version_ = first_word(arguments);
    else if(command == "creation_date")
      creation_date_ = first_word(arguments);
    else if(command == "num_checkpoints") {
      int num_checkpoints;
      sscanf(arguments.c_str(), "%d", &num_checkpoints);
      goal_.resize(num_checkpoints);
    }
    else if(command == "num_speed_limits") {
      int num_speed_limits;
      sscanf(arguments.c_str(), "%d", &num_speed_limits);
      speed_limit_.resize(num_speed_limits);
    }
    else if(state == CHECKPOINT_ID) {
      int id;
      sscanf(command.c_str(), "%d", &id);
      if(current_checkpoint >= num_goals())
        dgc_die("Error: mdf_file::mdf_file : Illegal MDF command.\n");
      goal_[current_checkpoint] = id;
      current_checkpoint++;
    }
    else if(state == SPEED_LIMIT_ID) {
      mdf_speed_limit *limit = new mdf_speed_limit;
      sscanf(command.c_str(), "%d", &limit->id);
      limit->id--;
      sscanf(arguments.c_str(), "%f %f", &limit->min_speed, 
             &limit->max_speed);
      limit->min_speed = dgc_mph2ms(limit->min_speed);
      limit->max_speed = dgc_mph2ms(limit->max_speed);
      if(current_limit >= num_speed_limits())
        dgc_die("Error: mdf_file::mdf_file : Illegal MDF command.\n");
      speed_limit_[current_limit] = limit;
      current_limit++;
    }
    else if(command == "end_speed_limits") {
      if(current_limit != num_speed_limits())
	dgc_die("Error: mdf_file::load : Incorrect number of speed limits specified.\n");
    }
    else if(command == "end_checkpoints") {
      if(current_checkpoint != num_goals())
	dgc_die("Error: mdf_file::load : Incorrect number of checkpoints specified.\n");
    }
    else if(command == "checkpoints" ||
            command == "speed_limits" ||
            command == "end_file") {
      /* don't need to parse these commands */
    }
    else {
      dgc_die("Error: could not parse command \"%s\"\n", command.c_str());
    }
  }
  infile.close();
  return 0;
}

int mdf_file::save(char *filename) const
{
  FILE *fp;
  int i;

  fp = fopen(filename, "w");
  if(fp == NULL) {
    fprintf(stderr, "Could not open file %s for writing.\n", filename);
    return -1;
  }
  
  fprintf(fp, "MDF_name\t%s\n", this->filename_.c_str());
  fprintf(fp, "RNDF\t%s\n", rndf_filename_.c_str());
  if(format_version_.size() > 0)
    fprintf(fp, "format_version\t%s\n", format_version_.c_str());
  if(creation_date_.size() > 0)
    fprintf(fp, "creation_date\t%s\n", creation_date_.c_str());
  fprintf(fp, "checkpoints\n");
  fprintf(fp, "num_checkpoints\t%d\n", num_goals());
  for(i = 0; i < num_goals(); i++)
    fprintf(fp, "%d\n", goal_[i] + 1);
  fprintf(fp, "end_checkpoints\n");
  fprintf(fp, "speed_limits\n");
  fprintf(fp, "num_speed_limits\t%d\n", num_speed_limits());
  for(i = 0; i < num_speed_limits(); i++)
    fprintf(fp, "%d\t%d\t%d\n", speed_limit(i)->id + 1, 
            (int)rint(dgc_ms2mph(speed_limit(i)->min_speed)),
            (int)rint(dgc_ms2mph(speed_limit(i)->max_speed)));
  fprintf(fp, "end_speed_limits\n");
  fprintf(fp, "end_file\n");
  fclose(fp);
  return 0;
}

void mdf_print(mdf_file *mdf)
{
  int i;

  fprintf(stderr, "MDF name : *%s*\n", mdf->filename().c_str());
  fprintf(stderr, "RNDF name : *%s*\n", mdf->rndf_filename().c_str());
  fprintf(stderr, "RNDF format verison : *%s*\n", 
          mdf->format_version().c_str());
  fprintf(stderr, "RNDF creation date : *%s*\n", mdf->creation_date().c_str());
  fprintf(stderr, "  Num checkpoints : %d\n", mdf->num_goals());
  for(i = 0; i < mdf->num_goals(); i++)
    fprintf(stderr, "    %d : %d\n", i + 1, mdf->goal(i));
  fprintf(stderr, "  Num speed limits : %d\n", mdf->num_speed_limits());
  for(i = 0; i < mdf->num_speed_limits(); i++)
    fprintf(stderr, "    %d : %d %.1f mph %.1f mph\n", i + 1, 
            mdf->speed_limit(i)->id + 1, 
            dgc_ms2mph(mdf->speed_limit(i)->min_speed),
            dgc_ms2mph(mdf->speed_limit(i)->max_speed));
}

}

