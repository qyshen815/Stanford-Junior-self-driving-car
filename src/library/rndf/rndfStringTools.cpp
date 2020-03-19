#include <string>

#include <global.h>

namespace dgc {

using namespace std;

void strip_comments(std::string *str)
{
  std::string::size_type pos1, pos2;
  int found;

  do {
    found = 0;
    pos1 = str->find("/*", 0);
    if(pos1 != string::npos) {
      pos2 = str->find("*/", pos1);
      if(pos2 != string::npos) {
        str->erase(pos1, pos2 - pos1 + 2);
        found = 1;
      }
    }
  } while(found);
}

void sanitize_line(std::string *str)
{
  unsigned int i;
  int mark;

  /* chop leading spaces */
  mark = 0;
  while((*str)[mark] != '\0' && isspace((*str)[mark]))
    mark++;
  if((*str)[mark] != '\0')
    str->erase(0, mark);

  /* chop trailing spaces */
  mark = str->length() - 1;
  while(mark >= 0 && isspace((*str)[mark]))
    mark--;
  str->erase(mark + 1, str->length() - (mark + 1));

  /* turn tabs into spaces */
  for(i = 0; i < str->length(); i++)
    if((*str)[i] == '\t')
      (*str)[i] = ' ';
}

std::string first_word(std::string line)
{
  char *tempstring;
  std::string response;

  tempstring = (char *)calloc(line.length() + 1, 1);
  dgc_test_alloc(tempstring);
  sscanf(line.c_str(), "%s", tempstring);
  response = tempstring;
  free(tempstring);
  return response;
}

} // namespace dgc
