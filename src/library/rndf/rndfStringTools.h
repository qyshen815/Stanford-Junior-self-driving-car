#ifndef RNDF_RNDFSTRINGTOOLS_H
#define RNDF_RNDFSTRINGTOOLS_H

#include <string>

namespace dgc {
void strip_comments(std::string* str);
void sanitize_line(std::string* str);
std::string first_word(std::string line);

} // namespace dgc

#endif // RNDF_RNDFSTRINGTOOLS_H
