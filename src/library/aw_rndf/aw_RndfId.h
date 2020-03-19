#ifndef RNDFID_H_
#define RNDFID_H_

#include <inttypes.h>
#include <string>
#include <algorithm>
#include <vector>
#include <set>
#include <map>
#include <stdexcept>
#include <boost/lexical_cast.hpp>

#include "aw_StringTools.h"

namespace vlr {

namespace rndf {

//! Rndf Id
typedef std::string RndfId;

//! comparing operator for RndfIds used for sorting maps, etc
struct RndfIdLess : public std::binary_function<RndfId, RndfId, bool>
{
	bool operator() (const RndfId& s1, const RndfId& s2) const
	{
		std::vector<RndfId> ids1, ids2;
		CStringTools::splitString(s1, ids1, ".");
		CStringTools::splitString(s2, ids2, ".");
		uint32_t i=0;
		while (true) {
			if (ids1.size() != ids2.size() ) {
				if (i >= ids1.size()) return true;
				if (i >= ids2.size()) return false;
			} else {
				if (i >= ids1.size()) return false;
			}
			uint32_t id1 = boost::lexical_cast<uint32_t>(ids1[i]);
			uint32_t id2 = boost::lexical_cast<uint32_t>(ids2[i]);
			if (id1 == id2) {
				++i;
				continue;
			}
			return id1 < id2;
		}
	}
};

//! extracts a single Id of the specified \a level
inline uint32_t getIdFromStr(const RndfId& strId, uint32_t level)
{
	std::vector<std::string> tokens;
	CStringTools::splitString(strId, tokens, ".");
	if (level >= tokens.size()) {
	  printf("%s: input: %s; requested level: %u, token number: %u\n", __FUNCTION__, strId.c_str(), level, (uint32_t)tokens.size());
//	  throw std::out_of_range("getIdFromStr: token count < level");
	return 4711;
	}
	return boost::lexical_cast<uint32_t>( tokens[level] );
}

template<class T>
size_t nextId(const std::map<std::string, T>& elements) {
std::vector<size_t> ids;
typename std::map<std::string, T>::const_iterator it = elements.begin(), it_end = elements.end();
for (; it != it_end; it++) {
  ids.push_back(it->second->getID());
}
std::sort(ids.begin(), ids.end());

for (size_t i = 1; i <= ids.size(); ++i) {
  if (i != ids[i - 1]) {
    return i;
  }
}

return ids.size() + 1;
}

template<class T> inline std::string nextIdStr(const std::map<std::string, T>& elements) {
  return boost::lexical_cast<std::string>(nextId(elements));
}

template<class T>
size_t nextId(const std::set<T>& elements) {
std::vector<size_t> ids;
typename std::set<T>::const_iterator it = elements.begin(), it_end = elements.end();
for (; it != it_end; it++) {
  ids.push_back((*it)->getID());
}
sort(ids.begin(), ids.end());

for (size_t i = 1; i <= ids.size(); ++i) {
  if (i != ids[i - 1]) {
    return i;
  }
}

return ids.size() + 1;
}

template<class T> inline std::string nextIdStr(const std::set<T>& elements) {
  return boost::lexical_cast<std::string>(nextId(elements));
}

template<class T>
size_t nextId(const std::vector<T>& elements) {
std::vector<size_t> ids;
typename std::vector<T>::const_iterator it = elements.begin(), it_end = elements.end();
for (; it != it_end; it++) {
  ids.push_back((*it)->getID());
}
sort(ids.begin(), ids.end());

for (size_t i = 1; i <= ids.size(); ++i) {
  if (i != ids[i - 1]) {
    return i;
  }
}

return ids.size() + 1;
}

template<class T> inline std::string nextIdStr(const std::vector<T>& elements) {
  return boost::lexical_cast<std::string>(nextId(elements));
}
}  // namespace rndf

} // namespace vlr

#endif /*RNDFID_H_*/
