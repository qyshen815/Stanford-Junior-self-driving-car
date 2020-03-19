#ifndef NAME_MAPPING_H
#define NAME_MAPPING_H

#include <assert.h>
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <sstream>

class NameMapping {
public:
  //! Default constructor: empty NameMapping.
  NameMapping();
  NameMapping(std::vector<std::string> idToName);
  NameMapping(std::istream& is);
  NameMapping(const NameMapping& nm);
  void addName(std::string name);
  void augment(const NameMapping & m1);
  std::string toName(size_t id) const;
  size_t toId(std::string name) const;
  std::vector<std::string> getIdToNameMapping() const;
  std::map<std::string, size_t> getNameToIdMapping() const;
  size_t getSerializationVersion() const;
  std::string serialize() const;
  //! Returns true if two this NameMapping is equal to m2.  (Permutations are different.)
  bool compare(const NameMapping & m2) const;
  //! Checks if m2 has the same names, regardless of order.
  bool isPermutation(const NameMapping& m2) const;
  //! Returns a string that reports how this NameMapping is different than m2.
  void diff(const NameMapping& m2, std::vector<std::string>* not_there, std::vector<std::string>* not_here) const;
  size_t size() const;
  
private:
  size_t serialization_version_;
  std::vector<std::string> idToName_;
  std::map<std::string, size_t> nameToId_;
};

class NameTranslator {
public:
  //! @param m1 The original mapping.
  //! @param m2 The destination mapping.
  NameTranslator(NameMapping m1, NameMapping m2);
  size_t toMap1(size_t id) const;
  size_t toMap2(size_t id) const;
  size_t size() const;
  
private:
  std::vector<size_t> id1ToId2_;
  std::vector<size_t> id2ToId1_;
};

#endif
