#ifndef NAME_MAPPING2_H
#define NAME_MAPPING2_H

#include <assert.h>
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <set>
#include <sstream>
#include <limits.h>

#include <ros/console.h>
#include <ros/assert.h>

#include <serializable/serializable.h>


class NameMapping2 : public Serializable
{
public:
  //! Default constructor: empty NameMapping2.
  NameMapping2();
  NameMapping2(const NameMapping2& other);
  void addName(const std::string& name);
  void addNames(const std::vector<std::string>& names);
  //! Special labels must be identical.
  void augment(const NameMapping2& other);

  std::string toName(int id) const;
  int toId(std::string name) const;

  //! id + offset = idx in vector.
  //! idx \in \N.
  //! offset must be positive.
  //! TODO: Make this setNumSpecialLabels
  void setOffset(int offset);
  int getOffset() const;
  int minId() const;
  int maxId() const;
  bool hasName(std::string name) const;
  bool hasId(int id) const;
  //! numSpecialLabels() + numCommonLabels().
  size_t size() const;
  size_t numSpecialLabels() const;
  size_t numCommonLabels() const;
  //! Includes special labels.
  bool empty() const;
  std::string status() const;
  std::vector<std::string> getCommonNames() const;

  void serialize(std::ostream& out) const;
  void deserialize(std::istream& in);
  
  void diff(const NameMapping2& other,
	    std::vector<std::string>* here_but_not_there,
	    std::vector<std::string>* there_but_not_here) const;
  std::string diff(const NameMapping2& other) const;
  //! Returns true if two this NameMapping2 is equal to other.  (Permutations are different.)
  bool operator==(const NameMapping2& other) const;
  bool operator!=(const NameMapping2& other) const;
  NameMapping2& operator=(const NameMapping2& other);
  NameMapping2& operator+=(const NameMapping2& other);
  NameMapping2 operator+(const NameMapping2& other);
  //! Asserts that all special names (i.e. those with id < 0) are equal.
  void assertSpecialNameEquality(const NameMapping2& other) const;
  
private:
  int offset_;
  std::vector<std::string> names_;
  std::map<std::string, size_t> indices_;

  friend class NameTranslator2;
};

class NameTranslator2
{
public:
  static const int NO_ID = INT_MIN;
  
  NameTranslator2(const NameMapping2& old_mapping, const NameMapping2& new_mapping);
  //! Returns the id (i.e. respects the offset).
  int oldToNewId(int id) const;
  //! Returns the index into names_ (i.e. ignores the offset).
  int oldToNewIdx(int idx) const;
  size_t newSize() const;
  size_t oldSize() const;
  std::string status() const;
      
private:
  NameMapping2 old_mapping_;
  NameMapping2 new_mapping_;
  int offset_;
  //! old_to_new_[i] is the index of the ith name in old_mapping_ (not the name with id i).
  std::vector<int> old_to_new_;

  void initialize();
};

class NameMappable
{
public:
  virtual ~NameMappable() {};
  virtual void applyNameMapping(const NameMapping2& new_mapping,	int id = 0) = 0;
  //! id is to indicate which NameMapping2, if there are multiple in the NameMappable object.
  virtual void applyNameMapping(const NameTranslator2& translator,
				const NameMapping2& new_mapping,
				int id = 0) = 0;
  //! Convenience method which creates a new NameTranslator2 and calls the other applyNameMapping.
  //! Useful when implementing the applyNameMapping which just takes a single new mapping.
  void applyNameMapping(const NameMapping2& old_mapping,
			const NameMapping2& new_mapping,
			int id = 0);
};

#endif // NAME_MAPPING2_H
