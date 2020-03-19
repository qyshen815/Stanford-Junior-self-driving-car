#include <name_mapping2/name_mapping2.h>

using namespace std;

NameMapping2::NameMapping2() :
  offset_(0)
{
}

NameMapping2::NameMapping2(const NameMapping2& other) :
  offset_(other.offset_),
  names_(other.names_),
  indices_(other.indices_)
{
} 

NameMapping2& NameMapping2::operator=(const NameMapping2& other)
{
  names_ = other.names_;
  indices_ = other.indices_;
  offset_ = other.offset_;

  return *this;
}

void NameMapping2::assertSpecialNameEquality(const NameMapping2& other) const
{
  ROS_FATAL_STREAM_COND(offset_ != other.offset_, "Special names in the two NameMapping2s must be the same.");
  for(int i = -offset_; i < 0; ++i)
    ROS_FATAL_STREAM_COND(toName(i).compare(other.toName(i)) != 0, "Special names in the two NameMapping2s must be the same.");
}

NameMapping2& NameMapping2::operator+=(const NameMapping2& other)
{
  assertSpecialNameEquality(other);
  
  for(size_t i = 0; i < other.names_.size(); ++i) {
    if(hasName(other.names_[i]))
      continue;

    addName(other.names_[i]);
  }

  return *this;
}

NameMapping2 NameMapping2::operator+(const NameMapping2& other)
{
  NameMapping2 new_mapping = *this;
  new_mapping += other;
  return new_mapping;
}

void NameMapping2::addName(const string& name)
{
  // No spaces.
  if(name.find(" ") != string::npos)
    ROS_FATAL_STREAM("Space found in NameMapping2 entry " << name << ", dying horribly.");

   // No duplicates.
  if(indices_.find(name) != indices_.end())
    ROS_FATAL_STREAM("Duplicate name " << name << " added to NameMapping2, dying horribly.");

  indices_[name] = names_.size();
  names_.push_back(name);
}

void NameMapping2::addNames(const vector<string>& names)
{
  for(size_t i = 0; i < names.size(); ++i)
    addName(names[i]);
}

void NameMapping2::setOffset(int offset)
{
  ROS_FATAL_STREAM_COND(offset < 0, "NameMapping2 offset must be >= 0");
  offset_ = offset;
}

int NameMapping2::getOffset() const
{
  return offset_;
}

int NameMapping2::minId() const
{
  return -offset_;
}

int NameMapping2::maxId() const
{
  return (int)size() - offset_ - 1;
}

bool NameMapping2::hasName(std::string name) const
{
  return indices_.count(name);
}

bool NameMapping2::hasId(int id) const
{
  int idx = id + offset_;
  if(idx >= 0 && idx < (int)size())
    return true;
  else
    return false;
}

void NameMapping2::augment(const NameMapping2 & other) {
  assertSpecialNameEquality(other);

  set<string> uniq;
  for(size_t i = 0; i < names_.size(); ++i)
    uniq.insert(names_[i]);

  for(size_t i = 0; i < other.size() - other.getOffset(); ++i) { 
    if(uniq.count(other.toName(i)) == 0)
      addName(other.toName(i));
  }
}

string NameMapping2::toName(int id) const {
  int idx = id + offset_;
  ROS_FATAL_STREAM_COND(idx < 0 || (size_t)idx >= names_.size(), "Attempted to access non-existent id " << id << " in NameMapping2: " << endl << status());

  return names_[idx];
}

int NameMapping2::toId(string name) const {
  map<string, size_t>::const_iterator it;
  it = indices_.find(name);
  ROS_FATAL_STREAM_COND(it == indices_.end(), "Could not find name " << name << " in the following NameMapping2: " << endl << status());

  return it->second - offset_;
}

#define SERIALIZATION_VERSION 1
void NameMapping2::serialize(std::ostream& out) const
{
  out << "NameMapping2" << endl;
  out << "SERIALIZATION_VERSION " << SERIALIZATION_VERSION << endl;
  out << "offset " << offset_ << endl;
  out << "num_names " << names_.size() << endl;
  for(size_t i=0; i<names_.size(); ++i)
    out << names_[i] << endl;
}

void NameMapping2::deserialize(std::istream& in)
{
  string buf;
  in >> buf;
  in >> buf;
  int serialization_version;
  in >> serialization_version;
  if(serialization_version != SERIALIZATION_VERSION)
    ROS_FATAL_STREAM("NameMapping2 serialization version is " << serialization_version << ", expected " << SERIALIZATION_VERSION);

  in >> buf;
  in >> offset_;
  
  in >> buf;
  size_t num_names;
  in >> num_names;

  for(size_t i = 0; i < num_names; ++i) { 
    in >> buf;
    addName(buf);
  }

  getline(in, buf); // Eat the final newline.
}

bool NameMapping2::operator==(const NameMapping2& other) const
{
  ROS_DEBUG("Comparing NameMapping2s.");
  
  if(size() != other.size()) {
    ROS_DEBUG("Size did not match.");
    return false;
  }

  if(offset_ != other.offset_) {
    ROS_DEBUG("Offset did not match.");
    return false;
  }
  
  for(size_t i = 0; i < names_.size(); ++i) {
    if(!other.hasName(names_[i])) { 
      ROS_DEBUG_STREAM("Other does not have name: " << names_[i] << ".");
      return false;
    }
    
    int idx = other.toId(names_[i]) + other.getOffset();
    ROS_ASSERT(idx >= 0);
    if(i != (size_t)(idx)) {
      ROS_DEBUG_STREAM("Name mismatch: " << i);
      return false;
    }
  }

  for(size_t i = 0; i < other.size(); ++i) {
    int id = (int)i - other.getOffset();
    string name = other.toName(id);
    if(!hasName(name)) {
      ROS_DEBUG_STREAM("*this does not have name: " << name << ".");
      return false;
    }

    if(id != toId(name)) {
      ROS_DEBUG_STREAM("Name mismatch: " << i);
      return false;
    }
  }
		       
  
  return true;
}

bool NameMapping2::operator!=(const NameMapping2& other) const
{
  return !(*this == other);
}

std::string NameMapping2::diff(const NameMapping2& other) const
{
  vector<string> here_but_not_there;
  vector<string> there_but_not_here;
  diff(other, &here_but_not_there, &there_but_not_here);

  ostringstream oss;
  oss << "here_but_not_there: " << endl;
  for(size_t i = 0; i < here_but_not_there.size(); ++i)
    oss << here_but_not_there[i] << endl;
  oss << "there_but_not_here: " << endl;
  for(size_t i = 0; i < there_but_not_here.size(); ++i)
    oss << there_but_not_here[i] << endl;

  return oss.str();
}

void NameMapping2::diff(const NameMapping2& other,
		       vector<string>* here_but_not_there,
		       vector<string>* there_but_not_here) const
{
  ROS_ASSERT(there_but_not_here->empty());
  ROS_ASSERT(here_but_not_there->empty());
  
  for(size_t i = 0; i < names_.size(); ++i) {
    if(other.indices_.find(names_[i]) == other.indices_.end())
      here_but_not_there->push_back(names_[i]);
  }

  for(size_t i = 0; i < other.names_.size(); ++i) {
    if(indices_.find(other.names_[i]) == indices_.end())
      there_but_not_here->push_back(other.names_[i]);
  }
}

size_t NameMapping2::numSpecialLabels() const
{
  return offset_;
}

size_t NameMapping2::numCommonLabels() const
{
  return size() - numSpecialLabels();
}

size_t NameMapping2::size() const
{
  return names_.size();
}

bool NameMapping2::empty() const
{
  return names_.empty();
}

std::string NameMapping2::status() const
{
  ostringstream oss;
  oss << "NameMapping2" << endl;
  oss << "Offset: " << offset_ << endl;
  for(size_t i = 0; i < names_.size(); ++i)
    oss << (int)i - offset_ << ": " << names_[i] << endl;
  return oss.str();
}

std::vector<std::string> NameMapping2::getCommonNames() const
{
  vector<string> names;
  for(size_t i = offset_; i < names_.size(); ++i)
    names.push_back(names_[i]);
  return names;
}


/***********************************************************
 * NameTranslator2
 ************************************************************/

NameTranslator2::NameTranslator2(const NameMapping2& old_mapping, const NameMapping2& new_mapping) :
  old_mapping_(old_mapping),
  new_mapping_(new_mapping)
{
  initialize();
}

std::string NameTranslator2::status() const
{
  ostringstream oss;
  oss << "NameTranslator2" << endl;
  int min_id = min(old_mapping_.minId(), new_mapping_.minId());
  int max_id = max(old_mapping_.maxId(), new_mapping_.maxId());
		  
  for(int i = min_id; i <= max_id; ++i) {
    oss << i << " ";
    if(old_mapping_.hasId(i))
      oss << old_mapping_.toName(i);
    else
      oss << "\t";
    oss << "\t";
    if(new_mapping_.hasId(i))
      oss << new_mapping_.toName(i);
    else
      oss << "\t";
    oss << endl;
  }

  oss << "Translation:" << endl;
  for(int i = 0; i < (int)old_to_new_.size(); ++i) {
    oss << i - offset_ << " <-> ";
    if(oldToNewId(i - offset_) != NO_ID)
      oss << oldToNewId(i - offset_);
    oss << endl;
  }

  return oss.str();
}

void NameTranslator2::initialize()
{
  old_mapping_.assertSpecialNameEquality(new_mapping_);
  offset_ = new_mapping_.offset_;
  
  old_to_new_.resize(old_mapping_.size());
  for(int i = old_mapping_.minId(); i <= old_mapping_.maxId(); ++i) {
    int idx = i + offset_;

    string old_name = old_mapping_.toName(i);
    if(!new_mapping_.hasName(old_name))
      old_to_new_[idx] = NO_ID;
    else
      old_to_new_[idx] = new_mapping_.indices_[old_name];
  }
}

int NameTranslator2::oldToNewIdx(int idx) const
{
  if(old_to_new_[idx] == NO_ID)
    return NO_ID;
  else
    return old_to_new_[idx] - offset_;
}

int NameTranslator2::oldToNewId(int id) const
{
  return oldToNewIdx(id + offset_);
}

size_t NameTranslator2::newSize() const
{
  return new_mapping_.size();
}

size_t NameTranslator2::oldSize() const
{
  return old_mapping_.size();
}


/************************************************************
 * NameMappable
 ************************************************************/

void NameMappable::applyNameMapping(const NameMapping2& old_mapping,
				    const NameMapping2& new_mapping,
				    int id)
{
  if(old_mapping == new_mapping)
    return;
  NameTranslator2 translator(old_mapping, new_mapping);
  applyNameMapping(translator, new_mapping, id);
}


