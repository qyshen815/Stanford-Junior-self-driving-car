/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Alex Teichman
*********************************************************************/
#include <multibooster/multibooster.h>
#include <signal.h>

using namespace std;
using boost::shared_ptr;
using namespace Eigen;
#define EIGEN_NO_DEBUG


MultiBoosterDataset::MultiBoosterDataset(vector<string> classes, vector<string> feature_spaces) :
  version_string_(DATASET_VERSION),
  class_map_(classes),
  feature_map_(feature_spaces)
{
}

MultiBoosterDataset::MultiBoosterDataset(const NameMapping& class_map, const NameMapping& descriptor_map) :
  version_string_(DATASET_VERSION),
  class_map_(class_map),
  feature_map_(descriptor_map)
{
}

MultiBoosterDataset::MultiBoosterDataset(string filename) :
  version_string_(DATASET_VERSION),
  class_map_(vector<string>()),
  feature_map_(vector<string>())
{
  if(!load(filename)) {
    cerr << "Unable to load MultiBoosterDataset " << filename << endl;
    throw 1;
  }
}

MultiBoosterDataset::MultiBoosterDataset(const MultiBoosterDataset& mbd) :
  version_string_(mbd.version_string_),
  class_map_(mbd.class_map_),
  feature_map_(mbd.feature_map_)
{
  // -- Copy in the objects.
  for(unsigned int i=0; i<mbd.objs_.size(); i++) {
    objs_.push_back(new Object(*(mbd.objs_[i])));
  }
  setObjs(objs_);
}

void MultiBoosterDataset::applyNewMappings(const NameMapping& new_class_map, const NameMapping& new_feature_map) {

  // -- Make sure these mappings don't delete things.
  for(size_t i=0; i<class_map_.size(); ++i) {
    new_class_map.toId(class_map_.toName(i)); //This will error if all current names don't exist in the new map.
  }
  for(size_t i=0; i<feature_map_.size(); ++i) {
    new_feature_map.toId(feature_map_.toName(i)); //This will error if all current names don't exist in the new map.
  }

  // -- Do the translation for objs.
  NameTranslator class_translator(class_map_, new_class_map);
  NameTranslator fs_translator(feature_map_, new_feature_map);

  for(unsigned int i=0; i<objs_.size(); i++) {
    objs_[i]->translate(class_translator, fs_translator); // Background objects keep their label of -1.  The class map only has positive classes.
  }

  class_map_ = new_class_map;
  feature_map_ = new_feature_map;
  
  // -- Update ymc_, num_objs_of_class_ as well.
  setObjs(objs_);
}
  

void MultiBoosterDataset::decimate(float decimation) {
  assert(decimation > 0.0 && decimation < 1.0);
  random_shuffle(objs_.begin(), objs_.end());
  crop(decimation);
}

void MultiBoosterDataset::crop(float cropval) {
  assert(cropval > 0.0 && cropval < 1.0);
  size_t max_obj_idx = cropval * objs_.size();
  for(size_t m=max_obj_idx+1; m<objs_.size(); ++m)
    delete objs_[m];
  objs_.resize(max_obj_idx+1);
  setObjs(objs_);
}

void MultiBoosterDataset::dropFeature(string drop) {
  vector<string> features = feature_map_.getIdToNameMapping();
  vector<string>::iterator fit = find(features.begin(), features.end(), drop);
  assert(fit != features.end());
  
  size_t idx = feature_map_.toId(drop);
  for(size_t m=0; m<objs_.size(); ++m) {
    objs_[m]->descriptors_.erase(objs_[m]->descriptors_.begin() + idx);
  }

  features.erase(fit);
  feature_map_ = NameMapping(features);
}

void MultiBoosterDataset::dropFeatures(vector<string> droplist) {
  for(size_t i=0; i<droplist.size(); ++i) {
    vector<string> features = feature_map_.getIdToNameMapping();
    if(find(features.begin(), features.end(), droplist[i]) == features.end()) {
      cerr << "There is no feature " << droplist[i] << " in this dataset." << endl;
    }
    else {
      dropFeature(droplist[i]);
    }
  }
}

bool MultiBoosterDataset::join(const MultiBoosterDataset& mbd2)
{

  // -- Augment 
  NameMapping tmp_class_map_ = class_map_;
  tmp_class_map_.augment(mbd2.class_map_);
  NameMapping tmp_feature_map_ = feature_map_;
  tmp_feature_map_.augment(mbd2.feature_map_);
  applyNewMappings(tmp_class_map_, tmp_feature_map_);
  
  // -- Copy in mbd2's objects, translating to our names.
  NameTranslator class_translator(mbd2.class_map_, class_map_);
  NameTranslator fs_translator(mbd2.feature_map_, feature_map_);
  const vector<Object*>& objs2 = mbd2.objs_;
  for(unsigned int i=0; i<objs2.size(); i++) {
    objs_.push_back(new Object(*(objs2[i])));
    objs_.back()->translate(class_translator, fs_translator);
  }

  // -- Recompute ymc_ and num_objs_of_class_. 
  setObjs(objs_);
  return true;
}

bool MultiBoosterDataset::compare(const MultiBoosterDataset& mbd) {
  if(mbd.version_string_.compare(version_string_) != 0)
    return false;
  if(mbd.class_map_.compare(class_map_) == false)
    return false;
  if(mbd.feature_map_.compare(feature_map_) == false)
    return false;
  if(mbd.ymc_ != ymc_)
    return false;
  if(mbd.num_bg_ != num_bg_)
    return false;
  if(objs_.size() != mbd.objs_.size())
    return false;
  if(mbd.num_objs_of_class_.size() != num_objs_of_class_.size())
    return false;

  for(size_t i=0; i<num_objs_of_class_.size(); ++i) {
    if(mbd.num_objs_of_class_[i] != num_objs_of_class_[i])
      return false;
  }

  for(size_t i=0; i<objs_.size(); ++i)
    if(!objs_[i]->equals(*mbd.objs_[i])) {
      return false;
    }
    
  return true;
}


void MultiBoosterDataset::setObjs(const vector<Object*> &objs)
{
  objs_ = objs;
  num_objs_of_class_.clear();
  num_objs_of_class_.resize(class_map_.size(), 0);
  num_bg_ = 0;
  
  // -- Get num_objs_of_class_.
  for(unsigned int m=0; m<objs.size(); m++) {
    if(objs[m]->label_ == -1)
      num_bg_++;
    else if(objs[m]->label_ == -2)
      continue;
    else {
      assert(objs[m]->label_ >= 0);
      num_objs_of_class_[(size_t)objs[m]->label_]++;
    }
  }

  // -- Construct ymc_
  ymc_ = MatrixXf::Zero(class_map_.size(), objs.size()); 
  for(unsigned int m=0; m<objs.size(); m++) {
    for(unsigned int c=0; c<class_map_.size(); c++) {
      //If the label is -1 (background), make all y_m^c's be -1.
      if(objs_[m]->label_ == -2) {
	ymc_(c,m) = 0;
      }
      else if(objs_[m]->label_ == (int)c) 
	ymc_(c,m) = 1;
      else
	ymc_(c,m) = -1;
    }
  }

  // -- Make sure every object has just one class.
  for(int m=0; m<ymc_.cols(); ++m) {
    bool has_one = false;
    bool valid = true;
    for(int c=0; c<ymc_.rows(); ++c) {
      if((int)ymc_(c,m) == 1 && has_one) {
	valid = false;
	break;
      }
      if((int)ymc_(c,m) == 1 && !has_one)
	has_one = true;
    }
    if(!valid) { 
      cout << "col " << m << ": " <<  ymc_.col(m).transpose() << endl;
      assert(0);
    }
  }
}

string MultiBoosterDataset::displayYmc() {
  ostringstream oss (ostringstream::out);
  oss << "ymc_: " << endl << ymc_ << endl;
  return oss.str();
}

string MultiBoosterDataset::displayObjects() {
  ostringstream oss (ostringstream::out);
  for(unsigned int i=0; i<objs_.size(); i++) {
    oss << "Object " << i << " " << endl;
    oss << objs_[i]->status(class_map_, feature_map_);
  }
  return oss.str();
}


/************************************************************
 * Object
 ************************************************************/

Object::Object(const Object& o)
{
  label_ = o.label_;
  
  descriptors_.resize(o.descriptors_.size());
  for(size_t i=0; i<o.descriptors_.size(); ++i) {
    if(o.descriptors_[i].vector) { 
      descriptors_[i].vector = new Eigen::VectorXf(*o.descriptors_[i].vector);
      descriptors_[i].length_squared = o.descriptors_[i].length_squared;
    }
    else {
      descriptors_[i].vector = NULL;
      descriptors_[i].length_squared = 0;
    }
  }
}

Object::Object(istream& in)
{
  deserialize(in);
}

void Object::deserialize(istream& in)
{
  in.read((char*)&label_, sizeof(int));
  int num_descriptors;
  in.read((char*)&num_descriptors, sizeof(size_t));
    
  descriptors_ = vector<descriptor>(num_descriptors);
  for(int i = 0; i < num_descriptors; ++i) {
    bool valid;
    in.read((char*)&valid, sizeof(bool));
    if(!valid)
      descriptors_[i].vector = NULL;
    else {
      descriptors_[i].vector = new VectorXf();
      eigen_extensions::deserialize(in, descriptors_[i].vector);
      descriptors_[i].length_squared = descriptors_[i].vector->squaredNorm();
    }
  }
}

void Object::serialize(ostream& out) const
{
  out.write((char*)&label_, sizeof(int));
  size_t num_descriptors = descriptors_.size();
  out.write((char*)&num_descriptors, sizeof(size_t));
  for(size_t d=0; d<descriptors_.size(); ++d) {
    VectorXf* v = descriptors_[d].vector;
    // If there's no descriptor for this descriptor space, we need to save a placeholder
    // or the descriptor mapping is violated.
    bool valid = v;
    if(v) assert(valid);
    if(!v) assert(!valid);
    
    out.write((char*)&valid, sizeof(bool));
    if(valid)
      eigen_extensions::serialize(*v, out);
  }
}

string Object::status(const NameMapping& class_map, const NameMapping& feature_map, bool showDescriptors) {

  ostringstream oss (ostringstream::out);
  oss << "Object with label " << label_;
  if(label_ == -1)
    oss << " (background) :" << endl;
  else if(label_ == -2)
    oss << " (unlabeled) :" << endl;
  else
    oss << " (" << class_map.toName(label_) << ") :" << endl;
  for(size_t i=0; i<descriptors_.size(); ++i) {
    VectorXf* v = descriptors_[i].vector;
    oss << i << ": " << feature_map.toName(i) << " feature space ";
    if(v) {
      oss << "(" << v->rows() << " dimensions) " << endl;
      if(showDescriptors) {
	oss << v->transpose() << endl;
      }
    }
    else
      oss << " -- no descriptor for this object." << endl;

  }
  return oss.str();
}

void Object::translate(const NameTranslator& class_translator, const NameTranslator& fs_translator) {
  if(label_ > -1) // -1 (bg) and -2 (unlabeled) should keep their labels.   
    label_ = class_translator.toMap2(label_);
  descriptor filler;
  filler.vector = NULL;
  filler.length_squared = 0;
  vector<descriptor> desc2(fs_translator.size(), filler);
  for(size_t i=0; i<descriptors_.size(); ++i) {
    desc2[fs_translator.toMap2(i)] = descriptors_[i];
  }
  descriptors_ = desc2;
}

bool Object::equals(const Object& other) const
{
  if(label_ != other.label_)
    return false;
  if(descriptors_.size() != other.descriptors_.size())
    return false;

  for(size_t j=0; j<descriptors_.size(); ++j) {
    if(!other.descriptors_[j].vector && !descriptors_[j].vector) // Move on if both don't have this descriptor.
      continue;

    if((!other.descriptors_[j].vector && descriptors_[j].vector) ||
       (other.descriptors_[j].vector && !descriptors_[j].vector))
      return false;

    if(other.descriptors_[j].length_squared != descriptors_[j].length_squared)
      return false;
    if(other.descriptors_[j].vector->rows() != descriptors_[j].vector->rows())
      return false;
    if(*other.descriptors_[j].vector != *descriptors_[j].vector)
      return false;
  }
  return true;
}

uint64_t Object::numBytes() const
{
  uint64_t num_bytes = sizeof(int);
  for(size_t i = 0; i < descriptors_.size(); ++i) {
    if(descriptors_[i].vector) 
      num_bytes += descriptors_[i].vector->rows() * sizeof(float);
  }
  return num_bytes;
}


std::string MultiBoosterDataset::status()
{
  ostringstream oss (ostringstream::out);

  oss << "MultiBoosterDataset status: \n";
  if(objs_.size() < 1) {
    oss << "No objects!" << "\n";
    return oss.str();
  }

  size_t num_unlabeled = 0;
  for(size_t i=0; i<objs_.size(); ++i) {
    if(objs_[i]->label_ == -2)
      num_unlabeled++;
  }
  
  oss << "  nClasses: " << class_map_.size() << "\n";
  oss << "  nObjects: " << objs_.size() << "\n";
  oss << "  nDescriptors: " << feature_map_.size() << "\n";
  oss << "  nBackground: " << num_bg_ << "\n";
  oss << "  nUnlabeled: " << num_unlabeled << "\n";
  oss << "  num_objs_of_class_: " << "\n";
  for(size_t i=0; i<num_objs_of_class_.size(); ++i) {
    oss << "    " << class_map_.toName(i) << ": " << num_objs_of_class_[i] << " objects.\n";
  }

  vector<int> nPts(feature_map_.size(), 0);
  vector<int> dim(feature_map_.size(), -1);
  for(unsigned int m=0; m<objs_.size(); m++) {
    Object& obj = *objs_[m];
    for(size_t d=0; d<obj.descriptors_.size(); ++d) {
      descriptor& desc = obj.descriptors_[d];
      if(desc.vector)
	nPts[d]++;
      if(desc.vector && desc.vector->rows() != 0)
	dim[d] = desc.vector->rows();
    }
  }

 
  oss << "  nPts: " << endl;
  for(size_t i=0; i<nPts.size(); ++i) {
    oss << "    nPts in " << feature_map_.toName(i) << " (";
    if(dim[i] == -1)
      oss << "unknown";
    else
      oss << dim[i];
    oss << "-dimensional) space: " << nPts[i] << "\n";
  }
  
  // -- Show example descriptors.
//   for(fit = objs_[0]->descriptors_.begin(); fit != objs_[0]->descriptors_.end(); fit++) {
//     cout << "   " << fit->first << ": " << endl << fit->second->transpose() << endl;
//   }


  return oss.str();
}

bool MultiBoosterDataset::save(string filename)
{
  ofstream f;
  f.open(filename.c_str());
  if(f.fail()) {
    cerr << "Failed to open file " << filename << endl;
    return false;
  }

  f << version_string_ << endl;
  f << class_map_.serialize() << endl;
  f << feature_map_.serialize() << endl;
  size_t num_objs = objs_.size();
  f.write((char*)&num_objs, sizeof(size_t));
  for(unsigned int i=0; i<objs_.size(); i++) {
    objs_[i]->serialize(f);
  }
  f.close();
  return true;
}

bool MultiBoosterDataset::load(string filename, bool quiet) {
  ifstream f;
  f.open(filename.c_str());
  if(f.fail()) {
    cerr << "Failed to open file " << filename << endl;
    return false;
  }

  string line;
  getline(f, line);
  if(line.compare(version_string_) != 0) {
    cerr << "Log " << filename << " is of the wrong type!" << endl;
    return false;
  }

  class_map_ = NameMapping(f);
  feature_map_ = NameMapping(f);
  getline(f, line); // Eat the space after the name mapping.  This really needs to be fixed.

  size_t num_objs;
  f.read((char*)&num_objs, sizeof(size_t));
  vector<Object*> objs(num_objs, NULL);
  for(size_t i = 0; i < objs.size(); ++i)
    objs[i] = new Object(f);
  f.close();
  
  for(size_t i = 0; i < objs.size(); ++i)
    assert(objs[i]);

  setObjs(objs);
  return true;
}

MultiBoosterDataset::~MultiBoosterDataset() {
  for(size_t i=0; i<objs_.size(); ++i) { 
    delete objs_[i];
  }
}

shared_ptr< vector< pair<float, int> > > MultiBoosterDataset::computeSortedDistances(size_t fsid, VectorXf center) {
  float center_length_squared = center.dot(center);
  shared_ptr< vector< pair<float, int> > > distance_idx(new vector< pair<float, int> >(0));
  distance_idx->reserve(objs_.size());
  
  for(size_t j=0; j<objs_.size(); ++j) {
    VectorXf* f = objs_[j]->descriptors_[fsid].vector;
    if(!f) //Ignore the objects that don't have this descriptor.
      continue;
    
    float dist = fastEucSquared(*f, center, objs_[j]->descriptors_[fsid].length_squared, center_length_squared);
    distance_idx->push_back(pair<float, int>(dist, j));
  }
  sort(distance_idx->begin(), distance_idx->end()); //Ascending sort.

  return distance_idx;
}
