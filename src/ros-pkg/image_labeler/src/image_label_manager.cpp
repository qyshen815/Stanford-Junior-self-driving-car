#include <image_labeler/image_label_manager.h>

using namespace std;
namespace bfs = boost::filesystem;

/************************************************************
 * Label
 ************************************************************/

Label::Label() :
  track_id_(-1),
  x_(-1),
  y_(-1),
  width_(-1),
  height_(-1)
{
}
 
Label::Label(int track_id, const std::string& class_name, int x, int y, int width, int height) :
  track_id_(track_id),
  class_name_(class_name),
  x_(x),
  y_(y),
  width_(width),
  height_(height)
{
}

Label::Label(istream& in)
{
  deserialize(in);
}

void Label::serialize(ostream& out) const
{
  out << track_id_ << " ";
  out << class_name_ << " ";
  out << x_ << " ";
  out << y_ << " ";
  out << width_ << " ";
  out << height_ << endl;
}

void Label::deserialize(istream& in)
{
  in >> track_id_;
  in >> class_name_;
  in >> x_;
  in >> y_;
  in >> width_;
  in >> height_;
}

int Label::minx() const
{
  return x_;
}

int Label::maxx() const
{
  return x_ + width_;
}

int Label::miny() const
{
  return y_;
}

int Label::maxy() const
{
  return y_ + height_;
}


/************************************************************
 * LabelSet
 ************************************************************/

// Label& LabelSet::operator[](size_t i)
// {
//   return labels_[i];
// }

// size_t LabelSet::size() const
// {
//   return labels_.size();
// }

void LabelSet::serialize(std::ostream& out) const
{
  out << labels_.size() << endl;
  for(size_t i = 0; i < labels_.size(); ++i)
    labels_[i].serialize(out);
}

void LabelSet::deserialize(std::istream& in)
{
  size_t num_labels;
  in >> num_labels;
  labels_.resize(num_labels);
  for(size_t i = 0; i < labels_.size(); ++i)
    labels_[i].deserialize(in);
}


/************************************************************
 * ImageLabelManager
 ************************************************************/

ImageLabelManager::ImageLabelManager(const string& root_path) :
  root_path_(root_path),
  annotations_path_(root_path_ + "/annotations.txt"),
  images_path_(root_path_ + "/images")
{
  // -- Create the directory structure if it doesn't exist.
  if(!bfs::exists(root_path_)) {
    bfs::create_directory(root_path_);
    saveAnnotations(annotations_path_);
    bfs::create_directory(getImagesPath());
  }

  loadAnnotations(annotations_path_);
}

int ImageLabelManager::size() const
{
  assert(filenames_.size() == labels_.size());
  assert(map_.size() == filenames_.size());
  
  return filenames_.size();
}

IplImage* ImageLabelManager::getRawImage(int id) const
{
  IplImage* img = cvLoadImage(getPathForImage(id).c_str());
  if(!img) {
    cout << "Failed while attempting to load " << getPathForImage(id) << endl;
    assert(img);
  }
  return img;
}

IplImage* ImageLabelManager::getLabeledImage(int id) const
{
  IplImage* img = getRawImage(id);
  drawLabels(img, getLabelsForImage(id));
  return img;
}

std::vector<Label> ImageLabelManager::getLabelsForImage(int id) const
{
  assert(id >= 0 && id < size());
  return labels_[id];
}

std::string ImageLabelManager::getFilenameForImage(int id) const
{
  assert(id >= 0 && id < size());
  return filenames_[id];
}

std::string ImageLabelManager::getPathForImage(int id) const
{
  assert(id >= 0 && id < size());
  return getImagesPath() + "/" + getFilenameForImage(id);
}

std::string ImageLabelManager::getImagesPath() const
{
  return images_path_;
}

int ImageLabelManager::getNumberOfLabelsForClass(const std::string& class_name) const
{
  int count = 0;
  for(size_t i = 0; i < labels_.size(); ++i) {
    for(size_t j = 0; j < labels_[i].size(); ++j) {
      if(labels_[i][j].class_name_.compare(class_name) == 0)
	++count;
    }
  }
  return count;
}

void ImageLabelManager::addLabeledImage(const std::string& filename,
					IplImage* img,
					std::vector<Label> labels)
{
  assert(img);
  cvSaveImage((getImagesPath() + "/" + filename).c_str(), img);

  filenames_.push_back(filename);
  labels_.push_back(labels);
  map_[filename] = size() - 1;

  saveAnnotations();
}

void ImageLabelManager::setLabelsForImage(int id,
					  const std::vector<Label>& labels)
{
  assert(id >= 0 && id < size());
  labels_[id] = labels;
}

void ImageLabelManager::saveAnnotations() const
{
  saveAnnotations(annotations_path_);
}

void ImageLabelManager::saveAnnotations(const std::string& filename) const
{
  ofstream file(filename.c_str());
  assert(file.is_open());
  serializeAnnotations(file);
  file.close();
}

void ImageLabelManager::loadAnnotations(const std::string& path)
{
  ifstream file(path.c_str());
  assert(file.is_open());
  deserializeAnnotations(file);
  file.close();
}

void ImageLabelManager::serializeAnnotations(ostream& out) const
{
  out << "ImageLabelManager" << endl;
  out << "Version " << IMAGE_LABEL_MANAGER_SERIALIZATION_VERSION << endl;

  for(int i = 0; i < size(); ++i) { 
    out << endl;
    out << filenames_[i] << endl;
    out << labels_[i].size() << endl;
    for(size_t j = 0; j < labels_[i].size(); ++j)
      labels_[i][j].serialize(out);
  }
}

void ImageLabelManager::deserializeAnnotations(istream& in)
{
  assert(filenames_.empty());
  assert(map_.empty());
  assert(labels_.empty());
  
  string ignore;
  in >> ignore;
  in >> ignore;

  int version;
  in >> version;
  assert(version == IMAGE_LABEL_MANAGER_SERIALIZATION_VERSION);

  while(true) {
    string line;
    getline(in, line);
    
    string filename;
    in >> filename;
    int num_labels;
    in >> num_labels;
    
    if(in.eof())
      break;

    vector<Label> labels;
    for(int i = 0; i < num_labels; ++i) { 
      labels.push_back(Label(in));
    }

    filenames_.push_back(filename);
    labels_.push_back(labels);
    map_[filename] = size() - 1;
  }
}

void ImageLabelManager::drawLabels(IplImage* img, const std::vector<Label>& labels)
{
  for(size_t i = 0; i < labels.size(); ++i) {
    Label const& l = labels[i];
    cvRectangle(img,
		cvPoint(l.x_, l.y_),
		cvPoint(l.x_ + l.width_, l.y_ + l.height_),
		getColor(l.class_name_), 3);
  }
}

CvScalar ImageLabelManager::getColor(const std::string& class_name)
{
  CvScalar color;
  if(class_name.compare("pedestrian") == 0)
    color = cvScalar(255, 0, 0);
  else if(class_name.compare("car") == 0)
    color = cvScalar(0, 0, 255);
  else if(class_name.compare("bicyclist") == 0)
    color = cvScalar(0, 255, 0);
  else
    color = cvScalar(127, 127, 127);

  return color;
}
  
