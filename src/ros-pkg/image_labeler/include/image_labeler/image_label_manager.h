#ifndef IMAGE_LABEL_MANAGER_H
#define IMAGE_LABEL_MANAGER_H

#include <fstream>
#include <vector>
#include <iostream>
#include <map>
#include <assert.h>
#include <boost/filesystem.hpp>

#include <opencv2/core/core_c.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>

#include <serializable/serializable.h>

#define IMAGE_LABEL_MANAGER_SERIALIZATION_VERSION 2

/*
 * Image origin is upper left hand corner.
 * x is the column number.
 * y is the row number.
 *
 */

class Label
{
public:
  int track_id_;
  std::string class_name_;
  // x coord of the upper left corner of the bounding box.
  int x_;
  // y coord of the upper left corner of the bounding box.
  int y_;
  int width_;
  int height_;

  Label();
  Label(int track_id, const std::string& class_name, int x, int y, int width, int height);
  Label(std::istream& in);
  void serialize(std::ostream& out) const;
  void deserialize(std::istream& in);

  int minx() const;
  int maxx() const;
  int miny() const;
  int maxy() const;
};

/* One-off set of labels for an image.
 */
class LabelSet : public Serializable
{
public:
  std::vector<Label> labels_;
  
  void serialize(std::ostream& out) const;
  void deserialize(std::istream& in);
};

class ImageLabelManager
{
public:
  //! @param root_path The path which contains annotations.txt and the images dir.
  //! Will create the directory if it doesn't exist.
  ImageLabelManager(const std::string& root_path);
  //! Returns the number of labeled images.
  int size() const;
  IplImage* getRawImage(int id) const;
  IplImage* getLabeledImage(int id) const;
  std::vector<Label> getLabelsForImage(int id) const;
  //! Returns the filename (not path) of image id.
  std::string getFilenameForImage(int id) const;
  std::string getPathForImage(int id) const;
  std::string getImagesPath() const;
  int getNumberOfLabelsForClass(const std::string& class_name) const;
  
  //! Saves img in the images/ directory and updates annotations.txt.
  //! @param filename The name of the file, not relative or absolute path.
  void addLabeledImage(const std::string& filename, IplImage* img, std::vector<Label> labels);
  void setLabelsForImage(int id, const std::vector<Label>& labels);
  //! Updates annotations.txt.
  void saveAnnotations() const;

  static CvScalar getColor(const std::string& class_name);
  static void drawLabels(IplImage* img, const std::vector<Label>& labels);
  
private:
  std::string root_path_;
  std::string annotations_path_;
  std::string images_path_;
  
  std::vector<std::string> filenames_;
  std::vector< std::vector<Label> > labels_;
  std::map<std::string, size_t> map_;

  void saveAnnotations(const std::string& path) const;
  void loadAnnotations(const std::string& path);
  void serializeAnnotations(std::ostream& out) const;
  void deserializeAnnotations(std::istream& in);

};


#endif // IMAGE_LABEL_MANAGER_H
