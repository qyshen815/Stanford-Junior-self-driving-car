#include <gtest/gtest.h>
#include <image_labeler/image_label_manager.h>

using namespace std;
namespace bfs = boost::filesystem;

TEST(ImageLabelManager, label_serialization)
{
  string filename = "example_label";
  
  Label label(0, "car", 42, 37, 200, 100);
  cout << "Original: " << endl;
  label.serialize(cout);
  cout << endl;

  assert(!bfs::exists(filename));
  
  ofstream ofile(filename.c_str());
  label.serialize(ofile);
  ofile.close();

  ifstream ifile(filename.c_str());
  Label deserialized(ifile);
  ifile.close();

  cout << "Saved and loaded: " << endl;
  deserialized.serialize(cout);

  EXPECT_TRUE(deserialized.track_id_ == label.track_id_);
  EXPECT_TRUE(deserialized.x_ == label.x_);
  EXPECT_TRUE(deserialized.y_ == label.y_);
  EXPECT_TRUE(deserialized.width_ == label.width_);
  EXPECT_TRUE(deserialized.height_ == label.height_);
  EXPECT_TRUE(deserialized.class_name_.compare(label.class_name_) == 0);

  bfs::remove(filename);
}


TEST(ImageLabelManager, load_real_data)
{
  ImageLabelManager dataset("example_run");
  cout << dataset.size() << endl;
  EXPECT_TRUE(dataset.size() == 3);

  for(int i = 0; i < dataset.size(); ++i) {
    cout << dataset.getPathForImage(i) << " - " << dataset.getFilenameForImage(i) << endl;
  }

  IplImage* raw = dataset.getRawImage(1);
  IplImage* labeled = dataset.getLabeledImage(1);
  cvShowImage("raw", raw);
  cvShowImage("labeled", labeled);
  cvWaitKey();
  cvReleaseImage(&raw);
  cvReleaseImage(&labeled);  
  
  vector<Label> labels = dataset.getLabelsForImage(1);
  for(size_t i = 0; i < labels.size(); ++i)
    labels[i].serialize(cout);

  //-1 car 298 820 173 61
  EXPECT_TRUE(labels[3].track_id_ == -1);
  EXPECT_TRUE(labels[3].class_name_.compare("car") == 0);
  EXPECT_TRUE(labels[3].y_ == 820);
  EXPECT_TRUE(labels[3].width_ == 173);

  labels[1].class_name_ = "bicyclist";
  dataset.setLabelsForImage(1, labels);
  IplImage* labeled2 = dataset.getLabeledImage(1);
  cvShowImage("labeled2", labeled2);
  cvWaitKey();
  cvReleaseImage(&labeled2);  

}

TEST(OpenCV, jpg_and_video)
{
  ImageLabelManager dataset("example_run");
  
  IplImage* img = dataset.getRawImage(1);
  try {
    cvSaveImage("opencv_test.jpg", img);
    cout << "OpenCV jpg write succeeded." << endl;
  }
  catch(...) {
    cout << "OpenCV jpg write failed." << endl;
  }

  CvVideoWriter* writer = cvCreateVideoWriter("opencv_test.avi",
					      CV_FOURCC('I', '4', '2', '0'),
					      10, cvSize(640, 480), true);
  if(!writer)
    cout << "OpenCV video write failed." << endl;
  else
    cout << "OpenCV video write succeeded." << endl;
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

