#include <image_labeler/image_labeler.h>


using namespace std;


string usageString()
{
  ostringstream oss;
  oss << "Usage: image_labeler DATASET_DIR" << endl;
  oss << "         DATASET_DIR must have a directory structure like that used by ImageLabelManager." << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc != 2) {
    cout << usageString() << endl;
    return 1;
  }

  ImageLabelManager dataset(argv[1]);
  OpenCVView view("Image Labeler");
  ImageLabelerController controller(&dataset, &view);
  view.setDelegate(&controller);

  controller.run();
  return 0;
}
  
  
