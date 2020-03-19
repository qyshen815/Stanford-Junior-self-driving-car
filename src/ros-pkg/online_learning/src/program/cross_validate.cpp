#include <online_learning/cross_validator.h>

using namespace std;
using namespace odontomachus;

string usageString()
{
  ostringstream oss;
  oss << "Usage: cross_validate METHOD [METHOD...] OD OUTPUT_PATH" << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  // -- Parse args.
  if (argc < 4) {
    cout << usageString() << endl;
    return 0;
  }

  string data_path = argv[argc-2];
  string output_path = argv[argc-1];

  set<string> methods;
  for (int i = 1; i < argc - 2; ++i)
    methods.insert(argv[i]);
  
  // -- Read back what is being done.
  set<string>::iterator it;
  cout << "Cross validating ";
  for (it = methods.begin(); it != methods.end(); ++it)
    cout << *it << " ";
  cout << endl;

  cout << "Using data at " << data_path << endl;
  cout << "Saving output to " << output_path << endl;

  // -- Load data.
  Dataset::Ptr data(new Dataset());
  data->load(data_path);
  cout << "Loaded dataset: " << endl;
  cout << data->status() << endl;

  // -- Run cross validation tests.
  CrossValidator cv(data, methods, output_path);
  data.reset();
  cv.run();

  return 0;
}
