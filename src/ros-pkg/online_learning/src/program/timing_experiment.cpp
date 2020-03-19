#include <online_learning/ssl.h>

using namespace std;
using namespace odontomachus;

string usageString()
{
  ostringstream oss;
  oss << "Usage: ssl METHOD OUTPUT_DIR THRESH --seed SEED [SEED ..] --unlabeled UNL [UNL ..]" << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc < 8) {
    cout << usageString() << endl;
    return 0;
  }

  string method = argv[1];
  ROS_ASSERT(method.compare("naive") == 0);
  vector<string> args;
  for(int i = 2; i < argc; ++i)
    args.push_back(argv[i]);
  SSLParams ssl_params;
  ssl_params.parse(args);
  cout << ssl_params << endl;

  SSL ssl(ssl_params);
  ssl.run();
  
  return 0;
}
