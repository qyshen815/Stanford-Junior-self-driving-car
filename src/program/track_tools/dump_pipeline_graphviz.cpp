#include <pipeline/pipeline.h>
#include <multibooster_support.h>

using namespace pipeline;
using namespace std;

void die() {
  cout << "Usage: dump_pipeline_graphviz FILENAME" << endl;
  exit(1);
}

int main(int argc, char** argv) {
  if(argc != 2)
    die();
  
  ClassifierPipeline cp(NULL, 1);
  cp.pipeline_.writeGraphviz(argv[1]);
  return 0;
}
