#include <pipeline/pipeline.h>
#include <multibooster_support.h>

using namespace std;

int main(int argc, char** argv)
{
  DescriptorPipeline dp(1);
  cout << dp.pipeline_.getGraphviz() << endl;
  return 0;
}
