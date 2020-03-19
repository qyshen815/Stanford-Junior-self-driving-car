#include <online_learning/online_learning.h>

using namespace std;
namespace om = odontomachus;

class Foo
{
public:
  vector<double> data_;
  Foo() {
    data_.resize(5e8, 0);
  }
};


int main(int argc, char** argv)
{

  Foo* data = new Foo();
  cout << "Allocated." << endl;
  sleep(2);
  delete data;
  cout << "Deallocated." << endl;
  sleep(2);

  vector<double>* foo = new vector<double>(5e8, 0);
  cout << "Allocated." << endl;
  sleep(2);
  delete foo;
  cout << "Deallocated." << endl;
  sleep(2);

  
  return 0;
}
