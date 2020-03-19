#include <multibooster_support.h>

using namespace std;

int main(int argc, char** argv) {
  if(argc != 2) {
    cout << argv[0] << " MATRIX" << endl;
    return -1;
  }

  cout << printMatrix(argv[1]) << endl;
  return 0;
}
