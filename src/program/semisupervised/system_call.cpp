#include <iostream>
#include <stdlib.h>

using namespace std;

int main(int argc, char** argv)
{
  cout << "Waiting for gdb attach..." << endl;
  for(int j = 0; j < 10; ++j) { 
    for(int i = 0; i < 1e9; ++i) {
      int x = i * 100;
    }
  }
  cout << "Running system call." << endl;
  string foo = ".";
  int ret = system(string("touch " + foo + "/`hostname`").c_str());
  
  system("touch `hostname`");
  cout << "Done." << endl;
  
  return 0;
}
