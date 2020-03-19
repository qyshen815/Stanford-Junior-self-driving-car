#include "ladybug3_model.h"

using namespace std;

int main(__attribute__((unused)) int argc, __attribute__((unused)) char** argv) {
  
  Ladybug3Intrinsics lb3("camera_info_dump");
  for(int i=0; i<6; ++i)
    cout << lb3.cameras_[i]->status() << endl;
}

