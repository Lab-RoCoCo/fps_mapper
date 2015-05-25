#include "ros_utils.h"
#include "stdio.h"

using namespace fps_mapper;

int main(int argc, char** argv){
  boss::IdContext context1;
  boss::IdContext context2;

  for (int i = 0; i<10; i++) {
    PinholeCameraInfo* cam1 = new PinholeCameraInfo;
    PinholeCameraInfoMsg msg1 = pinholeCameraInfo2msg(cam1, &context1);
    PinholeCameraInfo* cam2 = msg2pinholeCameraInfo(msg1, &context2);
    cerr << "cam2.id()" << endl;
  }

}
