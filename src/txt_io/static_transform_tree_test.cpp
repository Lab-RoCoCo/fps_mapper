#include "static_transform_tree.h"
#include "message_reader.h"
using namespace std;
using namespace Eigen;
using namespace txt_io;

int main (int argc, char** argv) {
  if (argc <1) {
    cerr << "usage: " <<argv[0] << " <filename> " << endl;
  }
  StaticTransformTree tree;
  MessageReader reader;
  reader.open(argv[1]);
  while(reader.good()){
    BaseMessage* msg=reader.readMessage();
    if (! msg)
      continue;
    StaticTransformMessage* tf=dynamic_cast<StaticTransformMessage*>(msg);
    if (tf) {
      tree.addMessage(tf);
    }
  }
  cerr << "tree root: " << tree.rootFrameId() << endl;
  tree.isWellFormed();
}
