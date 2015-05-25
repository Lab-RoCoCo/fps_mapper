#include "message_reader.h"
#include "globals/system_utils.h"
#include <cstring>

using namespace system_utils;
using namespace std;
using namespace txt_io;

const char* banner[]={
  "txt_io_dump_reader_app: a simple example on reading a dump file written with txt io",
  " it reads sequentially all elements in the file",
  " instantiates the objects and prints the class name",
  "",
  "usage: txt_io_dump_reader_app <dump_file>",
  0
};

int main(int argc, char ** argv) {
  if (argc<2 || ! strcmp(argv[1],"-h")) {
    printBanner(banner);
    return 0;
  }

  MessageReader reader;
  reader.open(argv[1]);
  txt_io::BaseMessage* msg=0;
  while ( (msg = reader.readMessage()) ) {
    cerr << msg->tag() << endl;
  }
  
  return 0;
}
