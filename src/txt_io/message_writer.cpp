#include <string>
#include <iostream>
#include <fstream>
#include "message_writer.h"
#include <sys/types.h>
#include <sys/stat.h>

namespace txt_io {
  using namespace std;

  MessageWriter::MessageWriter() {
    _os = 0;
    _filename = "";
    _binary_file_prefix = "";
  }

  MessageWriter::~MessageWriter() {
  }

  void MessageWriter::open(const std::string& filename_) {
    close();
    _filename = filename_;
    _binary_file_prefix = _filename+".d";

    mkdir(_binary_file_prefix.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
    _binary_file_prefix = _filename+".d/";
    _os = new ofstream(_filename.c_str());
  }


  void MessageWriter::close() {
    if (_os)
      delete _os;
    _os=0;
    _filename = "";
    _binary_file_prefix = "";
  }

  void MessageWriter::writeMessage(const BaseMessage& msg) {
    if (!_os)
      return;
    msg.setBinaryFilePrefix(_binary_file_prefix);
    *_os << msg.tag() << " ";
    msg.toStream(*_os);
   * _os << endl;
  }

}
