#include <string>
#include <iostream>
#include <fstream>
#include "message_reader.h"
#include <sstream>
#include "message_factory.h"

namespace txt_io {
  using namespace std;

  MessageReader::MessageReader() {
    _is = 0;
    _filename = "";
  }
  
  #define MAX_BUF_SIZE 65535

  BaseMessage* MessageReader::readMessage(){
    if (! _is)
      return 0;
    char buf[MAX_BUF_SIZE];
    _is->getline(buf, MAX_BUF_SIZE);
    istringstream is(buf);
    std::string tag;
    is >> tag;
    BaseMessage* m = MessageFactory::instance()->create(tag);
    if (!m)
      return 0;
    m->fromStream(is);
    return m;
  }

  bool MessageReader::good() const {
    if (! _is)
      return false;
    if (! _is->good())
      return false;
    return true;
  }

  void MessageReader::open(const std::string& filename_){
    if (_is){
      delete _is;
    }
    _filename=filename_;
    _is = new std::ifstream(_filename.c_str());
    if (! _is->good()){
      delete _is;
      _is = 0;
    }
  }
    
  void MessageReader::close(){
   if (_is){
      delete _is;
    }
    _is = 0;
    _filename = "";
  }

  MessageReader::~MessageReader() {
    close();
  }
    
}
