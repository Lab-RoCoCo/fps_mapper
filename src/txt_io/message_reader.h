#pragma once
#include <string>
#include <iostream>
#include <fstream>
#include "base_message.h"
namespace txt_io {
  class MessageReader {
  public:
    MessageReader();
    ~MessageReader();
    void open(const std::string& filename);
    void close();
    BaseMessage* readMessage();
    bool good() const;
    inline std::istream* inputStream() { return _is; }
    inline const std::string& filename() const { return _filename; }
    
  protected:
    std::ifstream* _is;
    std::string _filename;
  };

}
