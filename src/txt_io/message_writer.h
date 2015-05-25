#pragma once
#include <string>
#include <iostream>
#include <fstream>
#include "base_message.h"

namespace txt_io {
  class MessageWriter {
  public:
    MessageWriter();
    ~MessageWriter();
    void open(const std::string& filename);
    void close();
    void writeMessage(const BaseMessage& msg);

    inline std::ostream* outputStream() { return _os; }
    inline const std::string& filename() const { return _filename; }
    inline const std::string& binaryFilePrefix() const {return _binary_file_prefix;}
  protected:
    std::ofstream* _os;
    std::string _filename;
    std::string _binary_file_prefix;

  };

}
