#include "base_message.h"
#include <cstdio>
#include <cstring>

namespace txt_io {
  using namespace std;

  BaseMessage::BaseMessage(){
    _is_fetched = true;
    _is_tainted = false;
    _binary_full_filename = "";
  }
  
  BaseMessage::~BaseMessage(){
    release();
  }
  
  void BaseMessage::fromStream(std::istream& is) {
  }

  void BaseMessage::toStream(std::ostream& os) const {
  }

  void BaseMessage::fetch(){
    if (_is_fetched)
      return;
    _fetch();
    _is_fetched=true;
  }

  void BaseMessage::writeBack() {
      _writeBack();
      _is_tainted = false;
  }

  void BaseMessage::release(){
    if (_is_tainted && _is_fetched) {
      _writeBack();
    }
    if (_is_fetched)
      _release();
    _is_fetched = false;
  }

  std::string BaseMessage::binaryFullFilename() const {
    if (!_binary_full_filename.length()) {
      _binary_full_filename = _binary_file_prefix + _binaryFilename();
    }
    return _binary_full_filename;
  }


  std::string BaseMessage::_binaryFilename() const{
    return "";
  }

  void BaseMessage::_fetch(){
  }

  void BaseMessage::_release(){
  }

  void BaseMessage::_writeBack(){
  }

}
