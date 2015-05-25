#pragma once
#include <string>
#include <iostream>
namespace txt_io {


  class BaseMessage{
  public:
    BaseMessage();
    virtual ~BaseMessage();
    virtual const std::string& tag() const = 0;
    virtual void fromStream(std::istream& is);
    virtual void  toStream(std::ostream& os) const;
    void fetch();
    void release(); 
    void writeBack();
    inline void taint() {_is_tainted = true; }
    inline void untaint() {_is_tainted = false; }
    inline const std::string& binaryFilePrefix() const {return _binary_file_prefix;}
    inline void setBinaryFilePrefix(const std::string& bup) const {_binary_file_prefix=bup;}
    std::string binaryFullFilename() const;
  protected:
    virtual std::string _binaryFilename() const;
    virtual void _fetch();
    virtual void _release();
    virtual void _writeBack();
    
    mutable std::string _binary_file_prefix;
    mutable std::string _binary_full_filename;
    bool _is_fetched;
    bool _is_tainted;
  };
}
