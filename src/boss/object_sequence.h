#include "serializable.h"
#include "serializer.h"
#include "deserializer.h"
#include "bidirectional_serializer.h"
#include <stdexcept>
#include <list>

namespace boss {
  struct ObjectSequence: public std::map<int, Serializable*> {
    
    inline const std::string& name() const {return _name;}
    inline int firstSeq() const {return _first_seq;}
    inline int lastSeq() const {return _last_seq;}
    
    ObjectSequence(const std::string& name_);
    void push_back(Serializable* s);
    void pop_front();    
    Serializable* front();
    void serialize(std::list<std::string>& out, int& start_seq, int& end_seq, Serializer* ser);
    void deserialize(std::list<std::string>& in, Deserializer* des);

  protected:
    int _first_seq;
    int _last_seq;
    std::string _name;
  };
}
