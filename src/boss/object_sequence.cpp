#include "object_sequence.h"

namespace boss {
    
  ObjectSequence::ObjectSequence(const std::string& name_) {
    _first_seq = 0;
    _last_seq = 0;
    _name = name_;
  }

  void ObjectSequence::push_back(Serializable* s) {
    insert(std::make_pair(_last_seq, s));
    _last_seq++;
  }

  void ObjectSequence::pop_front() {
    if (empty())
      return;
    erase(_first_seq);
    _first_seq++;
  }
    
  Serializable* ObjectSequence::front() {
    if (empty())
      return 0;
    return begin()->second;
  }

  void ObjectSequence::serialize(std::list<std::string>& out, int& start_seq, int& end_seq, Serializer* ser) {
    ObjectSequence::iterator s = begin();
    ObjectSequence::iterator e = end();
    if (start_seq != 0)
      s = find(start_seq);
    if (s == end())
      throw std::runtime_error("invalid start id in sequence");
    if (end_seq!=0)
      e = find(end_seq);
    if (e == end() && end_seq!=0)
      throw std::runtime_error("invalid end id in sequence");
    for (iterator it = s; it!=e; it++){
      std::string line;
      ser->writeObject(line, *it->second);
      out.push_back(line);
      end_seq = it->first;
    }
    start_seq = s->first;
  }

  void ObjectSequence::deserialize(std::list<std::string>& in, Deserializer* des){
    for (std::list<std::string>::iterator it = in.begin(); it!=in.end(); it++){
      std::string& line = *it;
      Serializable * s = des->readObject(line);
      if (!s)
	throw std::runtime_error("unable to deserialize object. Is that registered?");
      push_back(s);
    }
  }
}
