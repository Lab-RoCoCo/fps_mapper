#pragma once

#include "base_sensor_message.h"
#include <vector>
#include <string>
#include <tr1/memory>

namespace txt_io {

  class MessageSeqSynchronizer{
  public:
    MessageSeqSynchronizer();
    void setTopics(const std::vector<std::string>& topics_);
    inline const std::vector<std::string> topics() const {return _topics;}
    inline std::vector<std::tr1::shared_ptr<BaseSensorMessage> >& messages() { return _messages; }
    void putMessage(BaseSensorMessage* msg);
    void putMessage(std::tr1::shared_ptr<BaseSensorMessage>& msg);
    bool messagesReady();
    void reset();
  protected:
    std::vector<std::tr1::shared_ptr<BaseSensorMessage> > _messages;
    std::vector<std::string> _topics;
    int _last_seq;
  };

}
