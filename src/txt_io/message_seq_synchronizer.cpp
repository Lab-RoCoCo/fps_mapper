#include "message_seq_synchronizer.h"
#include <iostream>

namespace txt_io {
  using namespace std;

  MessageSeqSynchronizer::MessageSeqSynchronizer() {
    _last_seq=-1;
  }

  void MessageSeqSynchronizer::setTopics(const std::vector<std::string>& topics_) {
    _topics = topics_;
    _messages.resize(_topics.size());
  }

  void MessageSeqSynchronizer::putMessage(BaseSensorMessage* msg) {
    for(size_t i=0; i<_topics.size(); i++){
      if (_topics[i]==msg->topic()) {
	_messages[i]=std::tr1::shared_ptr<BaseSensorMessage>(msg);
	if(msg->seq()>_last_seq)
	  _last_seq=msg->seq();
      }
    }
  }


  void MessageSeqSynchronizer::putMessage(std::tr1::shared_ptr<BaseSensorMessage>& msg) {
    for(size_t i=0; i<_topics.size(); i++){
      if (_topics[i]==msg->topic()) {
	_messages[i]=msg;
	if(msg->seq()>_last_seq)
	  _last_seq=msg->seq();
      }
    }
  }


  bool MessageSeqSynchronizer::messagesReady() {
    bool ready=false;
    for(size_t i=0; i<_messages.size(); i++){
      if (!_messages[i].get())
	return false;
      if (_messages[i]->seq()!=_last_seq)
	return false;
    }
    return true;
  }


  void MessageSeqSynchronizer::reset() {
    for(size_t i=0; i<_messages.size(); i++){
      _messages[i].reset();
    }
    _last_seq = -1;
  }

}
