#include "sensor_message_sorter.h"
#include <stdexcept>
namespace txt_io{
  using namespace std;

  SensorMessageSorter::Trigger::Trigger(SensorMessageSorter* sorter_, int priority_) {
    _sorter = sorter_;
    _priority = priority_;
    std::map<int, Trigger*>::iterator it = _sorter->_triggers.find(priority_);
    if(it!=_sorter->_triggers.end())
      throw std::runtime_error("cannot allocate a trigger with the same priority");
    _sorter->_triggers.insert(std::make_pair(priority(), this));
  }

  SensorMessageSorter::Trigger::~Trigger() {
    std::map<int, Trigger*>::iterator it = _sorter->_triggers.find(_priority);
    if(it==_sorter->_triggers.end())
      throw std::runtime_error("error in delete");
    _sorter->_triggers.erase(it);
  }

  SensorMessageSorter::SensorMessageSorter(double time_window){
    _time_window = time_window;
    _oldest_time = 0;
    _write_back_enabled = true;
  }

  void SensorMessageSorter::callTriggers(std::tr1::shared_ptr<BaseSensorMessage> msg){
    for (std::map<int, Trigger*>::iterator it = _triggers.begin(); it!=_triggers.end(); it++){
      it->second->action(msg);
    }
  }

  void SensorMessageSorter::insertMessage(BaseSensorMessage* msg) {
    if (msg->timestamp() < _oldest_time)
      return;
    _messages.insert(std::make_pair(msg->timestamp(), std::tr1::shared_ptr<BaseSensorMessage>(msg)));
    if (msg->timestamp() > _newest_time){
      _newest_time = msg->timestamp();
    }
    
    // purge the leftovers
    double cut_time = _newest_time - _time_window;
    while (!_messages.empty() && _messages.begin()->first < cut_time) {
      std::tr1::shared_ptr<BaseSensorMessage>& msg = _messages.begin()->second;
      callTriggers(msg);
      if (! _write_back_enabled)
	_messages.begin()->second->untaint();
      _messages.erase(_messages.begin());
      _oldest_time = _messages.begin()->first;
    }
  }

  SensorMessageSorter::~SensorMessageSorter(){
    for (DoubleMessageMap::iterator it=_messages.begin(); it!=_messages.end(); it++){
      it->second->untaint();
    }
  }
}
