#pragma once
#include "txt_io/sensor_message_sorter.h"
#include "tracker.h"
namespace fps_mapper {
  
  class CallTrackerTrigger: public txt_io::SensorMessageSorter::Trigger{
  public:
    CallTrackerTrigger(txt_io::SensorMessageSorter* sorter,
		       int priority,
		       Tracker* tracker_);
    virtual void action(std::tr1::shared_ptr<txt_io::BaseSensorMessage> msg);
    
    inline Tracker* tracker() {return _tracker;}
  protected:
    Tracker* _tracker;
  };
}
