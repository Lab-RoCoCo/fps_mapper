#pragma once
#include "sensor_message_sorter.h"
#include "message_writer.h"
#include <list>

namespace txt_io {

  typedef std::list<std::tr1::shared_ptr<BaseSensorMessage> >SensorMessageList;

  class MessageEnlisterTrigger: public SensorMessageSorter::Trigger{
  public:
    MessageEnlisterTrigger(SensorMessageSorter* sorter,
			 int priority,
			 SensorMessageList* list_);

    virtual void action(std::tr1::shared_ptr<BaseSensorMessage> msg);

  protected:
    SensorMessageList* _list;
  };
}
