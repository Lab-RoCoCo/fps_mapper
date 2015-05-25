#pragma once
#include "sensor_message_sorter.h"
#include "message_writer.h"

namespace txt_io {

  class MessageDumperTrigger: public SensorMessageSorter::Trigger{
  public:
    MessageDumperTrigger(SensorMessageSorter* sorter,
			 int priority,
			 MessageWriter* writer, 
			 const std::string& file_prefix="");

    virtual void action(std::tr1::shared_ptr<BaseSensorMessage> msg);

  protected:
    MessageWriter* _writer;
    std::string _file_prefix;
  };
}
