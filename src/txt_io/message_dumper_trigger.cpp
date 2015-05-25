#include "message_dumper_trigger.h"
#include "globals/defs.h"
#include "core/depth_utils.h"


namespace txt_io {
  using namespace std;

  MessageDumperTrigger::MessageDumperTrigger(SensorMessageSorter* sorter,
					     int priority,
					     MessageWriter* writer, 
					     const std::string& file_prefix) :
    SensorMessageSorter::Trigger(sorter, priority){
    _writer = writer;
    _file_prefix = file_prefix;
  }

  void MessageDumperTrigger::action(std::tr1::shared_ptr<BaseSensorMessage> msg){
    _writer->writeMessage(*msg);
    msg->writeBack();
  }

}
