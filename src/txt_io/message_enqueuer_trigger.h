#include "message_dumper_trigger.h"
#include "globals/defs.h"
#include "core/depth_utils.h"
#include <list>

namespace txt_io {
  using namespace std;

  std::list<std::tr1::shared_ptr<BaseSensorMsg> > SensorMessageList;

  MessageEnqueuerTrigger::MessageEnlisterTrigger(SensorMessageSorter* sorter,
						 int priority,
						 SensorMessageQueue* queue_) :
    SensorMessageSorter::Trigger(sorter, priority){
    _queue = queue;
  }

  void MessageDumperTrigger::action(BaseSensorMessage& msg){
  }

}
