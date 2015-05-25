#include "tf_overrider_trigger.h"

namespace txt_io {

  TfOverriderTrigger::TfOverriderTrigger(SensorMessageSorter* sorter,
					 int priority,
					 StaticTransformTree* tree):
    SensorMessageSorter::Trigger(sorter, priority), _tree(tree){
    
  }

  void TfOverriderTrigger::action(std::tr1::shared_ptr<BaseSensorMessage> msg) {
    Eigen::Isometry3f iso;
    if (_tree->getOffset(iso, msg->frameId())) {
      msg->setOffset(iso);
    }
  }

}
