#pragma once
#include "sensor_message_sorter.h"
#include "static_transform_tree.h"

namespace txt_io {

  class TfOverriderTrigger: public SensorMessageSorter::Trigger{
  public:
    TfOverriderTrigger(SensorMessageSorter* sorter,
		       int priority,
		       StaticTransformTree* tree);

    virtual void action(std::tr1::shared_ptr<BaseSensorMessage> msg);

  protected:
    StaticTransformTree* _tree;
  };
}
