#include "call_tracker_trigger.h"
#include "txt_io/pinhole_image_message.h"

namespace fps_mapper {
  using namespace txt_io;

  CallTrackerTrigger::CallTrackerTrigger(SensorMessageSorter* sorter,
					 int priority,
					 Tracker* tracker_): SensorMessageSorter::Trigger(sorter, priority) {
    _tracker = tracker_;
  }

  void CallTrackerTrigger::action(std::tr1::shared_ptr<txt_io::BaseSensorMessage> msg) {
    PinholeImageMessage* img = dynamic_cast<PinholeImageMessage*>(msg.get());
    if (! img)
      return;


    Matrix6f odom_info;
    odom_info.setIdentity();
    
    _tracker->processFrame(img->image(),
			   RGBImage(),
			   img->cameraMatrix(),
			   img->depthScale(),
			   img->seq(),
			   img->timestamp(),
			   img->topic(),
			   img->frameId(),
			   img->offset(),
			   img->odometry(),
			   odom_info);
    img->untaint();
  }
}
