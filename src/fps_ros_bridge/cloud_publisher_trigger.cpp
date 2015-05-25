#include "cloud_publisher_trigger.h"
#include "fps_ros_msgs.h"
#include "fps_mapper/StampedCloudMsg.h"

namespace fps_mapper {

  CloudPublisherTrigger::CloudPublisherTrigger(fps_mapper::Tracker* tracker, 
					       int event, int priority,
					       ros::NodeHandle & nh, 
					       tf::TransformBroadcaster* broadcaster_,
					       const std::string& prefix_,
					       const std::string& odom_frame_id) :
  fps_mapper::Tracker::Trigger(tracker, event, priority){
    _broadcaster = broadcaster_;
    _prefix = prefix_;
    _odom_frame_id = odom_frame_id;
    _reference_cloud_publisher = nh.advertise<fps_mapper::StampedCloudMsg>(_prefix+"/reference_cloud", 10);
    _current_cloud_publisher = nh.advertise<fps_mapper::StampedCloudMsg>(_prefix+"/current_cloud", 10)
;
    _count = 0;
    _skip_rate = 2;
  }

  void CloudPublisherTrigger::action(fps_mapper::Tracker::TriggerEvent )  {
    if (! tracker())
    return;
    _count ++;
    const Eigen::Isometry3f& T = tracker()->globalT();
    Eigen::Isometry3f delta = T*tracker()->lastInitialGuess().inverse();
    _broadcaster->sendTransform(tf::StampedTransform(eigen2tfTransform(delta), 
						     ros::Time(tracker()->lastTimestamp()), 
						     "tracker_origin_frame_id", _odom_frame_id));

    std_msgs::Header header;
    header.frame_id = "/base_link";
    header.seq = tracker()->lastSeq();
    header.stamp = ros::Time(tracker()->lastTimestamp());
    if (_reference_cloud_publisher.getNumSubscribers() && tracker()->referenceModel()) {
      fps_mapper::StampedCloudMsg msg;
      msg.header = header;
      cloud2msg(msg.cloud , *tracker()->referenceModel());
      _reference_cloud_publisher.publish(msg);
    }

    if (_current_cloud_publisher.getNumSubscribers() && tracker()->currentModel()) {
      fps_mapper::StampedCloudMsg msg;
      msg.header = header;
      cloud2msg(msg.cloud, *tracker()->currentModel());
      _current_cloud_publisher.publish(msg);
    }
  }

}
