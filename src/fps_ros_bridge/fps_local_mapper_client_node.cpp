#include "fps_ros_msgs.h"
#include "fps_mapper/StampedCloudMsg.h"
#include "cloud_publisher_trigger.h"
#include <ros/ros.h>
#include "txt_io/message_writer.h"
#include "local_map_listener.h"


using namespace boss;
using namespace fps_mapper;
using namespace std;
  

class LocalMapListenerEXAMPLE: public LocalMapListener {
public:
  // override this and do what you want with the local map
  virtual void onNewLocalMap(LocalMap* lmap) {
    cerr << "got local map pppppppppppppp " << endl;
  }
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "listener");
  LocalMapListenerEXAMPLE* example = new LocalMapListenerEXAMPLE;
  ros::NodeHandle n;
  example->init(n);
  ros::spin();
}
