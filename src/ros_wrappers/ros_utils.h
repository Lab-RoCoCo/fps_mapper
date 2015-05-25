#include "boss/id_context.h"
#include "core/cloud.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Transform.h"
#include "tf/tf.h"


namespace fps_mapper{
  Eigen::Isometry3f pose2eigen(const geometry_msgs::Pose& p);
  geometry_msgs::Pose eigen2pose(const Eigen::Isometry3f& t);

  Eigen::Isometry3f transform2eigen(const geometry_msgs::Transform& p);
  geometry_msgs::Transform eigen2transform(const Eigen::Isometry3f& t);

  Eigen::Isometry3f tfTransform2eigen(const tf::Transform& p);
  tf::Transform eigen2tfTransform(const Eigen::Isometry3f& t);

}
