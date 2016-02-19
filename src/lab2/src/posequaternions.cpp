#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

#include <tf/tf.h>
#include <geometry_msgs/Transform.h>

void poseCallback2(const turtlesim::PoseConstPtr& msg)
{
  tf::Quaternion q;
  q.setRPY(0, 0, msg->theta);

  q.normalize();

  tf::Vector3 v = q.getAxis();

  ROS_INFO_STREAM("Turtle2" << " :\n\tquaternion:\n\t\tw: " << q.getW() << "\n\t\tx: " << v.getX() <<
  "\n\t\ty: " << v.getY() << "\n\t\tz: " << v.getZ() << "\n\n");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "posequaternions");

  ros::NodeHandle node;
  ros::Subscriber sub2 = node.subscribe( "turtle2/pose", 10, &poseCallback2 );

  ros::spin();

  return 0;
}