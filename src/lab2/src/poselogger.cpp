#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

#include <tf/tf.h>
#include <geometry_msgs/Transform.h>

// ros::Publisher pub1;
// ros::Publisher pub2;

void poseCallback1(const turtlesim::PoseConstPtr& msg)
{
  ROS_INFO_STREAM("Turtle1" << " :\n\tposition:\n\t\tx: " << msg->x << "\n\t\ty: " << msg->y <<
  "\n\torientation: " << "\n\t\tYaw: " << msg->theta << "\n\n");
}

void poseCallback2(const turtlesim::PoseConstPtr& msg)
{
  ROS_INFO_STREAM("Turtle2" << " :\n\tposition:\n\t\tx: " << msg->x << "\n\t\ty: " << msg->y <<
  "\n\torientation: " << "\n\t\tYaw: " << msg->theta << "\n\n");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "poselogger");

  ros::NodeHandle node;
  ros::Subscriber sub1 = node.subscribe( "turtle1/pose", 10, &poseCallback1 );
  ros::Subscriber sub2 = node.subscribe( "turtle2/pose", 10, &poseCallback2 );

  ros::spin();

  return 0;
}