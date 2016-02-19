#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Quaternion.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

typedef struct {
  double x;
  double y;
  double theta;
} mypose;

mypose waypoints[3] = { { 2.1 , 5 , 0 } , {6.5 , 4.43 , 0} ,  {2 , 7 , 1.57} };

move_base_msgs::MoveBaseGoal WayPointToGoal(mypose p) {

  // Complete this function. It takes as input a variable of type mypose and it 
  // produces a an object of type move_bas_msgs::MoveBaseGoal

  move_base_msgs::MoveBaseGoal goal; 
  goal.target_pose.header.frame_id = "map"; // do not change this

  // complete the field in goal.target_pose.pose
}

int main(int argc,char **argv) {

  ros::init(argc,argv,"navigation");

  MoveBaseClient ac("move_base",true);
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }


  // Iterate over all waypoints in the array waypoints and send all of them 
  // to the navigation stack. Wait for each waypoint to be reached before sending the next one

  // To send a waypoint to the navigation stack use the methods 
  // 1) sendGoal
  // 2) waitForResult
  // defined in MoveBaseClient

}
