#include <ros/ros.h>
#include <turtlesim/Pose.h> // see rosmsg list for detail
#include <iomanip>

// A callback function. Executed each time a new pose message arrives
void poseMessageReceived(const turtlesim::Pose& msg){
  ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "position=(" << msg.x << "," << msg.y << ")" << " direction=" << msg.theta);
}

int main(int argc, char **argv){
  // initialize the ROS system and become a node.
  ros::init(argc, argv, "subscribe_to_pose");

  // Establish this program as a ROS node.
  ros::NodeHandle nh;

  // Create a subscriber object. rostopic list /turtle1/pose
  ros::Subscriber sub = nh.subscribe("turtle1/pose", 1000, &poseMessageReceived);

  // Let ROS take over.
  ros::spin();

  return 0;
}
