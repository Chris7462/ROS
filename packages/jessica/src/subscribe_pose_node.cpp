#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <iomanip>

// A callback function. Executed each time a new pose message arrives
void poseMessageReceived(const turtlesim::Pose &msg){
  ROS_INFO_STREAM("Listening on topic: turtle1/pose");
  ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "Position = (" << msg.x << "," << msg.y << ")" << " direction = " << msg.theta);
}

//void poseMessageReceived(const turtlesim::Pose::ConstPtr &msg){
//  ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "Position = (" << msg->x << "," << msg->y << ")" << " direction = " << msg->theta);
//}

int main(int argc, char **argv) {
  // Initialize the ROS system and become a node.
  ros::init(argc, argv, "subscribe_to_pose");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("turtle1/pose", 100, &poseMessageReceived);
  ros::spin();
  //ros::spinOnce();

  ROS_INFO_STREAM("Done callback");

  return 0;
}
