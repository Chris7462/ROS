#include <ros/ros.h>
#include <fusion_msgs/Num.h> // see rosmsg list for detail

// A callback function. Executed each time a new pose message arrives
void msgsMessageReceived(const fusion_msgs::Num& msg){
  ROS_INFO_STREAM("Random Number = " << msg.num);
}

int main(int argc, char **argv){
  // initialize the ROS system and become a node.
  ros::init(argc, argv, "subscribe_to_msg");

  // Establish this program as a ROS node.
  ros::NodeHandle nh;

  // Create a subscriber object. rostopic list /turtle1/pose
  ros::Subscriber sub = nh.subscribe("msgs_topic", 1000, &msgsMessageReceived);

  // Let ROS take over.
  ros::spin();

  return 0;
}
