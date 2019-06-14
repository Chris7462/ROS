#include <ros/ros.h>
#include <fusion_msgs/Num.h>

int main(int argc, char **argv){
  // Initialize the ROS system and become a node. "publish_velocity" is the node name in this case.
  ros::init(argc, argv, "publish_msg");

  // Establish this program as a ROS node.
  ros::NodeHandle nh;

  // Create a publisher object. Advertising message with type <geometry_msg::Twist> on the topic "turtle1/cmd_vel" and queue size 1000.
  ros::Publisher pub = nh.advertise<fusion_msgs::Num>("msgs_topic", 1000);

  srand(time(0));

  // Loop at 2Hz until the node is shut down.
  ros::Rate rate(2);

  // Create the msg
  fusion_msgs::Num msg;

  while( ros::ok() ){
    msg.num = rand();

    // publish the message.
    pub.publish(msg);

    // send a message to rosout with the details
    ROS_INFO_STREAM("Sending random number: " << msg.num );

    // wait until it's time for another iteration
    rate.sleep();
  }
  return 0;
}
