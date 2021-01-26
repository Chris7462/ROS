#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cstdlib>

int main(int argc, char **argv)
{
  // Initialize the ROS system and becom a node.
  ros::init(argc, argv, "publish_velocity");
  ros::NodeHandle nh;

  // Create a publisher object.
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 100);

  // Seed the random number generator.
  srand(time(0));

  // Loop at 2Hz until the node is shut down.
  ros::Rate rate(2);

  while ( ros::ok() ) {
    // Create and fill in the message. The other four fields, which are ignored by turtlesim, default to 0.
    geometry_msgs::Twist msg;
    msg.linear.x = static_cast<double>(rand())/static_cast<double>(RAND_MAX);
    msg.angular.z = 2*static_cast<double>(rand())/static_cast<double>(RAND_MAX)-1;

    // Publish the message
    pub.publish(msg);

    // Send a message to rosout with the details.
    ROS_INFO_STREAM("Sending random velocity command: linear = " << msg.linear.x << " angular = " << msg.angular.z);

    // Wait until it's time for another iteration.
    rate.sleep();
  }

  return 0;
}
