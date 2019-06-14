#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cstdlib>

int main(int argc, char **argv){
  // Initialize the ROS system and become a node. "publish_velocity" is the node name in this case.
  ros::init(argc, argv, "publish_velocity");

  // Establish this program as a ROS node.
  ros::NodeHandle nh;

  // Create a publisher object. Advertising message with type <geometry_msg::Twist> on the topic "turtle1/cmd_vel" and queue size 1000.
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);

  //srand(static_cast<unsigned int>(time(NULL)));
  srand(time(0));

  const std::string PARAM_NAME="~max_vel";
  double maxVel;
  bool ok = ros::param::get(PARAM_NAME,maxVel);
  if( !ok ){
    ROS_FATAL_STREAM("Could not get parameter " << PARAM_NAME);
    exit(1);
  }

  // Loop at 2Hz until the node is shut down.
  ros::Rate rate(2);

  // Create the msg
  geometry_msgs::Twist msg;
  while( ros::ok() ){
    // fill the msg, see rosmsg show geometry_msgs/Twist for detail
    msg.linear.x = maxVel*double(rand())/double(RAND_MAX);
    msg.angular.z = 2*double(rand())/double(RAND_MAX)-1;

    // publish the message.
    pub.publish(msg);

    // send a message to rosout with the details
    //ROS_INFO_STREAM("Sending random velocity command: " << " linear=" << msg.linear.x << " angular=" << msg.angular.z);

    // wait until it's time for another iteration
    rate.sleep();
  }
  return 0;
}
