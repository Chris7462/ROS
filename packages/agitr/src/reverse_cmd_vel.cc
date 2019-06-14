#include <ros/ros.h>
#include <geometry_msgs/Twist.h>  // rostopic info turtle1/cmd_vel

ros::Publisher *pubPtr;

void commandVelocityReceived(const geometry_msgs::Twist& msgIn){
  geometry_msgs::Twist msgOut;    
  // rosmsg show geometry_msgs/Twist
  msgOut.linear.x = -msgIn.linear.x;
  msgOut.angular.z = -msgIn.angular.z;
  pubPtr->publish(msgOut);
}

int main(int argc, char **argv){
  ros::init(argc,argv,"reverse_velocity");  // node name
  ros::NodeHandle nh;

  pubPtr = new ros::Publisher(nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel_reversed",1000));
//  pubPtr = new ros::Publisher(nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1000));

  ros::Subscriber sub = nh.subscribe("turtle1/cmd_vel", 1000, &commandVelocityReceived);

  ros::spin();

  return 0;
}
