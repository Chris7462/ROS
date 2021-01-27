// This program periodically generates log messages at various severity levels
#include <ros/ros.h>

int main(int argc, char **argv) {
  // Initialize the ROS system and become a node
  ros::init(argc, argv, "count_and_log");
  ros::NodeHandle nh;

  // Generate log messages of varying severity regularly.
  ros::Rate rate(10);
  for ( int i = 0; ros::ok(); ++i ) {
    // log only once
    ROS_DEBUG_STREAM_ONCE("Show only once: Counted to " << i);
    ROS_INFO_STREAM_ONCE("Show only once: " << i << " is divisible by 3.");
    ROS_WARN_STREAM_ONCE("Show only once: " << i << " is divisible by 5.");
    ROS_ERROR_STREAM_ONCE("Show only once: " << i << " is divisible by 10.");
    ROS_FATAL_STREAM_ONCE("Show only once: " << i << " is divisible by 20.");

    ROS_DEBUG_STREAM("Counted to " << i);
    if ( ( i % 3 ) == 0 ) {
      ROS_INFO_STREAM(i << " is divisible by 3.");
    }
    if ( ( i % 5 ) == 0 ) {
      ROS_WARN_STREAM(i << " is divisible by 5.");
    }
    if ( ( i % 10 ) == 0 ) {
      ROS_ERROR_STREAM(i << " is divisible by 10.");
    }
    if ( ( i % 20 ) == 0 ) {
      ROS_FATAL_STREAM(i << " is divisible by 20.");
    }

    ROS_DEBUG_STREAM_THROTTLE(0.1, "This appears every 0.1 seconds.");
    ROS_INFO_STREAM_THROTTLE(0.3, "This appears every 0.3 seconds.");
    ROS_WARN_STREAM_THROTTLE(0.5, "This appears every 0.5 seconds.");
    ROS_ERROR_STREAM_THROTTLE(1.0, "This appears every 1.0 seconds.");
    ROS_FATAL_STREAM_THROTTLE(2.0, "This appears every 2.0 seconds.");
  }

  return 0;
}
