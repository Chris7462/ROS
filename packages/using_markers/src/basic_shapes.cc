#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv){
  // node name: basic_shapes
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle nh;
  ros::Rate rate(1);
  // publish msgs with type vis_msg to topic named vis_marker
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while( ros::ok() ){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();

    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type
    marker.type = shape;

    // Set the marker action.
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the matrix
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // publish the marker
    while( marker_pub.getNumSubscribers() < 1 ){
      if ( !ros::ok() ){
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);

    // Cycle between different shapes
    switch ( shape ){
      case visualization_msgs::Marker::CUBE:
        shape = visualization_msgs::Marker::SPHERE;
        break;
      case visualization_msgs::Marker::SPHERE:
        shape = visualization_msgs::Marker::ARROW;
        break;
      case visualization_msgs::Marker::ARROW:
        shape = visualization_msgs::Marker::CYLINDER;
        break;
      case visualization_msgs::Marker::CYLINDER:
        shape = visualization_msgs::Marker::CUBE;
        break;
    }
    rate.sleep();
  }
  return 0;
}
