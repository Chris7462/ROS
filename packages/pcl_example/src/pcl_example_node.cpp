#include <iostream>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <cmath>

bool isVehicle(const pcl::PointXYZ &pt) {
  return (sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z) <= 3);
}

void callBack(const sensor_msgs::PointCloud2ConstPtr &msg) {
  //ROS_INFO_STREAM("Received Pandar topic");
  pcl::PointCloud<pcl::PointXYZ> cloud_in;
  pcl::fromROSMsg(*msg, cloud_in);

  std::cout << "Received PCL data\n";
  for ( auto point : cloud_in.points ) {
    if ( !isVehicle(point) ) {
      std::cout << "Points: " << point.x << ',' << point.y << ',' << point.z << '\n';
    }
  }
  std::cout << '\n';
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "subscribe_to_pcl2");
  ros::NodeHandle nh;
  std::string topic = nh.resolveName("pandar");
  //std::string topic = nh.resolveName("points_raw");
  const uint32_t queue_size = 1;

  ros::Subscriber sub = nh.subscribe(topic, queue_size, &callBack);
  ros::spin();

  return 0;
}
