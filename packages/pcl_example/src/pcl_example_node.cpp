#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>

void callBack(const sensor_msgs::PointCloud2ConstPtr &msg) {
  //ROS_INFO_STREAM("Received Pandar topic");
  pcl::PointCloud<pcl::PointXYZ> cloud_in;
  pcl::fromROSMsg(*msg, cloud_in);
  ROS_INFO_STREAM(cloud_in);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "subscribe_to_pcl2");
  ros::NodeHandle nh;
  std::string topic = nh.resolveName("pandar");
  const uint32_t queue_size = 1;

  ros::Subscriber sub = nh.subscribe(topic, queue_size, &callBack);
  ros::spin();

  return 0;
}
