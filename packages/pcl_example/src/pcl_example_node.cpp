#include <iostream>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <cmath>

bool isVehicle(const pcl::PointXYZ &pt) {
  return (sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z) <= 3);
}

void callBack(const sensor_msgs::PointCloud2ConstPtr &msg) {
  // put msg to internal data structure
  pcl::PointCloud<pcl::PointXYZ> cloud_in;
  pcl::fromROSMsg(*msg, cloud_in);

  // transform the point cloud 90 degree clockwise along with z-axis.
  pcl::PointCloud<pcl::PointXYZ> cloud_transformed;
  Eigen::Matrix4f transformationMatrix;
  transformationMatrix <<
    cos(M_PI_2), -sin(M_PI_2), 0, 0,
    sin(M_PI_2),  cos(M_PI_2), 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1;

  pcl::transformPointCloud(cloud_in, cloud_transformed, transformationMatrix);

  /*
  std::cout << "Raw PCL data\n";
  for ( auto point : cloud_in.points ) {
    if ( !isVehicle(point) ) {
      std::cout << "Points: " << point.x << ',' << point.y << ',' << point.z << '\n';
    }
  }
  std::cout << '\n';
  */

  std::cout << "Transformed PCL data\n";
  for ( auto point : cloud_transformed.points ) {
    //if ( !isVehicle(point) ) {
      std::cout << "Points: " << point.x << ',' << point.y << ',' << point.z << '\n';
    //}
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
