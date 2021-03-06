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

void HesaifromROSMsg(const sensor_msgs::PointCloud2 &cloud_msg, pcl::PointCloud<pcl::PointXYZI> &pc_out) {
  //Get the field structure of this point cloud
  int point_byes = cloud_msg.point_step;
  int offset_x;
  int offset_y;
  int offset_z;
  int offset_intensity;

  for ( size_t i = 0; i < cloud_msg.fields.size(); ++i ) {
    if ( cloud_msg.fields[i].name == "x" ) {
      offset_x = cloud_msg.fields[i].offset;
    }
    if ( cloud_msg.fields[i].name == "y" ) {
      offset_y = cloud_msg.fields[i].offset;
    }
    if ( cloud_msg.fields[i].name == "z" ) {
      offset_z = cloud_msg.fields[i].offset;
    }
    if ( cloud_msg.fields[i].name == "intensity" ) {
      offset_intensity = cloud_msg.fields[i].offset;
    }
  }

}

void callBack(const sensor_msgs::PointCloud2ConstPtr &msg) {
  // put msg to internal data structure
  pcl::PointCloud<pcl::PointXYZ> cloud_in;
  pcl::fromROSMsg(*msg, cloud_in);

  // transform the point cloud 90 degree clockwise along with z-axis.
//pcl::PointCloud<pcl::PointXYZ> cloud_transformed;
//Eigen::Matrix4f transformationMatrix;
//transformationMatrix <<
//  cos(M_PI_2), -sin(M_PI_2), 0, 0,
//  sin(M_PI_2),  cos(M_PI_2), 0, 0,
//  0, 0, 1, 0,
//  0, 0, 0, 1;

//pcl::transformPointCloud(cloud_in, cloud_transformed, transformationMatrix);

  std::cout << "Raw PCL data\n";
  for ( auto point : cloud_in.points ) {
    if ( !isVehicle(point) ) {
      std::cout << "Points: " << point.x << ',' << point.y << ',' << point.z << '\n';
    }
  }
  std::cout << '\n';

//std::cout << "Transformed PCL data\n";
//for ( auto point : cloud_transformed.points ) {
//  //if ( !isVehicle(point) ) {
//    std::cout << "Points: " << point.x << ',' << point.y << ',' << point.z << '\n';
//  //}
//}
//std::cout << '\n';

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "subscribe_to_pcl2");
  ros::NodeHandle nh;
  std::string topic = nh.resolveName("pandar");
  //std::string topic = nh.resolveName("points_raw");
  const uint32_t queue_size = 1;


//pcl::PointCloud<pcl::PointXYZ> cloud;
//cloud.push_back(pcl::PointXYZ(1,1,1));
//cloud.push_back(pcl::PointXYZ(2,2,2));
//cloud.push_back(pcl::PointXYZ(3,3,3));
//cloud.push_back(pcl::PointXYZ(4,4,4));
//cloud.push_back(pcl::PointXYZ(1,1,1));
//for ( size_t i = 0; i < cloud.size(); ++i ) {
//  ROS_INFO_STREAM("(x,y,z,i) = " << cloud.at(i));
//}

//pcl::PointCloud<pcl::PointXYZI>::Ptr in_temp(new pcl::PointCloud<pcl::PointXYZI>());
//copyPointCloud(cloud, *in_temp);
//for ( size_t i = 0; i < in_temp->size(); ++i ) {
//  ROS_INFO_STREAM("(x,y,z,i) = " << in_temp->at(i));
//}

//pcl::PointCloud<pcl::PointXYZI>::Ptr pc_copy(new pcl::PointCloud<pcl::PointXYZI>());
//*pc_copy = *in_temp;
//std::cout << &pc_copy << '\t' << &in_temp << '\n';

//pcl::PointXYZI point;
//for ( size_t i = 0; i < cloud.size(); ++i ) {
//  point.x = cloud[i].x;
//  point.y = cloud[i].y;
//  point.z = cloud[i].z;
//  ROS_INFO_STREAM(point);
//}
//for ( auto &point : pc_copy->points ) {
//  ROS_INFO_STREAM(point);
//}

  ros::Subscriber sub = nh.subscribe(topic, queue_size, &callBack);
  ros::spin();

  return 0;
}
