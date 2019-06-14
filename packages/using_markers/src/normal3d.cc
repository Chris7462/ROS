#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <random>
#include <chrono>

int main(int argc, char** argv){
  ros::init(argc,argv,"normal3d");
  ros::NodeHandle nh;
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker",10);

  ros::Rate rate(5);

  int i = 0 ;
  while ( ros::ok() ){
    // create vis_msgs objects
    visualization_msgs::Marker points;

    // setting header::stame and frame_id
    points.header.stamp = ros::Time::now();
    points.header.frame_id = "/my_frame";

    // object namespace
    points.ns = "points";

    // object id
    points.id = 0;

    // object type
    points.type = visualization_msgs::Marker::POINTS;

    // object action
    points.action = visualization_msgs::Marker::ADD;

    // object pose; position.x,y,z = 0 by default orientation.x,y,z=0
    points.pose.orientation.w = 1.0;

    // object scale; x,y,z
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    // object color
    points.color.b = 1.0f;
    points.color.a = 1.0;

    // generate normal points
    const uint32_t nrolls = 10000;


    // construct a trivial random generator engine from a time-based seed:
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::normal_distribution<double> distribution(0.0,1.0);

    for ( uint32_t i = 0 ; i < nrolls ; ++i ){

      double x = distribution(generator);
      double y = distribution(generator);
      double z = distribution(generator);

      geometry_msgs::Point p;
      p.x = x;
      p.y = y;
      p.z = z;

      points.points.push_back(p);
    }

    marker_pub.publish(points);

    // send a message to rosout with the details
    ROS_INFO_STREAM("Sending msg: " << i );
    ++i;

    rate.sleep();
  }

  return 0;
}
