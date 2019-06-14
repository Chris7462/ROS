#include <ros/ros.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv){
  ros::init(argc,argv, "spawn_turtle");
  ros::NodeHandle nh;

  // Create a client object for the spawn service. This needs to know the data type of the service and its name
  ros::ServiceClient spawnClient = nh.serviceClient<turtlesim::Spawn>("spawn");

  // Create the request and response object.
  turtlesim::Spawn::Request req;
  turtlesim::Spawn::Response resp;

  // Full in the request data members. Data members are from "rossrv show turtlesim/Spawn"
  req.x = 2;
  req.y = 3;
  req.theta = M_PI/2;
  req.name = "Leo";

  // Actually call the service. This won't return until the service is complete
  bool success = spawnClient.call(req,resp);

  // Check for success and use the response
  if ( success ){
    ROS_INFO_STREAM("Spawned a turtle named " << resp.name);
  } else {
    ROS_ERROR_STREAM("Failed to spawn.");
  }

  return 0;
}
