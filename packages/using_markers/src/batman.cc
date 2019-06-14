#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

const double XMIN = -7.0;
const double XMAX = 7.0;
const double STEP = 0.01;


int main(int argc, char** argv){
  ros::init(argc,argv,"batman");
  ros::NodeHandle nh;
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker",10);

  ros::Rate rate(10);

  int i = 0;
  bool add_flag = true;

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
    points.color.r = 1.0f;
    points.color.b = 1.0f;
    points.color.g = 1.0f;
    
    if ( add_flag ){
      ++i; 
      if ( i == 10 ){
        add_flag = false;
      }
    } else {
      --i;
      if ( i == 0 ){
        add_flag = true;
      }
    }
    points.color.a = i*0.1f;

    // msg object
    geometry_msgs::Point p;

    double x, y, n = 5;
		for ( x = XMIN ; x <= XMAX ; x += STEP ){
			// eq1
			y = 2*sqrt(-abs(abs(x)-1)*abs(3-abs(x))/((abs(x)-1)*(3-abs(x))))*(1+abs(abs(x)-3)/(abs(x)-3))*sqrt(1-(x/7)*(x/7))+(5+0.97*(abs(x-.5)+abs(x+.5))-3*(abs(x-.75)+abs(x+.75)))*(1+abs(1-abs(x))/(1-abs(x)));
			if ( !isnan(y) ){
				p.x = x;
				//p.y = y;
        p.z = y+n;
        points.points.push_back(p);
			}

			// eq2
			y = -3*sqrt(1-(x/7)*(x/7))*sqrt(abs(abs(x)-4)/(abs(x)-4));
			if ( !isnan(y) ){
				p.x = x;
				//p.y = y;
        p.z = y+n;
        points.points.push_back(p);
			}

			// eq3
			y = abs(x/2)-0.0913722*(x*x)-3+sqrt(1-(abs(abs(x)-2)-1)*(abs(abs(x)-2)-1));
			if ( !isnan(y) ){
				p.x = x;
				//p.y = y;
        p.z = y+n;
        points.points.push_back(p);
			}

			// eq4
			y = (2.71052+(1.5-.5*abs(x))-1.35526*sqrt(4-(abs(x)-1)*(abs(x)-1)))*sqrt(abs(abs(x)-1)/(abs(x)-1))+0.9;
			if ( !isnan(y) ){
				p.x = x;
				//p.y = y;
        p.z = y+n;
        points.points.push_back(p);
			}
    }

    marker_pub.publish(points);

    rate.sleep();
  }

  return 0;
}
