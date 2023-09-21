#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "phasespace/Markers.h"
#include "phasespace/Cameras.h"

using namespace std;

void error_cb(const std_msgs::String::ConstPtr &msg)
{
  ROS_INFO_STREAM("message=" << msg->data.c_str());
}

void camera_cb(const phasespace::Cameras &msg)
{
  for(size_t i = 0; i < msg.cameras.size(); i++)
    ROS_INFO_STREAM("id=" << msg.cameras[i].id
                    << " pos[" << msg.cameras[i].x << " " << msg.cameras[i].y << " " << msg.cameras[i].z
                    << "] rot[" << msg.cameras[i].qw << " " << msg.cameras[i].qx << " " << msg.cameras[i].qy << " " << msg.cameras[i].qz
                    << "] cond=" << msg.cameras[i].cond);
}

void marker_cb(const phasespace::Markers &msg)
{
  for(size_t i = 0; i < msg.markers.size(); i++)
    ROS_INFO_STREAM("t=" << msg.markers[i].time
                    << " id=" << msg.markers[i].id
                    << " [" << msg.markers[i].x << " " << msg.markers[i].y << " " << msg.markers[i].z
                    << "] cond=" << msg.markers[i].cond);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle nh;
  //ros::Subscriber markerSub = nh.subscribe("phasespace_markers", 1000, marker_cb);
  //ros::Subscriber cameraSub = nh.subscribe("phasespace_cameras", 1000, camera_cb);
  ros::Subscriber errorSub = nh.subscribe("phasespace_errors", 1000, error_cb);
  ros::spin();  
  return 0;
}
