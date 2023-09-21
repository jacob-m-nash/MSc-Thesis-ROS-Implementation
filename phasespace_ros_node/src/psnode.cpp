#include <iostream>
#include <ros/package.h>

#include "ros/ros.h"
#include "owl.hpp"
#include "std_msgs/String.h"
#include "phasespace/Camera.h"
#include "phasespace/Cameras.h"
#include "phasespace/Marker.h"
#include "phasespace/Markers.h"
#include "phasespace/Rigid.h"
#include "phasespace/Rigids.h"
#include "RSJparser.tcc"

using namespace std;


// for debugging purposes
inline void printInfo(const OWL::Markers &markers)
{
  for(OWL::Markers::const_iterator m = markers.begin(); m != markers.end(); m++)
    if(m->cond > 0)
      cout << "   " << m->id << ") pos=" << m->x << "," << m->y << "," << m->z << endl;
}

inline void printInfo(const OWL::Rigids &rigids)
{
  for(OWL::Rigids::const_iterator r = rigids.begin(); r != rigids.end(); r++)
    if(r->cond > 0)
      cout << "   " << r->id << ") pose=" << r->pose[0] << "," << r->pose[1] << "," << r->pose[2]
           << " " << r->pose[3] << "," << r->pose[4] << "," << r->pose[5] << "," << r->pose[6]
           << endl;
}


int main(int argc, char** argv)
{
  // get the owl server address through the command line
  // 'rosrun phasespace phasespace_node 192.168.1.123 object.json 1'
  string address = argc > 1 ? argv[1] : "localhost";

  // getting the name of the json file
  string json_name = argc > 2 ? argv[2] : "none.json";
  if(json_name == "none.json")
  {
    cout << "No json file provided. Exiting..." << endl;
    return 0;
  }
  // id of the body to track // TODO it does not work if the third argument is not provided is giving zero not -100 to fix
  int desired_body_to_track = argc > 3 ? atoi(argv[3]) : -100;
  //DEBUG
  cout << "desired_body_to_track: " << desired_body_to_track << endl;


  // build path to json file wich are in the rigid_objects folder
  string json_path = ros::package::getPath("phasespace") + "/rigid_body_objects/" + json_name;
  // DEBUG
  std::cout << "json_path: " << json_path << std::endl;
  // read the json file
  ifstream json_file(json_path);
  // extract string from json_file
  string json_file_str((istreambuf_iterator<char>(json_file)), istreambuf_iterator<char>());
  // DEBUG print json_file_str
  cout << "json_file_str: " << json_file_str << endl;
  // read the rigid body json file
  RSJresource  my_json(json_file_str); // str is a string containing the json file
  // DEBUG print my_json
  cout << "my_json: " << my_json.as_str()<< endl;

  // create the context
  OWL::Context owl;
  OWL::Markers markers;
  OWL::Cameras cameras;
  OWL::Rigids rigids;


  // initialize ROS
  ros::init(argc, argv, "phasespace_node");
  ros::NodeHandle nh;
  //ros::Publisher bodyPub = nh.advertise<std_msgs::String>("/number_of_bodies", 1000, true);
  ros::Publisher errorsPub = nh.advertise<std_msgs::String>("phasespace_errors", 1000, true);
  ros::Publisher camerasPub = nh.advertise<phasespace::Cameras>("phasespace_cameras", 1000, true);
  ros::Publisher markersPub = nh.advertise<phasespace::Markers>("phasespace_markers", 1000);
  ros::Publisher rigidsPub = nh.advertise<phasespace::Rigids>("phasespace_rigids", 1000);
  ros::Publisher rigid_one_Pub = nh.advertise<phasespace::Rigid>("/phasespace_body_one", 1000);

  // simple example
  if(owl.open(address) <= 0 || owl.initialize("timebase=1,1000000") <= 0) return 0;

  // create rigid bodies
  for (auto ii=my_json["trackers"].as_array().begin(); ii!=my_json["trackers"].as_array().end(); ++ii){
    uint32_t tracker_id = (*ii)["id"].as<int>();
    // DEBUG printing tracker_id
    cout << "tracker_id: " << tracker_id << endl;
    string tracker_name = (*ii)["name"].as<std::string>();
    // DEBUG printing tracker_name
    cout << "tracker_name: " << tracker_name << endl;
    owl.createTracker(tracker_id, "rigid", tracker_name);
    // adding markers to the rigid body
    for (auto it=(*ii)["markers"].as_array().begin(); it!=(*ii)["markers"].as_array().end(); ++it){
      uint32_t marker_id = (*it)["id"].as<int>();
      // DEBUG printing marker id
      cout << "marker_id: " << marker_id << endl;
      string marker_name = (*it)["name"].as<std::string>();
      // DEBUG printing marker_name
      cout << "marker_name: " << marker_name << endl; 
      string marker_pos = (*it)["options"].as<std::string>();
      // DEBUG printing marker_pos
      cout << "marker_pos: " << marker_pos << endl;
      owl.assignMarker(tracker_id, marker_id, marker_name, marker_pos);
    }
  }

  // start streaming
  owl.streaming(1);


  // main loop
  while(ros::ok() && owl.isOpen() && owl.property<int>("initialized"))
  {
      // blocking reading from phasespace system
      const OWL::Event *event = owl.nextEvent(1000);
      if(!event) continue;

      if(event->type_id() == OWL::Type::ERROR)
      {
          cerr << event->name() << ": " << event->str() << endl;
          std_msgs::String str;
          str.data = event->str();
          errorsPub.publish(str);
      }
      else if(event->type_id() == OWL::Type::CAMERA)
      {
          if(event->name() == string("cameras") && event->get(cameras) > 0)
          {
              phasespace::Cameras out;
              for(OWL::Cameras::iterator c = cameras.begin(); c != cameras.end(); c++)
              {
                  phasespace::Camera cam;
                  //
                  cam.id = c->id;
                  cam.flags = c->flags;
                  cam.x = c->pose[0];
                  cam.y = c->pose[1];
                  cam.z = c->pose[2];
                  cam.qw = c->pose[3];
                  cam.qx = c->pose[4];
                  cam.qy = c->pose[5];
                  cam.qz = c->pose[6];
                  cam.cond = c->cond;
                  //
                  out.cameras.push_back(cam);
              }
              camerasPub.publish(out);
          }
      }
      else if(event->type_id() == OWL::Type::FRAME)
      {  
          if(event->find("markers", markers) > 0)
          { 
              // DEBUG
              //cout << " " << event->type_name() << " " << event->name() << "=" << markers.size() << ":" << endl;
              //printInfo(markers);
              phasespace::Markers out;
              for(OWL::Markers::iterator m = markers.begin(); m != markers.end(); m++)
              {	  if(m->cond > 0){
		          phasespace::Marker mout;
		          mout.id = m->id;
		          mout.time = m->time;
		          mout.flags = m->flags;
		          mout.cond = m->cond;
		          mout.x = m->x;
		          mout.y = m->y;
		          mout.z = m->z;
		          out.markers.push_back(mout);
                  }
              }

              markersPub.publish(out);
          }
          if(event->find("rigids", rigids) > 0)
          { 
            // DEBUG
            //cout << " " << event->type_name() << " " << event->name() << "=" << rigids.size() << ":" << endl;
            //printInfo(rigids);
            phasespace::Rigids out;
            int count = 0;
            bool published_single_body = false;
            for(OWL::Rigids::iterator r = rigids.begin(); r != rigids.end(); r++)
            {
                phasespace::Rigid rout;
                rout.id = r->id;
                rout.time = r->time;
                rout.flags = r->flags;
                rout.cond = r->cond;
                rout.x = r->pose[0];
                rout.y = r->pose[1];
                rout.z = r->pose[2];
                rout.qw = r->pose[3];
                rout.qx = r->pose[4];
                rout.qy = r->pose[5];
                rout.qz = r->pose[6];
                
                if (count == desired_body_to_track){
                  rigid_one_Pub.publish(rout);
                  published_single_body = true;
                } 

                out.rigids.push_back(rout);
                count++;
            }
            //std_msgs::String str_body;
            //str_body.data = std::to_string(count);
            //bodyPub.publish(str_body);
            rigidsPub.publish(out);
            if(!published_single_body){
              std::cout<<"the id= " << desired_body_to_track << " is not in the list of bodies"<<std::endl;
            }
          }
      }
  } // while

  owl.done();
  owl.close();
  
  return 0;
}
