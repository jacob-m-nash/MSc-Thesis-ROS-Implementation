# Phasespace Ros node

this package contains a wrapper of the phasespace api to track rigid boddies. The phasespace api used in this package is **5.3**  

# Installation

assuming that ROS noetic with catkin-tools has been installed in your machine, to install the package it is suffcient to create a catkin workspace and after that run:

```
catkin build phasespace
```

# Usage

to run the package first you need to ensure that the client computer is connected to the same subnet to which the phasespace is connected to. to run the package just run:

```
rosrun phasespace phasespace_node $PHASESPACE_IP rigid_object.json
```

instead of **$PHASESPACE_IP** you should use the current ip of the phasespace on the subnet (to know which is the phasespace ip there exist plenty of tools booht fo ubuntu and android (NMAP on ubuntu netananalyzer on android), or accessing the router and check thwe connected device will do the job.

rigid_object.json contains all the information about your rigid body. for tracking your rigid you have to create a new one usning the LED markers on the rigid body you want to track. In order to create the json file you need to use the master app that can be downloaded from [here](https://customers.phasespace.com/anonymous/Software/5.3/). Download the master client and in the bin folder you can find the master eexecutable.

once you run it you have to select the markers on the rigid body that you want to track and after clicking on the left **OWL Trackers** and then click on the **create** button. In order to make the json file visible to the package move the file inside the **rigid_body_objects** in the package folder

Master app interface: 
![images that show the master app interface to create a rigid body json file](https://github.com/RPL-CS-UCL/phasesapce/blob/main/images/create_json_rigid_body.png "master interface")

the ros node opens 3 topic:

1.  **\phasespace_cameras** which contains the rototraslation of the camera w.r.t. the reference frame
2.  **\phasespace_markers** which contains the rototraslation of each LED marker w.r.t. the reference frame
3.  **\phasespace_rigids** which contains the rototraslation of the **single** rigid body in the scene

# TO DO

1. adding multi rigid body support
2. adding scan function to automatically detect the phasespace server on the router 
3. adding flag to selc what information stream (now everything is published)
