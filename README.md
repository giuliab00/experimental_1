Assignment 1 of Experimental Robotics Laboratory
================================================

The goal of this assignment is to work with a rosbot by moving it in the environment to find and reach 4 different Aruco markers. This has been done firstly in a simulation environment using gazebo and a rosbot with a fixed camera, then this behaviour has been tested and modified a little to work with a real rosbot in the laboratorium. Lastly in the simulation the model of the robot has been modified in order to have a rosbot with a rotating camera in the simulation.

Here is explained in detail the behaviour of the algorithms, while at the end of this document it is possible to find the differences between the the real robot implementation and the simulation.

How to download
----------------------

In order to run the solution it is necessary to have the following ROS package:

* OpenCV: that must be the same version of your ros, in our case noetict If you are downloading on the rosbot you don't need it since it's already there.

```bash
git clone https://github.com/ros-perception/vision_opencv
git checkout noetic
```
* ArUco: in order to have the models of the marker in the simulation and all the libraries provided by Aruco to recognize the markers. The following is for ROS noetic
```bash
git clone https://github.com/CarmineD8/aruco_ros
```
In order to have the marker visible in the gazebo simulation move the folder **models** into the folder .\gazebo

If instead we are working with the real rosbot, which are provided with ROS melodic the Aruco package to download is the following
```bash
git clone https://github.com/pal-robotics/aruco_ros
git checkout melodic-devel
```

* Rosbot: For the simulation with the fixed camera is important to have the model of the rosbot. This can be obtained with the following lines. Once again remember to branch to the correnct implementation for your ROS version.
```bash
git clone https://github.com/husarion/rosbot_ros
git checkout noetic
```

Lastly you can finally download our package. 
```bash
git clone https://github.com/giuliab00/experimental_1
```

It contains two branches:
* **main**: for the rosbot and the implentation with the fixed camera. 
* **simulation** for the simulation with the rotating camera. 

So rember to branch on the interessed one.


Run with the ROSBOT
-----------------------------
### How to run the solution

If you want to run the package with the rosbot there are two possible way:
* download and install this package inside the rosbot, taking into account the rosbot is provided with ROS melodic and the OpenCV libraries are already there so the only things to download are:
    * Aruco for Ros melodic
    * This package and checkout to the main branch
* share the ROS master with the rosbot and run this code from your pc. To do this you need to be connected on the same network and adjust the ROS_MASTER ip to be the same of the rosbot.

To run the code in both case just execute this command on the bash :

```bash
roslaunch experimental_1 laboratorium.launch
```

The two nodes are now running and the rosbot will start looking for marker and reach them.

### Architecture and Pseudocode
In order to achieve the solution it has been thought of the following architecture:\
![architecture 1](https://github.com/giuliab00/experimental_1/assets/114082533/d2a204b4-cba2-49c8-89d2-11cba2d59665)


There are two node: the **navlog** Node that is the one in charge of controlling the behaviour of the rosbot and the **marker Detecor** Node that is the one in charge of recognizing the markers. Now let's see in the detail how this work and the main differences with the Simulation with rotating camera.  

#### navlog Node
The NavLog Node is the one in charge of controlling the behaviour of the robot by publishing the ID of the marker to found, make the rosbot rotate until the marker isn't found. Once the marker is found it makes the rosbot align to the ceneter of marker and go forward to reach it. This behaviour have been implemented in a python script (found in the scripts folder with the name nav2.py).

```python

Initialize ROS node

Define a class for navigation-logic:

    Initialize variables for distance, angle, acknowledgment, state, velocities,
    Initialize publishers and subscribers

    Callback for markerPose subscription:
        if(marker found  == marker to found):
            Acknowledge marker detection
            Update distance and angle
        else
            marker not found 
            update distance

    Change state function:
        State 0: Look for marker
            if marker is detected:
                move to state 1
            else marker is NOT detected:
                Robot turn on itself

        State 1: Reach marchek
            if robot reach the marker:
                Stop
                Move to state 0
            elif misaligned to the left:
                turn left
            elif misalignment to the right:
                turn right
            else:
                go forward


    Routine function:
        List of marker IDs

        Until there are markers to reach:
            Publish marker ID and remove from the list
            Set state 0
            Move backward to better see the marker
            Stop
            Until the marker is not reached:
                Iterate the control
        When finished, the robot spins
        notify markerDetecto to finish

Main:
    
    Wait for other nodes to initialize properly
    Create and spin the controller node
    Spinning thread to ensure that ROS callbacks are executed
    Start the logic node routine
    shutdown the node

```

#### markerDetector Node
This node is the one recognizing marker and computing the values to tell the navlog about the distance between the rosbot and the marker. To detect the marker the ArUco marker detector has been used, then if the marker to found has been detect the distance between it's center and the camera center and the dimension of pixel of the side of the marker are computed. To comunicate with the navlog Node a custom message is published containing, the id of the marker found, an ack, the size of the side and the distance between centers. 

```    cpp
Include needed library
Define a MarkerDetector Class {
    Initialize variable for aruco marker detection (detector, marker size, camera parameter), CV image,  
    Initialize publishers and subscriber

    image_callback() function {
        create a cv_bridge
        try{
            copy image
            get center of the camera
            clear detected_markers list
            detect marker
            for(marker in detected_markers){
                draw detected marker on image
                if(marker.id == marker_to_ found.id){
                    get center of the marker
                    compute distance between marker and camera center along x axis
                    compute marker size dimension in pixel
                    set msg field
                    pusblish(msg)
                }
            }
            Visualize Camera POV 
        }
        catch{
            error
        }
    }

    find_marker_callback() function{
        set the id of the marker to found to the received one
    }

    camera_info_callback() function {
        set the Aruco Camera Parameter from camera info
   }

   exit_callback() function {
        shutdown the node
   }

}
main(){
    init ros Node
    create markerDetector node
    ros spin node
}

```

### Video
Here it is possible to find the video showing the rosbot in action.


https://github.com/giuliab00/experimental_1/assets/114082533/557a6603-cb2d-4cb0-8ea1-5774017435cc


As seen in the video the rosbot is a little slow but this can be adjust by modifying the values of the velocity variables initialized in the navlog node.

SIMULATION with rotating Camera
------------------------------------

### How to run the solution

If you have followed the previous steps, and are in the simulation branch it is possible to start the simulation with the rotating camera using the following command:

```bash
roslaunch experimental_1 run.launch
```
to properly execute the launch file it's important to have xterm because it allows to have each node in a single window and see clearly the output of each node. To download xterm write this line on your terminal
```bash
sudo apt-get install xterm
```

### Architecture and Pseudocode

In order to achieve the solution it has been thought of the following architecture:\
![architecture2](https://github.com/giuliab00/experimental_1/assets/114082533/bf7abbab-11e5-4ba6-a6c7-73cadecccda3)

In this case the **geometry** node is added to the previous architecture and is the the one in charge of computing the misalignmnet between the camera and the body frame. Regarding the **navlog** node and the **markerDetector** node they have the same behaviour as before. The **navlog** Node that is the one in charge of controlling the behaviour of the rosbot and the **marker Detecor** Node that is the one in charge of recognizing the markers.

#### Modification to make the camera rotate
To make the camera rotate with respect to the body some modification have been made:
* Firstly the urdf of the rosbot need to be changed in particular the rosbot.xacro and the rosbot.gazebo file :
    * rosbot.xacro: the joint connecting the camera to the body wich by default is fixed need to be continuos to allow a rotation. Then to motorize this joint a trasmission need to be add;
    * rosbot.gazebo : here the plug in for the control of the motorize joint is added;
* Then in the joint_state_controller.yaml in the config folder the controller for the camera using a PD controller is added;
* Lastly in the launch file the controller must be called and when the spawner node is called in it's args also the new controller need to be provided.

#### navlog Node
As said before the NavLog Node is the one in charge of controlling the behaviour of the robot by publishing the ID of the marker to found, make the camera rotate until the marker isn't found. Once the marker is found it makes the camera align with the body and the center of the marker and then reach the marker on a straight line. This behaviou have been implemented in a python script (found in the scripts folder with the name nav2.py).

```python

Initialize ROS node

Define a class for navlog Node:

    Initialize variables for distance, angle, acknowledgment, state, velocities, camera misalignment
    
    Initialize publishers and subscribers

    search_marker function:
        while( marker not found):
            send command to rotate camera
        stop camera
    
    Callback for markerPose subscription:
        if(marker found  == marker to found):
            Acknowledge marker detection
            Update distance and angle
        else
            marker not found 
            update distance

    Callback for camera rotation subscription:
        update the camera misalignmnet
    
    Align body function:
        while(camera not aligned):
            turn the body in the direction of the camera
            make center of camera align with center of marker
        stop rosbot and camera turning

    Routine function:
        List of marker IDs

        while(there are markers to reach):
            Publish marker ID and remove from list
            Set state 0
            search_marker()
            align_body()
            Until the marker is not reached:
                rosbot go forward
            Stop rosbot
            Set to 0 ack, distance and marker to found
        When finished, the robot spins 
        Then stop

Main:
    
    Wait for other nodes to initialize properly
    Create and spin the controller node
    Spinning thread to ensure that ROS callbacks are executed
    Start the logic node routine
    Shutdown the node

```
#### geometry Node
This node is the one publishing the rotation of the camera with respect to the body frame, this rotatation will be used by the navlog to make the rosbot and camera align. To get this rotation the functionality provided by transform are used, in particular the lookupTransform, which given the name of the two reference frame returns the transformation matrix among them. From the transformation matrix between the body and the camera frame the rotation around z (yaw) is obtained.

```    cpp
Include needed library
Define a geometry NOde Class {
    initialize variable for Transform matrices
    initialize publisher and ros timer

    printTransform(tf :: transfor Matrix) {
        print transformation matrix
    }
		
    timerCamCallback(){
        getTransform(body, camera);
        get yaw from transform matrix
        publish yaw
    }

    getTransform(refFrame, childFrame, transformMatrix){
        if(wait for Transform take too much time){
            Error
            return false
        }
        else{
            try{
                lookupTransform(refFrame, childFrame, ros::Time(0), transform)
            }
            catch {
                Error
            }
            return true
            }
        }
    exit_callback() function {
        shutdown the node
    }
};

main(){
    init ros Node
    create geometryNode node
    ros spin node
}
```

#### markerDetector Node
This node is the one recognizing marker and computing the values to tell the navlog about the distance between the rosbot and the marker. It's behaviour it's the same as the one implemented to run on the rosbot the main difference is the topic to get the camera images that is *"/camera/color/image_raw"* for the simulation and *"/camera/rgb/image_raw"*

### Video
Here it is possible to find the video showing the behaviour in the simulation with the rotating camera



https://github.com/giuliab00/experimental_1/assets/114082533/bd02b73e-6956-45aa-b1c5-8841b2ffd713



Simulation VS Real World
-------------------------
The main differences between the two implementations are:
* The addition of the geometry Node to know ho wthe camera is rotate
* the modification of the rosbot model in order to have the rotating camera and controlling it
* The change of topic to look in markerDetector depending on the simulation or the real rosbot
* change in the behaviour of the navlog 


Drawback and Possible improvements
-------------------------
There are different improvement regarding both the simulation and the real rosbot.

Regarding the simultaion one main problem was the detection of marker 12 after reaching marker 11 due to the shadow of the gray boxes. To overcome this problem we change the color of the boxes in the simulation to be white in this way the robot detect the marker easier because the shadow are not so dark anymore and make the square of the marker recognizible. Another solution could be to make the robot take a step back after reaching each marker. To be even more precise combining the two tecniques could be a good idea.

The simulation with the fixed camera has been developed in an initial phase of the project but right now no launch file is provided to see it. But can be easily obtained in the main function by launch a simulation with the standard rosbot model provided by huarison and the same node used by the real robot by remapping the topic look by the markerDetecor to the one of the simulation. 

One interesting improvements will be to use transformation matrix to know the position of the marker with respect to the robot. This will allow a more efficient control. This will require to work with ROS transform, by adding some publisher of the transformation of the rosbot and it's camera to know it's position. Once the marker is detected the ArUco library allow to know the position of then marker with respect to the camera, then by doing some multiplication between transformation matrices is possible to obtain the position of the marker with respect to the rosbot or even with respect to the world. 

Regarding the control would be also interesting to se a control proportional to the error, so that the rosbot will go faster when is distant and slower once it almost reach the marker.

