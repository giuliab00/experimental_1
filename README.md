Assignment 1 of Experimental Robotics Laboratory
================================================

In this assignment we are given an environment where the robot start at the point (0,0) and 4 markers with a specific meaning are placed.
We are requested to reach all the markers by controlling the camera and the navigation of the robot. This must be done for either the simulation and the real robot.\
Here is explained in detail the behaviour for the simulation, while at the end of this document isd possible to finde the differences with the real robot implementation.

How to download
----------------------

In order to run the solution it is necessary to ...

How to run the solution
----------------------

First of all is necessary:

```bash

```


Architecture and Pseudocode
----------
In order to achieve the solution it has been thpought of the following architecture:\
image architecthure\
little explanation

### navlog

```python

Initialize ROS node

Define constants and variables

Initialize ROS publishers, subscribers and variables

Define a class for navigation logic:

    Initialize variables for distance, angle, acknowledgment and state
    Set control parameters
    Initialize publishers and subscribers

    Callback for markerPose subscription:
        Acknowledge marker detection
        Update distance and angle

    Change state function:
        State 0: Rotation
            If marker is detected:
                If there is misalignment:
                    Move to state 2 for correction
                Else:
                    Move to state 1 for distance control
            If marker is NOT detected:
                Robot turn on itself

        State 1: Distance control
            If robot too far from marker:
                Move forward
            Else:
                Stop
                Move to state 0 for rotation waiting for the next marker

        State 2: Error correction
            If misaligned to the left:
                Turn clockwise
            If misaligned to the right:
                Turn counter-clockwise
            If well aligned:
                Move to state 1 for distance control

    Routine function:
        List of marker IDs

        Until there are markers to reach:
            Publish marker ID
            Set state 0
            Move backward to better see the marker
            Stop
            Until the marker is not reached:
                Iterate the control

            Remove the reached token from the list
            Set the control values

        When finished, the robot spins

Main:
    
    Wait for other nodes to initialize properly
    Create and spin the controller node
    Spinning thread to ensure that ROS callbacks are executed
    Start the logic node routine
    On shutdown log a message

```

### markerDetector Node

```    cpp

Inclue needed library
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
                    compute distance among marker and camera center
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

}
main(){
    init ros Node
    create markerDetector node
    ros spin node
}

```

Simulation VS Real World
-------------------------
Differencies in the real world implementation

Video
----------------------
Here it is possible to find the video showing respectively the behaviour with the simulation\
video1\
and the real robot\
video2

Drawback and Possible improvements
-------------------------
Transformation matrix !!!
