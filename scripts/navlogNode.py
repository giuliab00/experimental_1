#!/usr/bin/env python

import rospy
import actionlib
import actionlib.msg
import experimental_1.msg
from experimental_1.msg import markerDistance
from tf import transformations
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32
from threading import Thread
import math

goal_position = Pose2D()
current_pose = Pose2D()
current_pose.x = 0.0
current_pose.y = 0.0
current_pose.theta = 0.0

MarkerID = 11
Kp_d = 0.2
Kp_a = 0.2
limit_vel = 0.6
limit_wl = 0.6
limit_wr = -0.5
dist_threshold = 0.07
ang_threshold = 0.08
correction = True

vel_pub = None
srv_ask_pixels = None
robot_pose_sub = None
movement = None

class NavLogRobot:

    def __init__(self):

        #init node
        rospy.init_node("navlogNode")

        #init var
        self.distance = 0.0
        self.angle = 0.0
        self.ack = False
        self.state = 0

        # Publish marker ID goal
        self.pub_marker_id = rospy.Publisher("/requestMarkerId", Int32, queue_size=10)

        # Subscribe to the topic for goal position from marker detection
        self.sub_marker_dist = rospy.Subscriber("/requestMarkerId", markerDistance, self.clbk_vision)
        
    
    # Callback for markerPose subscription
    def clbk_vision(self, msg):
        rospy.loginfo("Got distance from vision")
        #acknowledge
        self.ack = msg.ack
        #distanza se <200 vado avanti
        self.distance = msg.l_pixel
        #angolo deve arrivare a 40 o -40
        self.angle = msg.centerDistance

    def change_state(self):
        #stato 0 rotazione
        if self.state == 0:
            #controllo ruoto su me stesso
            if self.ack:
                #misalignment da lontano è sui 20 e nella distanza giusta è circa 40 
                if ((self.angle < -40) or (self.angle > 40)):
                    self.state = 2 
                else:
                    self.state = 1

        #stato 1 controllo distanza
        if self.state == 1:
            if self.distance < 200 :
                #dobbiamo andare avanti piano piano
                #matte metti il controllo
            else:
                #remove marke from marker list
                #ricominiciamo a ruotare
                self.state = 0
                #mandiamo nuovo markereId
                #pub

        #stato 2 correzione errore
        if self.state == 2:
            if self.angle < -40:
               #controllo giro verso dx
               #  
            elif self.angle > 40:
                #controllo giro verso sx
               
# Main routine
    def routine(self):
        #lista marker
        #while lista piena
            #publish marker id
            #settiamo statpo a 0
            #while marker not found
                #change state
                # tolgo marker dalla lista

        
# Main function
def main():
    # Wait for other nodes to initialize properly
    time.sleep(1)

    # Create and spin the controller node
    logic = NavLogNode()

    # Spinning thread to ensure that ROS callbacks are executed
    spin_thread = Thread(target=rospy.spin)
    spin_thread.start()

    # Start the logic node routine
    logic.routine()

    # On shutdown...
    rospy.loginfo("Shutdown logic node...")

if __name__ == '__main__':
    main()

    
        
