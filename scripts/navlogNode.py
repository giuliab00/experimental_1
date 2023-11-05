#!/usr/bin/env python

import rospy
import actionlib
import actionlib.msg
import experimental_1.msg
from experimental_1.msg import markerDistance
from tf import transformations
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, Int32
from threading import Thread
import math
import time

goal_position = Pose2D()

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

class NavLogNode:

	def __init__(self):

		#init node
		rospy.init_node("navlogNode")
		#init var
		self.distance = 0.0
		self.angle = 0.0
		self.ack = False
		self.state = 0
		self.current_pose = Pose2D()
		
		#parameters for control
		self.Kp_d = 0.2
		self.Kp_a = 0.2
		self.limit_vel = 0.6
		self.limit_wl = 0.6
		self.limit_wr = -0.5
		self.dist_threshold = 0.07
		self.ang_threshold = 0.08

		# Publish marker ID goal
		self.pub_marker_id = rospy.Publisher("/requestMarkerId", Int32, queue_size=10)

		#Publisher for /cmd_vel
		self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

		# Subscribe to the topic for goal position from marker detection
		self.sub_marker_dist = rospy.Subscriber("/markerDistance", markerDistance, self.clbk_vision)
		
		#Subscribe to /odom
		self.odom_sub = rospy.Subscriber("/odom", Odometry, self.clbk_odom)

	#Callback for markerPose subscription
	def clbk_vision(self, msg):
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
			else:
				cmd = Twist()				
				cmd.angular.z = 0.1
				self.cmd_pub.publish(cmd)

		#stato 1 controllo distanza
		if self.state == 1:			
			if self.distance < 200 :
				#dobbiamo andare avanti piano piano
				#matte metti il controllo
				cmd = Twist()
				cmd.linear.x = 0.1
				self.cmd_pub.publish(cmd)
			else:
				cmd = Twist()
				self.cmd_pub.publish(cmd)
				self.state = 0				

		#stato 2 correzione errore
		if self.state == 2:
			if self.angle < -40:
				#controllo giro verso dx
				cmd = Twist()
				cmd.angular.z = -0.1
				self.cmd_pub.publish(cmd)  
			elif self.angle > 40:
				#controllo giro verso sx
				cmd = Twist()
				cmd.angular.z = 0.1
				self.cmd_pub.publish(cmd)
			else:
				self.state = 1
				

# Main routine
	def routine(self):
		#lista marker
		marker_list = [11,12,13,15]
		#while lista piena
		rospy.loginfo("Starting...")
		while(marker_list):
			#publish marker id
			self.pub_marker_id.publish(marker_list[0])
			rospy.loginfo("Sent marker ID: %d" % marker_list[0])
			#settiamo stato a 0
			self.state = 0
			cmd = Twist()
			cmd.linear.x = -0.1
			self.cmd_pub.publish(cmd)
			time.sleep(1)
			cmd.linear.x = 0
			self.cmd_pub.publish(cmd)
			#while marker not found
			while(self.distance < 200):
				#change state
				self.change_state()
			#tolgo marker dalla lista
			marker_list.pop(0)
			self.ack = False
			self.distance = 0
		cmd = Twist()
		cmd.angular.z = 5
		self.cmd_pub.publish(cmd)
			
	def clbk_odom(self, msg):
		
		# We retrieve data from the '/odom' topic about the robot's current position
		self.current_pose.x = msg.pose.pose.position.x
		self.current_pose.y = msg.pose.pose.position.y
		
		# We retrieve data from the '/odom' topic about the robot's current orientation
		quaternion = (msg.pose.pose.orientation.x,
					msg.pose.pose.orientation.y,
					msg.pose.pose.orientation.z,
					msg.pose.pose.orientation.w)
		
		euler = transformations.euler_from_quaternion(quaternion)
		
		self.current_pose.theta = euler[2]

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
	time.sleep(1)
	logic.routine()

	# On shutdown...
	rospy.loginfo("Shutdown logic node...")

if __name__ == '__main__':
	main()

