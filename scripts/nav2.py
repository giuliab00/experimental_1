#!/usr/bin/env python

# Import necessary ROS and Python packages
import rospy
from experimental_1.msg import markerDistance
from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Bool, Float32, Int32
from threading import Thread
import math
import time


class NavLogNode:

	def __init__(self):

		# Initialize the ROS node
		rospy.init_node("navlogNode")

		# Initialize variables for distance, angle, acknowledgment and state
		self.distance = 0.0
		self.angle = 0.0
		self.ack = False
		self.to_found = 0
		self.state = 0
		self.l_vel = 0.05
		self.a_vel = 0.05

		# Publish for /requestMarkerId
		self.pub_marker_id = rospy.Publisher("/requestMarkerId", Int32, queue_size=10)

		# Publisher for /cmd_vel
		self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

		# Subscribe to the topic for goal position from marker detection
		self.sub_marker_dist = rospy.Subscriber("/markerDistance", markerDistance, self.clbk_vision)

	# Callback for markerPose subscription
	def clbk_vision(self, msg):
		if(msg.marker_id == self.to_found):
			# Acknowledge marker detection
			self.ack = msg.ack
			# Update distance 
			self.distance = msg.l_pixel
			# Update angle 
			self.angle = msg.centerDistance
		else :
			#wrong/old marker found 
			self.ack = False
			self.distance = 0

	#State 0: Search
	#State 1: Reach
	def change_state(self):
		cmd = Twist()
		if(self.state == 0):
			if(self.ack):
				self.state = 1
				cmd.angular.z = 0
				self.cmd_pub.publish(cmd)
			else:
				cmd.angular.z = self.a_vel
				self.cmd_pub.publish(cmd)
		elif(self.state == 1):
			#Reach
			if(self.distance >= 200):
				self.state = 0
				cmd.angular.z = 0
				cmd.linear.x = 0
				self.ack = False
				self.distance = 0
				self.cmd_pub.publish(cmd)
			elif((self.angle/self.distance) > 0.2):
				#turn right
				cmd.angular.z = self.a_vel
			elif((self.angle/self.distance) < -0.2):
				#turn left					
				cmd.angular.z = -self.a_vel
			else:
				#go forward
				cmd.angular.z = 0.0
				cmd.linear.x = self.l_vel
			self.cmd_pub.publish(cmd)

	def routine(self):
		# List of marker IDs
		marker_list = [11, 12, 13, 15]

		rospy.loginfo("Starting...")

		# Until there are marker to reach
		while marker_list:

			# Publish marker ID
			self.to_found = marker_list.pop(0)
			self.pub_marker_id.publish(self.to_found)
			self.distance = 0
			rospy.loginfo("Sent marker ID: %d" % self.to_found)

			# Set state 0
			self.state = 0
			
			# Move backword to better see the marker
			cmd = Twist()
			cmd.linear.x = -self.l_vel
			self.cmd_pub.publish(cmd)
			time.sleep(1)
			# Stop
			cmd.linear.x = 0
			self.cmd_pub.publish(cmd)
			
			# Untill the marker is not reached
			while self.distance < 200:
				# Iterate the control
				self.change_state()
			# Remove the reached token from the list
			#marker_list.pop(0)
			# Set the control values 
			self.to_found = 0
			
		#When finished Robot spin 
		cmd = Twist()
		cmd.angular.z = 3
		self.cmd_pub.publish(cmd)
		time.sleep(5)
		cmd.angular.z = 0
		self.cmd_pub.publish(cmd)
		

# Main routine
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
    rospy.signal_shutdown('Node shutting down')

if __name__ == '__main__':
    main()

