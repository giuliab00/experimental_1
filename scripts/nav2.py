#!/usr/bin/env python3

# Import necessary ROS and Python packages
import rospy
from experimental_1.msg import markerDistance
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, Int32, Float64
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
		self.camera_theta = 0.0
		self.l_vel = 0.2
		self.a_vel = 0.2

		#Publish to cntrol the camera
		self.camera_pub = rospy.Publisher("/camera_velocity_controller/command", Float64, queue_size=10)

		# Publish marker ID goal
		self.pub_marker_id = rospy.Publisher("/requestMarkerId", Int32, queue_size=10)

		# Publisher for /cmd_vel
		self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

		# Subscribe to the topic for goal position from marker detection
		self.sub_marker_dist = rospy.Subscriber("/markerDistance", markerDistance, self.clbk_vision)

		#Subscriber to know camera misalignment
		self.camera_sub = rospy.Subscriber("/camera_yaw",Float32, self.clbk_camera)

		self.exit_pub = rospy.Publisher("/task_complete", Bool, queue_size = 10)

	"""
		This method is used to control the camera for searching the actual desired
		aruco marker.
	"""
	def search_marker(self):
		cam_msg = Float64()
		cam_msg.data = 0.1
		self.camera_pub.publish(cam_msg)
		#Searching the marker in the evironment
		rospy.loginfo("searching marker ID: %d" % self.to_found)
		while(not(self.ack)):
			#waiting...
			time.sleep(0.01)
		#Stop the camera
		rospy.loginfo("founded!")
		cam_msg.data = 0.0
		self.camera_pub.publish(cam_msg)

	"""
		This callback function is used to retrive the iformation
		about the actual desired marker
	"""
	# Callback for markerPose subscription
	def clbk_vision(self, msg):
		# Acknowledge marker detection
		if(msg.marker_id == self.to_found):
			self.ack = msg.ack
			# Update distance 
			self.distance = msg.l_pixel
			# Update angle 
			self.angle = msg.centerDistance
		else :
			self.ack = False
			# Update distance 
			self.distance = 0

	"""
		This callback function retrive the yaw angle of the camera frame
		with respect to the body frame
	"""
	#Callback camera_rotation
	def clbk_camera(self, msg):
		self.camera_theta = msg.data

	"""
		This function handle to align the body frame to the camera frame.
		In function of the smaller angle of rotation the RosBot start to rotate
		while the camera try to stay aligned with the marker pointed
	"""
	def align_body(self):
		robot_cmd = Twist()
		cam_cmd = Float64()
		rospy.loginfo("Pointing the marker found and align the body to the camera")
		while(abs(self.camera_theta) > 0.02):
			if(self.camera_theta > math.pi):
				#turn right the robot 
				robot_cmd.angular.z = -self.a_vel
			else:
				#turn left the robot		
				robot_cmd.angular.z = self.a_vel
			#conditions for traking the center of the marker
			if((self.angle/self.distance) > 0.1):
				#turn right the camera
				cam_cmd.data = self.a_vel
			elif((self.angle/self.distance) < -0.1):
				#turn left the camera			
				cam_cmd.data = -self.a_vel
			self.camera_pub.publish(cam_cmd)
			self.cmd_pub.publish(robot_cmd)
		#Stop the robot
		cam_cmd.data = 0.0
		robot_cmd.angular.z = 0.0
		self.camera_pub.publish(cam_cmd)
		self.cmd_pub.publish(robot_cmd)

	"""
		Main function containing the marker IDs and the control logic of the job
		For each marker, firstly the RosBot search it by selecting the right id,
		then it align the body to the camera and at the end it reach the marker 
		taking care of the dimension of the marker inside the camera image matrix
	"""
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
			cmd = Twist()
			# Untill the marker is not reached
			self.search_marker()
			self.align_body()
			rospy.loginfo("Reach marker ID: %d" % self.to_found)
			while self.distance < 200:
				#Go Straight
				cmd.linear.x = self.l_vel
				self.cmd_pub.publish(cmd)
			cmd.linear.x = 0.0
			self.cmd_pub.publish(cmd)
			# Remove the reached token from the list
			# reset callback values 
			self.to_found = 0
			self.ack = False
			self.distance = 0
		#When finished Robot spin
		cmd.angular.z = 0
		self.cmd_pub.publish(cmd)
		# Notify markerDetector to finish 
		msg = Bool()
		msg.data = True
		self.exit_pub.publish(msg)

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
	rospy.signal_shutdown('Shutting down the node')
	
if __name__ == '__main__':
    main()

