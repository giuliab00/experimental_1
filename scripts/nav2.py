#!/usr/bin/env python3

# Import necessary ROS and Python packages
import rospy
import actionlib
import actionlib.msg
import experimental_1.msg
from experimental_1.msg import markerDistance
from tf import transformations
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, Int32, Float64
from threading import Thread
import math
import time

# Initialize ROS publishers, subscribers and variables
vel_pub = None
srv_ask_pixels = None
robot_pose_sub = None
movement = None

class NavLogNode:

	def __init__(self):

		# Initialize the ROS node
		rospy.init_node("navlogNode")

		# Initialize variables for distance, angle, acknowledgment and state
		self.distance = 0.0
		self.angle = 0.0
		self.ack = False
		self.to_found = 0
		self.current_pose = Pose2D()
		self.camera_theta = 0.0

		#Publish to cntrol the camera
		self.camera_pub = rospy.Publisher("/camera_velocity_controller/command", Float64, queue_size=10)

		# Publish marker ID goal
		self.pub_marker_id = rospy.Publisher("/requestMarkerId", Int32, queue_size=10)

		# Publisher for /cmd_vel
		self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

		# Subscribe to the topic for goal position from marker detection
		self.sub_marker_dist = rospy.Subscriber("/markerDistance", markerDistance, self.clbk_vision)

		# Subscribe to /odom
		self.odom_sub = rospy.Subscriber("/odom", Odometry, self.clbk_odom)

		#Subscriber to know camera misalignment
		self.camera_sub = rospy.Subscriber("/camera_yaw",Float32, self.clbk_camera)

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


	#Callback camera_rotation
	def clbk_camera(self, msg):
		self.camera_theta = msg.data

	def align_body(self):
		robot_cmd = Twist()
		cam_cmd = Float64()
		rospy.loginfo("Pointing the marker found and align the body to the camera")
		while(abs(self.camera_theta) > 0.02):
			if(self.camera_theta > math.pi):
				#turn right the robot 
				robot_cmd.angular.z = -0.05
			else:
				#turn left the robot		
				robot_cmd.angular.z = 0.05
			#conditions for traking the center of the marker
			if((self.angle/self.distance) > 0.1):
				#turn right the camera
				cam_cmd.data = 0.05
			elif((self.angle/self.distance) < -0.1):
				#turn left the camera			
				cam_cmd.data = -0.05
			self.camera_pub.publish(cam_cmd)
			self.cmd_pub.publish(robot_cmd)
		#Stop the robot
		cam_cmd.data = 0.0
		robot_cmd.angular.z = 0.0
		self.camera_pub.publish(cam_cmd)
		self.cmd_pub.publish(robot_cmd)

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
				cmd.linear.x = 0.05
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

	def clbk_odom(self, msg):
		# Retrieve robot's current position and orientation from /odom topic
		self.current_pose.x = msg.pose.pose.position.x
		self.current_pose.y = msg.pose.pose.position.y
		quaternion = (
		    msg.pose.pose.orientation.x,
		    msg.pose.pose.orientation.y,
		    msg.pose.pose.orientation.z,
		    msg.pose.pose.orientation.w
		)
		euler = transformations.euler_from_quaternion(quaternion)
		self.current_pose.theta = euler[2]

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

if __name__ == '__main__':
    main()

