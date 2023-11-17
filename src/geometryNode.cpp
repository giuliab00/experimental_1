#include <iostream>
#include "ros/ros.h"
#include <sstream>
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"

/*Transforms*/
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <unistd.h>

/*      subscriers:
                - /task_complete                        : topic for close the node
 * 
 * 	publishers:
 * 		- /camera_yaw				: yaw angle of the camera frame
 * 		
 */

class GeometryNode {
	
	//Attributes
	private:

		/*Node handler*/
		ros::NodeHandle nh_;

		/*Subscribers and Publishers*/
		ros::Publisher cameraRot_pub_;

		/*Timers*/
		float timer_camera_period = 0.1;
		ros::Timer camera_timer_;

		//Trasformation matrix of camera w.r.t. the simulated world
		tf::Transform wTb;

		//Transformation matrix of the <camera> w.r.t. the <body>
		tf::StampedTransform bTc;

		tf::TransformListener _tfListener;
		std::string camera_frame;
  		std::string body_frame;
  		
  		ros::Subscriber killer_sub_;
		
	public:
		/*Constructor*/
		GeometryNode(): nh_("~") {
			/*Frames*/
			camera_frame = "camera_link";
			body_frame = "body_link";

			//Publisher for rotation of the camera
			cameraRot_pub_ = nh_.advertise<std_msgs::Float32>("/camera_yaw", 10);
		
			/*Timer for retrive the yaw of the camera*/
			camera_timer_ = nh_.createTimer(ros::Duration(timer_camera_period), &GeometryNode::timerCamCallback, this);
			
			killer_sub_ = nh_.subscribe("/task_complete",1, &GeometryNode::killer_callback, this);
		}

		/*
			Debug function to print on the terminal a specific Transformation 
			function
		*/
		void printTransform(const tf::Transform& transform) {
			tf::Vector3 translation = transform.getOrigin();
			tf::Quaternion rotation = transform.getRotation();

			ROS_INFO("Translation (x, y, z): (%.2f, %.2f, %.2f)", translation.x(), translation.y(), translation.z());
			ROS_INFO("Rotation (x, y, z, w): (%.2f, %.2f, %.2f, %.2f)", rotation.x(), rotation.y(), rotation.z(), rotation.w());
		}

		/*
			callback function to retrive with "getTransform" function the transformation
			matrix of the camera frame with respect to the body frame, then from the Rotation matrix 
			it compute the Euler's angle and publish only the Yaw angle 
		*/
		void timerCamCallback(const ros::TimerEvent&){
			getTransform(body_frame, camera_frame, bTc);
			tf::Matrix3x3 wRc = static_cast<tf::Transform>(bTc).getBasis();
			tfScalar Y;	tfScalar P;	tfScalar R;
			wRc.getRPY(R,P,Y);
			std_msgs::Float32 send;
			send.data = static_cast<float>(Y); //rad
			cameraRot_pub_.publish(send);
		}

		/*
			By tflistemer it search for the trasformation function of interested frames
		*/
		bool getTransform(const std::string& refFrame, const std::string& childFrame, tf::StampedTransform& transform) {
			std::string errMsg;

			if (!_tfListener.waitForTransform(refFrame, childFrame, ros::Time(0), ros::Duration(0.5), ros::Duration(0.01), &errMsg)) {
				ROS_ERROR_STREAM("Unable to get pose from TF: " << errMsg);
				return false;
			}
			else {
				try {
					// get latest available
					_tfListener.lookupTransform(refFrame, childFrame, ros::Time(0), transform);
				}
				catch (const tf::TransformException& e) {
					ROS_ERROR_STREAM("Error in lookupTransform of " << childFrame << " in " << refFrame);
					return false;
				}
			}
			return true;
		}
		
		/*
		        If recived a message of finish work this callback 
		        ends the node
		*/
		void killer_callback(const std_msgs::Bool &msg){
		        if(msg.data) ros::shutdown();
		}

};

int main(int argc, char **argv){
	sleep(2);
	// init ros Node
	ros::init(argc, argv, "geometry_node");	

	//create markerDetecor
	GeometryNode node;
	
	//spin Node
	ros::spin();
}