#include <iostream>
#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"

#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"

#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "experimental_1/markerDistance.h"
#include <cmath>
#include <opencv2/highgui.hpp>
#include <aruco_ros/aruco_ros_utils.h>

#include "opencv2/core/mat.hpp"


#include <sstream>

/*  subscriers:
 * 		-/camera/color/image_raw
 * 		-/camera/depth/image_raw
 * 		-/requestMarkerId
 * 
 * 	publishers:
 * 		-/markerDistance
 * 
 * 		
 */

class MarkerDetector {
	
	//Attributes
	private:
		//aruco marker detector used to recognize the markers
		aruco::MarkerDetector mDetector_;
		//vector where all recognized markers are save
		std::vector<aruco::Marker> markers_;
		//aruco camera parameter neeeded to identify marker position
		aruco::CameraParameters camParam_;
		bool useRectifiedImages = true;
		//marker size
		double marker_real_size_ = 0.2;
		double marker_size_ = marker_real_size_;
		// ?
		bool useCamInfo_ ;
		//CV image
		cv::Mat inImage_;
		
		//id of the marker to find
		int actual_marker_id_;
		//message to publish
		experimental_1::markerDistance toSend;
		
		//ROS pubs/subs
		ros::NodeHandle nh_;
		image_transport::ImageTransport it_;
		image_transport::Subscriber image_sub_;
		
		//Subscriber to get the camera info 	
		ros::Subscriber cam_info_sub;	
		
		//publisher of markerDistance
		ros::Publisher markerPoseGoal_pub_;
		//subscriber to markerId to find
		ros::Subscriber requestMarkerId_sub_;
		
		
		
	public:
		MarkerDetector() :
			nh_("~"), it_(nh_), useCamInfo_(true) 
		{
			image_sub_ = it_.subscribe("/camera/color/image_raw",1, &MarkerDetector::image_callback, this);
			cam_info_sub = nh_.subscribe("/camera/color/camera_info", 1, &MarkerDetector::cam_info_callback, this);
						
			nh_.param<bool>("use_camera_info", useCamInfo_, false);
			camParam_ = aruco::CameraParameters();
			
			//our ROS pub-sub-srv
			markerPoseGoal_pub_ = nh_.advertise<experimental_1::markerDistance>("/markerDistance",10); 
			requestMarkerId_sub_ = nh_.subscribe("/requestMarkerId",1, &MarkerDetector::find_marker_callback, this);
			
			//Create window for RosBot POV
			cv::namedWindow("ROSBot POV", cv::WINDOW_AUTOSIZE);	
			
		}
		
		void image_callback(const sensor_msgs::ImageConstPtr& msg) {
			
			ros::Time curr_stamp = msg->header.stamp;
			//create cv_bridge
			cv_bridge::CvImagePtr cv_image_ptr; 
    
			try {
				//get CV Image from sensor image
				cv_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
				inImage_ = cv_image_ptr->image;
				
				//get center of the camera Frame
				int width = static_cast<float>(inImage_.cols);
				int height = static_cast<float>(inImage_.rows);
				cv::Point2f camera_center(width/2.0f, height/2.0f);
   
				// clear out previous detection results
				markers_.clear();

				// ok, let's detect
				mDetector_.detect(inImage_, markers_, camParam_, marker_size_, false);

				// draw detected markers on the image for visualization
				for (std::size_t i = 0; i < markers_.size(); ++i) {
					markers_[i].draw(inImage_, cv::Scalar(0, 0, 255), 2);
					if(markers_[i].id == actual_marker_id_){
						//get center of the marker
						cv::Point2f marker_center = markers_[i].getCenter();
						//draw circle center of marker
						cv::circle(inImage_, marker_center, 1, cv::Scalar(255, 255, 0), 2);
						//draw circle center of camera
						cv::circle(inImage_, camera_center, 3, cv::Scalar(0, 0, 255), 2);
						//compute center distance
						float center_distance = camera_center.x - marker_center.x;
						
						//compute dimension of the side of the marker in pixel
						float marker_distance = markers_[i].getPerimeter()/4.0;
						//set msg field
						toSend.ack = 1; 
						toSend.l_pixel = marker_distance;
						toSend.centerDistance = center_distance;
						//publish
						markerPoseGoal_pub_.publish(toSend);
					}	
				}
				//Visualize the camera POV
				cv::imshow("ROSBot POV", inImage_);
				cv::waitKey(3);
				
			}	
			catch (cv_bridge::Exception& e)
			{
			  ROS_ERROR("cv_bridge exception: %s", e.what());
			}
		}
			
		void find_marker_callback(const std_msgs::Int32 &msg) {
				/*TODO: Change to a service...*/
				actual_marker_id_ = msg.data;
				ROS_INFO("Belin mi è arrivata una richiesta marker %d", msg.data);
				
		}
		  
		 // wait for one camerainfo to get Aruco CameraParam, then shut down that subscriber
		 void cam_info_callback(const sensor_msgs::CameraInfo &msg) {
			camParam_ = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);

			// handle cartesian offset between stereo pairs
			// see the sensor_msgs/CameraInfo documentation for details
			cam_info_sub.shutdown();
			
		  }

				
};

int main(int argc, char **argv){
	
	ros::init(argc, argv, "marker_detector");	
	

	MarkerDetector node;

	ros::spin();
}
