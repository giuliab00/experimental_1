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
#include "nav_msgs/Odometry.h"

/*Transforms*/
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <sstream>

/*  subscriers:
 * 		TODO:update
 * 
 * 	publishers:
 * 		TODO:update
 */

class MarkerDetector {
	
	//Attributes
	private:
		/*------------Aruco------------*/		
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
		/*-----------------------------*/
		
		// Parameters
		bool useCamInfo_ ;
		bool isCameraCompressed;
		
		//CV image decompressed from it_
		cv::Mat inImage_;
		
		//id of the marker to find
		int actual_marker_id_;
		bool POV_window_b_;
		
		//message to publish
		experimental_1::markerDistance toSend;
		sensor_msgs::ImagePtr outputMsg;
		
		//ROS pubs/subs
		ros::NodeHandle nh_;
		image_transport::ImageTransport it_;
		image_transport::Subscriber image_sub_;
		image_transport::Publisher debug_pub_;
		
		//Subscriber to get the camera info 	
		ros::Subscriber cam_info_sub;	
		
		//publisher of markerDistance
		ros::Publisher markerPoseGoal_pub_;
		//subscriber to markerId to find
		ros::Subscriber requestMarkerId_sub_;		
		
	public:
		MarkerDetector() : nh_("~"), it_(nh_), useCamInfo_(true) {

			//create the subscribe to camera image /camera/rgb/image_raw/compressed
			image_sub_ = it_.subscribe("/camera/color/image_raw",1, &MarkerDetector::image_callback, this);

			//create subscriber to camera info in order to set the proper Aruco Camera Parameter
			cam_info_sub = nh_.subscribe("/camera/color/camera_info", 1, &MarkerDetector::cam_info_callback, this);

			//Pub debug
			debug_pub_ = it_.advertise("/debug/image_raw", 1);

			//Read params	
			nh_.param<bool>("use_camera_info", useCamInfo_, false);
			nh_.param<bool>("pov_window", POV_window_b_, true);
			nh_.param<bool>("campressed_camera", isCameraCompressed, false); //To handle the compression
			
			camParam_ = aruco::CameraParameters();
			
			//publisher to notify NavLogic about the distance from the marker
			markerPoseGoal_pub_ = nh_.advertise<experimental_1::markerDistance>("/markerDistance",10);
			//subscriber to know the marker to found
			requestMarkerId_sub_ = nh_.subscribe("/requestMarkerId",1, &MarkerDetector::find_marker_callback, this);
			
			//Debug window for local running
			if(POV_window_b_){
				//Create window for RosBot POV
				cv::namedWindow("ROSBot POV", cv::WINDOW_AUTOSIZE);	
			}	
		}

		void image_callback(const sensor_msgs::ImageConstPtr& msg) {
			/*Callback function of /camera/color/image_raw
   			each time an image is published update the list of marker detected
      		If the marker to found is detected compute it's distance from the center of the camera, to notify
	 		if there's a misalignemet, and how many pixel is the side of the marker to see if the rosbot is still distant 
	 		*/
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

				// detect marker
				mDetector_.detect(inImage_, markers_, camParam_, marker_size_, false);
				
				for (std::size_t i = 0; i < markers_.size(); ++i) {
					// draw detected markers on the image for visualization
					markers_[i].draw(inImage_, cv::Scalar(0, 0, 255), 2);
					//if the marker to found is detected
					if(markers_[i].id == actual_marker_id_){
						//get center of the marker
						cv::Point2f marker_center = markers_[i].getCenter();
						//draw on the image the center of marker for visualization
						cv::circle(inImage_, marker_center, 1, cv::Scalar(255, 255, 0), 2);
						//draw on the image the center of camera for visualization
						cv::circle(inImage_, camera_center, 3, cv::Scalar(0, 0, 255), 2);
						//compute distance of the two center on the x axis
						float center_distance = camera_center.x - marker_center.x;
						
						//compute dimension of the side of the marker in pixel
						float marker_distance = markers_[i].getPerimeter()/4.0;
						
						//set msg field
						toSend.ack = 1; 
						toSend.l_pixel = marker_distance;
						toSend.centerDistance = center_distance;
						toSend.marker_id = markers_[i].id;
						//publish 
						markerPoseGoal_pub_.publish(toSend);
					}	
				}

				//Visualize the camera POV
				if(POV_window_b_){
					cv::imshow("ROSBot POV", inImage_);
					cv::waitKey(1);
				}				
				/*Debug publisher*/
				outputMsg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, inImage_).toImageMsg();
				debug_pub_.publish(outputMsg);				
			}
				
			catch (cv_bridge::Exception& e) {
				ROS_ERROR("cv_bridge exception: %s", e.what());
			}
		}

		void printTransform(const tf::Transform& transform) {
			tf::Vector3 translation = transform.getOrigin();
			tf::Quaternion rotation = transform.getRotation();

			ROS_INFO("Translation (x, y, z): (%.2f, %.2f, %.2f)", translation.x(), translation.y(), translation.z());
			ROS_INFO("Rotation (x, y, z, w): (%.2f, %.2f, %.2f, %.2f)", rotation.x(), rotation.y(), rotation.z(), rotation.w());
		}
			
		void find_marker_callback(const std_msgs::Int32 &msg) {
			/*Update the marker to found once a new one has been published*/
			actual_marker_id_ = msg.data;
			ROS_INFO("Belin mi � arrivata una richiesta marker %d", msg.data);	
		}
		  
		// wait for one camerainfo to get Aruco CameraParam, then shut down that subscriber
		void cam_info_callback(const sensor_msgs::CameraInfo &msg) {
			/*Get one camera info to properly set the ArucoCameraParameter*/ 
			camParam_ = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);
			//kill the subscriber
			cam_info_sub.shutdown();
		}
				
};

int main(int argc, char **argv){
	// init ros Node
	ros::init(argc, argv, "marker_detector");	

	//create markerDetecor
	MarkerDetector node;
	
	//spin Node
	ros::spin();
}
