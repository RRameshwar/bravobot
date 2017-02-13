#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/LaserScan.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
	ros::NodeHandle nh_;

	// Camera
	image_transport::ImageTransport it_;
	image_transport::Subscriber img_sub_;

public:
	ImageConverter()
	: it_(nh_)
	{
		img_sub_ = it_.subscribe("/image_raw", 1000, &ImageConverter::imgCb, this);
		//<sensor_msgs::ImageConstPtr>

		// cv::setMouseCallback(OPENCV_WINDOW, &ImageConverter::processMouseEvent);
    	cv::namedWindow(OPENCV_WINDOW);
	}
	~ImageConverter(){
		cv::destroyWindow(OPENCV_WINDOW);
	}

	void imgCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // 
    // Convert to HSV
    // 
    cv::Mat hsv_image;
    cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

    mainColor(cv_ptr->image, hsv_image);

    cv::imshow("hsv image", hsv_image);
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
  }

    void mainColor(const cv::Mat src, const cv::Mat hsv_image)
    {
    	// Calibration function that finds color of largest contour in image
    	cv::Mat canny_output; cv::Mat src_gray;
    	//cv::Mat src;
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;

		//Variables for finding contours
		int thresh = 100;
		cv::RNG rng(12345);

		//Variables for finding largest area contour
		int largest_area=0;
		int largest_contour_index=0;
		cv::Rect bounding_rect;

    	cv::cvtColor( src, src_gray, CV_BGR2GRAY );
    	cv::blur( src_gray, src_gray, cv::Size(3,3) );

    	/// Detect edges using canny
		cv::Canny( src_gray, canny_output, thresh, thresh*2, 3 );
		/// Find contours
		cv::findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

		for( int i = 0; i< contours.size(); i++ ) // iterate through each contour. 
		{
	        double area = contourArea( contours[i],false);  //  Find the area of contour
	        if(area > largest_area){
		       largest_area = area;
		       largest_contour_index = i;                //Store the index of largest contour
		       bounding_rect = boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
	    	}
			
			std::cout << "Largest Area is currently: " << largest_area << std::endl;

		}

		cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		drawContours( hsv_image, contours, largest_contour_index, color, 2, 8, hierarchy, 0, cv::Point() );

		/// Draw contours
		// cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );
		// for( int i = 0; i< contours.size(); i++ )
		//    {
		//      cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		//      drawContours( hsv_image, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
		//    }

    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  // TrackPerson tp;
  ros::spin();
  return 0;
}