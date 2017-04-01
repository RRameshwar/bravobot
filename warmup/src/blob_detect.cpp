#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/LaserScan.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/thread/mutex.hpp>
#include <iostream>
#include <warmup/LidarCone.h>
#include <keyboard/Key.h>
#include <stdio.h>
#include <geometry_msgs/Point.h>

/* From test_slic.cpp */
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <float.h>
using namespace std;

#include "slic.h"

//static const std::string OPENCV_WINDOW = "Image window";

// ROS -> OpenCV -> RGB-Depth?!
class ImageConverter
{
  ros::NodeHandle nh_;

  // Camera
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  // Lidar
  ros::Subscriber laser_sub_;
  ros::Publisher laser_pub_;

  ros::Subscriber keyboard_sub_;
  ros::Subscriber color_sub_;
  ros::Publisher com_pub_;

  float lastScan_[512];
  boost::mutex lastScan__mutex_;
  boost::mutex reconfig_mutex_;
  unsigned int scanSize_;
  unsigned int rightEdgeScanIndex_;
  unsigned int leftEdgeScanIndex_;

  unsigned int min_hue_;
  unsigned int max_val_;

  // dynamic reconfigure
  ros::Subscriber reconfig_sub_;

  bool verbose_;
  bool is_calibrating_;

  int counter;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/image_raw", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    // Subscribe to laser scan data
    laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &ImageConverter::laserScanCallback, this);
    laser_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_cone", 1000);

    //Publishes position of center of pass
    com_pub_ = nh_.advertise<geometry_msgs::Point>("/center_of_mass", 1);
    
    // Listens to calibration keypress
    keyboard_sub_ = nh_.subscribe<keyboard::Key>("/keyboard/keydown", 1, &ImageConverter::keyBoardCb, this);

    // TODO: color sub uses Lidar Cone message for min hue and max val.... not very clear
    color_sub_ = nh_.subscribe<warmup::LidarCone>("/color_threshold", 1, &ImageConverter::changeColorCb, this);

    reconfig_sub_ = nh_.subscribe<warmup::LidarCone>("dynamic_reconfigure/sensor_cone", 1, &ImageConverter::reconfigCb, this);
    //cv::setMouseCallback(OPENCV_WINDOW, &ImageConverter::processMouseEvent);
    //cv::namedWindow(OPENCV_WINDOW);

    verbose_ = false;
    
    // Starts off as calibration = true, since we haven't received a color message yet
    is_calibrating_ = true;

    /* Calibrated values for bravobot based on dynamic reconfigure test */
    rightEdgeScanIndex_ = 345;
    leftEdgeScanIndex_ = 180;
  }

  ~ImageConverter()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
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

    // If calibration is not running, we already have the thresholding values, so do blob detection!
    // TODO: this is a busy loop when calibrating, so in the future turn the node on and off
    if (!is_calibrating_) {
    
        // 
        // Convert to HSV
        // 
        cv::Mat hsv_image;
        cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

        //blob detection wheeeee
        cv::Mat threshold_image;

        // Thresholding Range determined by calibrated limits 
        inRange(hsv_image, cv::Scalar(0, 0, 0), cv::Scalar(min_hue_, 255, max_val_), threshold_image);

        int scan_width = rightEdgeScanIndex_ - leftEdgeScanIndex_;

        float scale_factor = float(scan_width) / float(threshold_image.cols);
        cv::Mat depth_image(threshold_image.rows*scale_factor, scan_width, CV_8UC1);

        cv::Mat small_color_threshold;
        cv::resize(threshold_image, small_color_threshold, depth_image.size());

        int col_counter = 0;

        for (unsigned int i = scanSize_; i > 0; i--)
        {
          if (ImageConverter::isScanRangeInCone(i))
          {
            cv::Mat column = small_color_threshold.col(col_counter);
            column *= 1/lastScan_[i];
            col_counter++;
          }
        }

        blech(small_color_threshold);

        cv::Mat small_depth_thresh;
        cv::Mat img_eroded;
        cv::Mat img_dilated;

        std::cout << small_color_threshold.size() << std::endl;
        
        inRange(small_color_threshold, cv::Scalar(180, 0, 0), cv::Scalar(255, 255, 255), small_depth_thresh);


        dilate(small_depth_thresh, img_dilated, cv::Mat(), cv::Point(-1, -1), 1);
        erode(img_dilated, img_eroded, cv::Mat(), cv::Point(-1, -1), 1);

        cv::Point com = center_of_mass(img_eroded);

        //Publish center of mass to /center_of_mass
        geometry_msgs::Point com_output;
        com_output.x = com.x;
        com_output.y = com.y;
        com_pub_.publish(com_output);

        
        // for viewing
        com.x += img_eroded.cols/2;
        com.y = -1*(com.y - img_eroded.rows/2);

        cv::circle(img_eroded, com, 10, cv::Scalar(100), -1);
        cv::resize(small_depth_thresh, small_depth_thresh, threshold_image.size());
        cv::resize(img_eroded, img_eroded, threshold_image.size());
        cv::resize(img_dilated, img_dilated, threshold_image.size());
        
        //cv::imshow("threshold_image", threshold_image);
        cv::imshow("small_depth_thresh", small_depth_thresh);
        cv::imshow("img_eroded", img_eroded);
        cv::imshow("color_threshold", threshold_image);
        // cv::imwrite("Screenshot.bmp", graph);
        // cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3);
         
     } 

  }


  int convertScanRangeToCameraDepth(float range)
  {
    return int(128.0 / range);
  }

  bool isScanRangeInCone(unsigned int scanRangeIndex)
  {
    return scanRangeIndex < rightEdgeScanIndex_ && scanRangeIndex > leftEdgeScanIndex_; 
  }

  static void processMouseEvent(int event, int x, int y, int, void* )
  {
    ROS_INFO("TODO: implement some command that hovers and prints the rgb color value");
  }

  void reconfigCb(warmup::LidarCone msg) {
    boost::mutex::scoped_lock reconfig_lock(reconfig_mutex_); 
    rightEdgeScanIndex_ = msg.right_limit;
    leftEdgeScanIndex_ = msg.left_limit;
  }
  
  // TODO: LidarCone msg is being used for min Hue, max Value, which is not very clear
  void changeColorCb(warmup::LidarCone msg) {
    min_hue_ = msg.left_limit; 
    max_val_ = msg.right_limit;
    is_calibrating_ = false;
  }
  
  // Any keyboard press will activate calibration phase
  void keyBoardCb(keyboard::Key msg) {
    std::cout << "Key Pressed" << std::endl;
    is_calibrating_ = true;
  }

  void laserScanCallback(sensor_msgs::LaserScan msg)
  {
    unsigned int size = msg.ranges.size();
    scanSize_= size;

    // TODO(rlouie): set the right/left edges of sensor fusion cone by calibration
    /* rightEdgeScanIndex_ = size*7/12; */
    /* leftEdgeScanIndex_ = size*5/12; */

    // 
    // Smooth the scan
    //
    float lidarDists[size];
    {
      boost::mutex::scoped_lock scoped_lock(lastScan__mutex_);
      for(unsigned int i = 0; i < size; i++){
          double delta = fabs(msg.ranges[i] - lastScan_[i]);
          if(delta < 0.3){
              lidarDists[i] = msg.ranges[i];
          } else{
              lidarDists[i] = std::numeric_limits<double>::infinity();
          }
          lastScan_[i] = msg.ranges[i];
      }
    }

    //
    // Cone of interest
    //
    if (rightEdgeScanIndex_ || leftEdgeScanIndex_)
    {
      boost::mutex::scoped_lock reconfig_lock(reconfig_mutex_);
      for(unsigned int i = 0; i < size; ++i)
      {
        if (ImageConverter::isScanRangeInCone(i))
        {
          msg.ranges[i] = lidarDists[i];
        }
        else
        {
          msg.ranges[i] = std::numeric_limits<double>::infinity();
        }
      }
    }

    // 
    // Publish
    // 
    laser_pub_.publish(msg);
  }

  //Find center of mass of legs by taking the average of the white points in the image.
  cv::Point center_of_mass (cv::Mat input){
    float xsum = 0;
    float ysum = 0;
    float numPoints = 0;
    float width = input.cols;
    float height = input.rows;

    for (int j = 0; j < height; j++){
      for (int i = 0; i < width; i++){
        cv::Scalar color = input.at<uchar>(cv::Point(i, j));
        if (color.val[0] > 150){
          xsum += i;
          ysum += j;
          numPoints ++;
        }
      }
    }

    cv::Point com;
    com.x = (int)((xsum/numPoints) - (width/2));
    com.y = (int)((-1*ysum/numPoints) + (height/2));

    return com;
  }

  void blech (cv::Mat input){
    //normalize the image
    //cv::Scalar mean, stddev;
    //cv::meanStdDev(input, mean, stddev);
    double min, max;
    cv::minMaxLoc(input, &min, &max);
    float width = input.cols;
    float height = input.rows;

    for (int j = 0; j < height; j++){
      for (int i = 0; i < width; i++){
        cv::Scalar color = input.at<uchar>(cv::Point(i, j));
        
        //color.val[0] = (color.val[0] - mean.val[0])*255/stddev.val[0]+127;
        //color.val[0] = lim(color.val[0], 0, 255);
        color.val[0] = 255*color.val[0]/max;

        input.at<uchar>(cv::Point(i, j)) = color.val[0];
        
      }
    }
  }

  float lim (float input, float min, float max){
    if (input<min){
      return min;
    }
    if (input>max){
      return max;
    }
    return input;
  }

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "blob_detect");
  ImageConverter ic;
  ros::spin();
  return 0;
}
