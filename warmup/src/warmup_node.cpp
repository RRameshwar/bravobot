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
#include <stdio.h>

static const std::string OPENCV_WINDOW = "Image window";

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

  float lastScan[512];
  boost::mutex lastScan_mutex_;
  boost::mutex reconfig_mutex_;
  unsigned int scanSize_;
  unsigned int rightEdgeScanIndex_;
  unsigned int leftEdgeScanIndex_;

  // dynamic reconfigure
  ros::Subscriber reconfig_sub_;

  bool verbose_;

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

    reconfig_sub_ = nh_.subscribe<warmup::LidarCone>("dynamic_reconfigure/sensor_cone", 1, &ImageConverter::reconfigCb, this);
    cv::setMouseCallback(OPENCV_WINDOW, &ImageConverter::processMouseEvent);
    cv::namedWindow(OPENCV_WINDOW);

    verbose_ = true;
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
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

    // 
    // Convert to HSV
    // 
    cv::Mat hsv_image;
    cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);
    cv::Mat hsv_split[3];
    cv::split(hsv_image, hsv_split);

    // 
    // Construct depth image
    // 
    // Skip this until the lidar data has caught up 
    boost::mutex::scoped_lock reconfig_lock(reconfig_mutex_);
    if (!leftEdgeScanIndex_ || !rightEdgeScanIndex_)
    {
      return;
    }
    int scan_width = rightEdgeScanIndex_ - leftEdgeScanIndex_;
    if (scan_width < 0)
    {
      return;
    }
    float scale_factor = float(scan_width) / float(hsv_image.cols);
    cv::Mat depth_image(hsv_image.rows*scale_factor, scan_width, CV_8UC1);

    if (verbose_)
    {
      std::cout << "leftEdgeScanIndex_:  " << leftEdgeScanIndex_ << std::endl;
      std::cout << "rightEdgeScanIndex_:  " << rightEdgeScanIndex_ << std::endl;
      std::cout << "scan size:  " << scanSize_ << std::endl;
      std::cout << "scan width:  " << scan_width << std::endl;
      std::cout << "scale_factor:  " << scale_factor << std::endl;
      std::cout << depth_image.cols << std::endl;
      std::cout << depth_image.rows << std::endl;
    }
    {
      boost::mutex::scoped_lock scoped_lock(lastScan_mutex_);
      int col_counter = 0;

      for (unsigned int i = 0; i < scanSize_; i++)
      {
        if (ImageConverter::isScanRangeInCone(i))
        {
          cv::Mat column = depth_image.col(col_counter);
          cv::Scalar val(ImageConverter::convertScanRangeToCameraDepth(lastScan[i]));
          column.setTo(val);
          col_counter++;
        }
      }
    }

    // 
    // Create Small Hue / Value
    // 
    cv::Mat small_hue;
    cv::resize(hsv_split[0], small_hue, depth_image.size());
    cv::Mat small_val;
    cv::resize(hsv_split[2], small_val, depth_image.size());

    // 
    // Merge Depth and RGB
    // 
    cv::Mat final_image;
    std::vector<cv::Mat> channels;
    channels.push_back(small_hue);
    channels.push_back(depth_image);
    channels.push_back(small_val);
    cv::merge(channels, final_image);

    cv::Mat graph = cv::Mat(370, 255, CV_8UC3, cv::Scalar(255,255,255));

    for ( int indexrow = 0; indexrow < small_hue.rows; ++indexrow ) {
      for ( int indexcol = 0; indexcol < small_hue.cols; ++indexcol ) {

        float color = small_hue.at<float>(indexrow,indexcol, 0);
        color = roundf (color);
        int colorindex = static_cast<int>(color);
        // if (colorindex < 0){
        //   colorindex = 0;
        // }
        // else if (colorindex > 360){
        //   colorindex = 360;
        // }


        float depth = depth_image.at<float> (indexrow,indexcol,0);
        depth = roundf (depth);
        int depthindex = static_cast<int>(depth);
        // if (depthindex < 0){
        //   depthindex = 0;
        // }
        // else if (depthindex > 255){
        //   depthindex = 255;
        // }

        // std::cout << depthindex << std::endl;

        for (int adjcolor_index = colorindex+9; adjcolor_index <= colorindex + 15; ++adjcolor_index){
          for (int adjdepth_index = depthindex+9; adjdepth_index <= depthindex + 15; ++adjdepth_index){
            if (adjcolor_index >= 0 && adjcolor_index <= 360 && adjdepth_index >= 0 && adjdepth_index <= 255){
              // std::cout << "adjcolor_index: " << adjcolor_index << std::endl;
              cv::Vec3b pointcolor = graph.at<cv::Vec3b>(adjcolor_index,adjdepth_index);
              pointcolor = (0,0,0);
              graph.at<cv::Vec3b>(adjcolor_index,adjdepth_index) = pointcolor;
            }
            else{
              ;
            }

            // if (tempindex > 0 || tempindex < 360){
            //   cv::Vec3b tempcolor = graph.at<cv::Vec3b>(tempindex,depthindex);
            //   tempcolor = (0,0,0);
            //   graph.at<cv::Vec3b>(tempindex,depthindex) = tempcolor;
            // }
          }
        }

        // std::cout << "Finished!" << colorindex << ", " << depthindex << std::endl;

      }
    }

    // Update GUI Window
    // cv::imshow("hsv", hsv_image);
    cv::imshow("hue", small_hue);
    // cv::imshow("value", hsv_split[2]);
    cv::imshow("depth", depth_image);
    cv::imshow("final_image", final_image);
    cv::imwrite("Screenshot.bmp",graph);
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
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
      boost::mutex::scoped_lock scoped_lock(lastScan_mutex_);
      for(unsigned int i = 0; i < size; i++){
          double delta = fabs(msg.ranges[i] - lastScan[i]);
          if(delta < 0.3){
              lidarDists[i] = msg.ranges[i];
          } else{
              lidarDists[i] = std::numeric_limits<double>::infinity();
          }
          lastScan[i] = msg.ranges[i];
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

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}