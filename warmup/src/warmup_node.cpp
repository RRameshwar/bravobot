#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/LaserScan.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/thread/mutex.hpp>

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
  int scanSize_;
  int rightEdgeScanIndex_;
  int leftEdgeScanIndex_;


public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    // Subscribe to laser scan data
    laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &ImageConverter::laserScanCallback, this);
    laser_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_cone", 1000);

    cv::setMouseCallback(OPENCV_WINDOW, &ImageConverter::processMouseEvent);
    cv::namedWindow(OPENCV_WINDOW);
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

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    cv::Mat hsv_image;
    cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

    cv::Mat hsv_split[3];
    cv::split(hsv_image, hsv_split);

    // 
    // TODO(rlouie):
    // Iterate through lastScan[leftEdgeScanIndex_] -> lastScan[rightEdgeScanIndex_]
    // Define the depth of the image by each scan value
    // You can define a new cv::Mat with depth 2, where we have hue and depth
    // A visualization of just the depth channel would be a good debugging step.
    // 

    // Update GUI Window
    cv::imshow("hsv", hsv_image);
    cv::imshow("hue", hsv_split[0]);
    cv::imshow("value", hsv_split[2]);
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }

  static void processMouseEvent(int event, int x, int y, int, void* )
  {
    ROS_INFO("TODO: implement some command that hovers and prints the rgb color value");
  }

  void laserScanCallback(sensor_msgs::LaserScan msg)
  {
    int size = msg.ranges.size();
    if (!scanSize_) scanSize_= size;

    // TODO(rlouie): set the right/left edges of sensor fusion cone by calibration
    if (!rightEdgeScanIndex_) rightEdgeScanIndex_ = size*1/6;
    if (!leftEdgeScanIndex_) rightEdgeScanIndex_ = size*5/6;

    // 
    // Smooth the scan
    //
    float lidarDists[size];
    {
      boost::mutex::scoped_lock scoped_lock(lastScan_mutex_);
      for(int i = 0; i < size; i++){
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
    for(unsigned int i = 0; i < size; ++i)
    {
      if (i < rightEdgeScanIndex_ || i > leftEdgeScanIndex_)
      {
        msg.ranges[i] = lidarDists[i];
      }
      else
      {
        msg.ranges[i] = std::numeric_limits<double>::infinity();
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