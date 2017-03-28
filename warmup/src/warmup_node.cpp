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
#include <keyboard/Key.h>
#include <time.h>
#include <sys/time.h>

/* From test_slic.cpp */
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include <float.h>
using namespace std;

#include "slic.h"

static const std::string OPENCV_WINDOW = "Image window";

/* timing helpers */
struct timeval get_time_now() {
    struct timeval now;
    gettimeofday(&now, NULL);
    return now;
}

double timevaldiff(struct timeval *starttime, struct timeval *finishtime)
{
    double msec;
    msec=(finishtime->tv_sec-starttime->tv_sec)*1000;
    msec+=(finishtime->tv_usec-starttime->tv_usec)/1000;
    return msec;
}

double get_time_elapsed(struct timeval t0) {
    struct timeval now = get_time_now();
    double time_elapsed = timevaldiff(&t0, &now);
    return time_elapsed;
}

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

  // Keyboard Listener/Timers for Person Calibration
  ros::Subscriber keyboard_sub_;
  struct timeval time_start_;

  bool verbose_;
  bool do_graph_;
  bool do_slic_;

  int counter;

  vector<CvScalar> template_color_vec;

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

    keyboard_sub_ = nh_.subscribe<keyboard::Key>("/keyboard/keydown", 1, &ImageConverter::keyBoardCb, this);
    verbose_ = false;
    do_graph_ = false;
    do_slic_ = false;

    /* Calibrated values for bravobot based on dynamic reconfigure test */
    rightEdgeScanIndex_ = 345;
    leftEdgeScanIndex_ = 180;
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

    if (hsv_image.cols == 0){
      // Catches division by zero
        hsv_image.cols = 640;
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

    boost::mutex::scoped_lock scoped_lock(lastScan_mutex_);
    int col_counter = 0;

    for (unsigned int i = scanSize_; i > 0; i--)
    {
      if (ImageConverter::isScanRangeInCone(i))
      {
        cv::Mat column = depth_image.col(col_counter);
        cv::Scalar val(ImageConverter::convertScanRangeToCameraDepth(lastScan[i]));

        column.setTo(val);
        col_counter++;
      }
    }

    // scale the values of depth image between 0 - 255
    cv::normalize(depth_image, depth_image, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    // use otsu thresholding to only get the depths of things close to bot (presumably legs)
    cv::threshold(depth_image, depth_image, 0, 255, cv::THRESH_BINARY+cv::THRESH_OTSU);

    //
    // Create Small Images
    //
    cv::Mat small_hue;
    cv::resize(hsv_split[0], small_hue, depth_image.size());
    cv::Mat small_sat;
    cv::resize(hsv_split[1], small_sat, depth_image.size());
    cv::Mat small_val;
    cv::resize(hsv_split[2], small_val, depth_image.size());

    cv::Mat small_bgr;
    cv::resize(cv_ptr->image, small_bgr, depth_image.size());
    cv::Mat small_lab;
    cv::cvtColor(small_bgr, small_lab, cv::COLOR_BGR2Lab);

    //
    // Merge Depth and RGB
    //
    cv::Mat final_image;
    cv::Mat small_hsv;
    std::vector<cv::Mat> channels;
    channels.push_back(small_hue);
    channels.push_back(small_sat);
    channels.push_back(small_val);
    cv::merge(channels, small_hsv);
    channels.push_back(depth_image);
    cv::merge(channels, final_image);

    // SLIC
    //
    if (do_slic_)
    {
        IplImage *lab_image = new IplImage(small_lab);
        /* Yield the number of superpixels and weight-factors from the user. */
        int w = lab_image->width, h = lab_image->height;
        int nr_superpixels = 10;
        int nc = 10;

        double step = sqrt((w * h) / (double)nr_superpixels);

        /* Perform the SLIC superpixel algorithm. */
        Slic slic;
        slic.generate_superpixels(lab_image, step, nc);
        slic.create_connectivity(lab_image);

        /* Do second level of clustering on the superpixels */
        cv::Mat final_image_copy = small_hsv.clone(); //Used to be BGR FFFFFFFF
        IplImage *final_image_ipl = new IplImage(final_image_copy);
        IplImage *depth_image_ipl = new IplImage(depth_image);

        // display superpixel contours
        /* CvScalar cvBlack = {{0, 0, 0}}; */
        /* slic.display_contours(final_image_ipl, cvBlack); */

        slic.two_level_cluster (final_image_ipl, 0, 0.8, 3, 0.3);
        CvScalar template_color = slic.calibrate_template_color(final_image_ipl, depth_image_ipl);
        template_color_vec.push_back(template_color);
        cv::Mat final_slic_image = cv::Mat(final_image_ipl);
        cv::Mat bigger_final_slic_image;
        cv::resize(final_slic_image, bigger_final_slic_image, cv_ptr->image.size());

        cv::imshow("result", bigger_final_slic_image);

        double time_elapsed = get_time_elapsed(time_start_);
        if (time_elapsed >= 5 * 1000) {
            do_slic_ = false;
        }
    }

    if (do_graph_) {

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
    cv::imwrite("Screenshot.bmp",graph);
    }

    // Update GUI Window
    // cv::imshow("hsv", hsv_image);
//    cv::imshow("hue", small_hue);
    // cv::imshow("value", hsv_split[2]);
   cv::imshow("depth", depth_image);
    // cv::imshow("final_image", final_image);
    // cv::imwrite("Screenshot.bmp", graph);
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }


  int convertScanRangeToCameraDepth(float range)
  {
    // Ignore ranges outside of 3 meters
    if (range == 0){
      range = std::numeric_limits<float>::infinity();
    }
    range = std::min(range, static_cast<float>(3));

    // function of depth_image_val vs range linear
    return static_cast<int>(255 * (3-range) / float(3));
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

  void keyBoardCb(keyboard::Key msg) {
    std::cout << "Key Pressed" << std::endl;
    time_start_ = get_time_now();
    do_slic_ = true;
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
          }
          else if(std::isnan(msg.ranges[i])){
              msg.ranges[i] = std::numeric_limits<double>::infinity();
          }
          else{
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
