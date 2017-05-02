#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/LaserScan.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/thread/mutex.hpp>
#include <iostream>
#include "warmup/ColorThreshold.h"
#include "warmup/LidarCone.h"
//#include <keyboard/Key.h>
//#include <warmup/ColorThreshold.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include <stdio.h>
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
#include <geometry_msgs/Vector3.h>

using namespace std;

#include "slic.h"

//static const std::string OPENCV_WINDOW = "Image window";

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
class Calibrator
{
  ros::NodeHandle nh_;

  // Camera
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

  // Lidar
  ros::Subscriber laser_sub_;

  float lastScan[512];
  boost::mutex lastScan_mutex_;
  boost::mutex reconfig_mutex_;
  unsigned int scanSize_;
  unsigned int rightEdgeScanIndex_;
  unsigned int leftEdgeScanIndex_;

  // dynamic reconfigure
  ros::Subscriber reconfig_sub_;

  // Keyboard Listener/Timers for Person Calibration
  ros::Subscriber start_sub_;
  ros::Publisher done_pub_;

  ros::Publisher color_pub_;
  struct timeval time_start_;

  bool verbose_;
  bool do_slic_;

  vector<CvScalar> template_color_vec;

public:
  Calibrator()
    : it_(nh_)
  {
    // TODO: LidarCone msg being used for min hue and max val... not so clear
    color_pub_ = nh_.advertise<warmup::ColorThreshold>("/color_threshold", 1);
    start_sub_ = nh_.subscribe<std_msgs::Bool>("start", 1, &Calibrator::Start_cb, this);
    done_pub_ = nh_.advertise<std_msgs::Bool>("stop", 10);
    verbose_ = false;
    do_slic_ = false;

      /* Calibrated values for bravobot based on dynamic reconfigure test */
    rightEdgeScanIndex_ = 358;
    leftEdgeScanIndex_ = 187;
  }

  void bringup(){
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/image_raw", 1, &Calibrator::imageCb, this);
    // Subscribe to laser scan data
    laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &Calibrator::laserScanCallback, this);

    reconfig_sub_ = nh_.subscribe<warmup::LidarCone>("dynamic_reconfigure/sensor_cone", 1, &Calibrator::reconfigCb, this);
    //cv::setMouseCallback(OPENCV_WINDOW, &Calibrator::processMouseEvent);
    //cv::namedWindow(OPENCV_WINDOW);
    do_slic_ = true;
  }

  void sleep(bool condition){
      std::cout << "calibration complete" << std::endl;
      std_msgs::Bool msg;
      msg.data = condition;
      done_pub_.publish(msg);
      image_sub_.shutdown();
      laser_sub_.shutdown();
      reconfig_sub_.shutdown();
      //cv::destroyWindow(OPENCV_WINDOW);
  }

  ~Calibrator()
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
      if (Calibrator::isScanRangeInCone(i))
      {
        cv::Mat column = depth_image.col(col_counter);
        cv::Scalar val(Calibrator::convertScanRangeToCameraDepth(lastScan[i]));

        column.setTo(val);
        col_counter++;
      }
    }

    // scale the values of depth image between 0 - 255
    cv::normalize(depth_image, depth_image, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    // resize depth_image to full size
    cv::resize(depth_image, depth_image, cv_ptr->image.size());
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

//    // Transform it into the C++ cv::Mat format
//      cv::Mat image1(depth_image);

    // SLIC
    //
    if (do_slic_)
    {
        //Cropping about 1/3 of lab_image
        cv::Mat lab_image2 = cropMiddleThirds(small_lab);
        IplImage *lab_image = new IplImage(lab_image2);

        /* Yield the number of superpixels and weight-factors from the user. */
        int w = lab_image->width, h = lab_image->height;
        int nr_superpixels = 250;
        int nc = 20;

        double step = sqrt((w * h) / (double)nr_superpixels);

        /* Perform the SLIC superpixel algorithm. */
        Slic slic;
        slic.generate_superpixels(lab_image, step, nc);
        slic.create_connectivity(lab_image);

        /* Do second level of clustering on the superpixels */
        cv::Mat final_image_copy1 = small_hsv.clone(); //Used to be BGR FFFFFFFF

        //Cropping about 1/3 of final_image_copy1
        cv::Mat final_image_copy = cropMiddleThirds(final_image_copy1);

        //Cropping about 1/3 of depth_image
        cv::Mat depth_image_middleCropped = cropMiddleThirds(depth_image);

        IplImage *final_image_ipl = new IplImage(final_image_copy);
        IplImage *depth_image_ipl = new IplImage(depth_image_middleCropped);

        // display superpixel contours
        CvScalar cvBlack = {{230,159,0}};
        slic.display_contours(final_image_ipl, cvBlack);

        /* slic.two_level_cluster (final_image_ipl, 0, 0.9, 3, 0.3); */
        /* CvScalar template_color = slic.calibrate_template_color(final_image_ipl, depth_image_ipl); */
        /* template_color_vec.push_back(template_color); */

        // Add the colours defining the legs in this frame to the overall color calibration vector
        vector<CvScalar> colours_this_frame = slic.get_leg_color(final_image_ipl, depth_image_ipl);
        cout << colours_this_frame.size() << endl;
        //cout << "error above here" << endl;
        template_color_vec.reserve(template_color_vec.size() + distance(colours_this_frame.begin(),colours_this_frame.end()));
        template_color_vec.insert(template_color_vec.end(),colours_this_frame.begin(),colours_this_frame.end());

        cv::Mat final_slic_image = cv::Mat(final_image_ipl);
        //cv::Mat bigger_final_slic_image;
        //cv::resize(final_slic_image, bigger_final_slic_image, cv_ptr->image.size());

        //cv::imshow("result", final_slic_image);


        // Once calibration period has completed
        double time_elapsed = get_time_elapsed(time_start_);

        if (time_elapsed >= 10 * 1000) {
            // Stop doing SLIC calibration
            do_slic_ = false;
            // Extract color ranges for the legs we were calibrating onto
            // Perform Min/Max in the 3 dimensions in color space
            vector<cv::Scalar> minmaxColours = Calibrator::minmaxColourCalibration();
            cout << minmaxColours.size() << endl;

            // Widening the range of HSV to maximize white pixels at initialization
            minmaxColours[0].val[0] = max(0.0, minmaxColours[0].val[0] - 40);
            minmaxColours[0].val[1] = max(0.0, minmaxColours[0].val[1] - 40);
            minmaxColours[0].val[2] = max(0.0, minmaxColours[0].val[2] - 40);
            minmaxColours[1].val[0] = min(255.0, minmaxColours[1].val[0] + 40);
            minmaxColours[1].val[1] = min(255.0, minmaxColours[1].val[1] + 40);
            minmaxColours[1].val[2] = min(255.0, minmaxColours[1].val[2] + 40);

            // turn hsv to black and white
            cv::Mat threshold_image;
            cv::inRange(small_hsv, minmaxColours[0], minmaxColours[1], threshold_image);
            cv::Mat threshold_mid_crop = cropMiddleTop(threshold_image);
            cv::Mat threshold_left_crop = cropLeftThird(threshold_image);
            cv::Mat threshold_right_crop = cropRightThird(threshold_image);

            //cv::imshow("threshold", threshold_image);
            int left_whitepixel = cv::countNonZero(threshold_left_crop);
            int mid_whitepixel = cv::countNonZero(threshold_mid_crop);
            int right_whitepixel = cv::countNonZero(threshold_right_crop);
            int outer_whitepixel = left_whitepixel + right_whitepixel;

            vector<int> outer_and_inner_whitepixels;
            outer_and_inner_whitepixels.push_back(outer_whitepixel);
            outer_and_inner_whitepixels.push_back(mid_whitepixel);

            //cout << "BEFORE: 30th and 70th" << endl;
            //cout << "(" << minmaxColours[0].val[0] << ",";
            //cout << minmaxColours[0].val[1] << ",";
            //cout << minmaxColours[0].val[2] << ")" << endl;
            //cout << "(" << minmaxColours[1].val[0] << ",";
            //cout << minmaxColours[1].val[1] << ",";
            //cout << minmaxColours[1].val[2] << ")" << endl;

            for (int i=0; i<6; i++)
            {
                int param1 = i >= 3; // first 3 times in the loop we are doing lower, last 3 times we are doing upper
                int param2 = i % 3; // cycle between hue, sat, val

                bool keep_tuning_hsv = true;
                int increment = 20;
                while (keep_tuning_hsv)
                {
                    if (param1) // if doing upper
                    {
                        increment *= -1;
                    }
                    else // if doing lower
                    {
                        increment *= 1;
                    }
                    minmaxColours[param1].val[param2] += increment;

                    if(minmaxUpdate(minmaxColours, outer_and_inner_whitepixels, small_hsv))
                    {
                        //do nothing, because we found a better parameter and we should keep it
                    }
                    else
                    {
                        minmaxColours[param1].val[param2] -= increment;
                        keep_tuning_hsv = false;
                        cout << "moving on to next parameter" << endl;
                    }

                    //cout << "AFTER: 30th and 70th" << endl;
                    //cout << "(" << minmaxColours[0].val[0] << ",";
                    //cout << minmaxColours[0].val[1] << ",";
                    //cout << minmaxColours[0].val[2] << ")" << endl;
                    //cout << "(" << minmaxColours[1].val[0] << ",";
                    //cout << minmaxColours[1].val[1] << ",";
                    //cout << minmaxColours[1].val[2] << ")" << endl;
                }
            }

            cv::inRange(small_hsv, minmaxColours[0], minmaxColours[1], threshold_image);
            threshold_mid_crop = cropMiddleTop(threshold_image);
            threshold_left_crop = cropLeftThird(threshold_image);
            threshold_right_crop = cropRightThird(threshold_image);
            left_whitepixel = cv::countNonZero(threshold_left_crop);
            mid_whitepixel = cv::countNonZero(threshold_mid_crop);
            right_whitepixel = cv::countNonZero(threshold_right_crop);
            outer_whitepixel = left_whitepixel + right_whitepixel;

//            cout << "LeftPixel" << left_whitepixel << "  ,RightPixel";
//            cout << right_whitepixel << "  ,MidPixel";
//            cout << mid_whitepixel << ")" << endl;

            if((outer_whitepixel/2.0) < 0.9*mid_whitepixel)
            {
                cout << endl << "Person Follower Calibrated!" << endl;
                sleep(true); // return person calibrated
            }
            else if(((outer_whitepixel/2.0) > 0.7*mid_whitepixel) && ((outer_whitepixel/2.0) < 1.3*mid_whitepixel))
            {
                cout << endl << "Calibration Unsuccessful! Please Calibrate again..." << endl;
                sleep(false); // return person not found
            }
            else
            {
                cout << endl << "ERROR!!" << endl;
                sleep(false); // return error (*need to implement try catch*)
            }

            //cv::inRange(small_hsv, minmaxColours[0], minmaxColours[1], threshold_image);
            //cv::imshow(OPENCV_WINDOW, threshold_image);
            //cv::imshow("calibrated", threshold_image);
            cout << endl << "AFTER: 30th and 70th" << endl;
            cout << "(" << minmaxColours[0].val[0] << ",";
            cout << minmaxColours[0].val[1] << ",";
            cout << minmaxColours[0].val[2] << ")" << endl;
            cout << "(" << minmaxColours[1].val[0] << ",";
            cout << minmaxColours[1].val[1] << ",";
            cout << minmaxColours[1].val[2] << ")" << endl << endl;
            warmup::ColorThreshold msg;
            msg.min.x = minmaxColours[0].val[0];
            msg.min.y = minmaxColours[0].val[1];
            msg.min.z = minmaxColours[0].val[2];
            msg.max.x = minmaxColours[1].val[0];
            msg.max.y = minmaxColours[1].val[1];
            msg.max.z = minmaxColours[1].val[2];
            color_pub_.publish(msg);
            //sleep(true);
        }
    }
    cv::waitKey(3);
  }

  cv::Mat cropMiddleThirds(cv::Mat imageToCrop)
  {
      int starting_third = 0.3*(imageToCrop.cols-1);
      cv::Mat image0(imageToCrop);
      cv::Rect myROI0(starting_third, 0, (imageToCrop.cols-1)-(2*starting_third), (imageToCrop.rows - 1));
      cv::Mat croppedImage = image0(myROI0);
      return croppedImage;
  }

   cv::Mat cropMiddleTop(cv::Mat imageToCrop)
   {
      int starting_third = 0.3*(imageToCrop.cols-1);
      int top_half = 0.5*(imageToCrop.rows-1);
      cv::Mat image0(imageToCrop);
      cv::Rect myROI0(starting_third, 0, (imageToCrop.cols-1)-(2*starting_third), top_half);
      cv::Mat croppedImage = image0(myROI0);
      return croppedImage;
   }

  cv::Mat cropLeftThird(cv::Mat imageToCrop)
  {
      int starting_third = 0.3*(imageToCrop.cols-1);
      cv::Mat image0(imageToCrop);
      cv::Rect myROI0(0, 0, starting_third, (imageToCrop.rows - 1));
      cv::Mat croppedImage = image0(myROI0);
      return croppedImage;
  }

  cv::Mat cropRightThird(cv::Mat imageToCrop)
  {
      int starting_third = 0.7*(imageToCrop.cols-1);
      cv::Mat image0(imageToCrop);
      cv::Rect myROI0(starting_third, 0, (imageToCrop.cols-1)-(starting_third), (imageToCrop.rows - 1));
      cv::Mat croppedImage = image0(myROI0);
      return croppedImage;
  }

  bool minmaxUpdate(vector<cv::Scalar> &minmaxColours, vector<int> &outer_and_inner_whitepixels,
                    cv::Mat small_hsv)
  {
      int left_whitepixel_new;
      int mid_whitepixel_new;
      int right_whitepixel_new;
      int outer_whitepixel_new;
      cv::Mat threshold_image;
      cv::inRange(small_hsv, minmaxColours[0], minmaxColours[1], threshold_image);
      cv::Mat threshold_mid_crop = cropMiddleTop(threshold_image);
      cv::Mat threshold_left_crop = cropLeftThird(threshold_image);
      cv::Mat threshold_right_crop = cropRightThird(threshold_image);

      left_whitepixel_new = cv::countNonZero(threshold_left_crop);
      mid_whitepixel_new = cv::countNonZero(threshold_mid_crop);
      right_whitepixel_new = cv::countNonZero(threshold_right_crop);
      outer_whitepixel_new = left_whitepixel_new + right_whitepixel_new;

      //cout << "outer: " << outer_whitepixel_new << " inner: " << mid_whitepixel_new << endl;
      //cout << "outer is darker: " << (outer_whitepixel_new < 0.9*outer_and_inner_whitepixels[0]) << endl;
      //cout << "middle is more or equally white: " << !(mid_whitepixel_new < 0.9*outer_and_inner_whitepixels[1]) << endl;
      //cv::imshow("threshold_candidate", threshold_image);
      //cv::waitKey(0); // show image until button press
      if((outer_whitepixel_new < 0.9*outer_and_inner_whitepixels[0]) && // if new outer is darker
            !(mid_whitepixel_new < 0.9*outer_and_inner_whitepixels[1])) // if new middle is more or equally white
      {
          // keep
          //cout << "found a better value!" << endl;
          outer_and_inner_whitepixels[0] = outer_whitepixel_new;
          outer_and_inner_whitepixels[1] = mid_whitepixel_new;
          return true;
      }
      else
      {
          return false;
      }
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

  // Start Calibration upon start message
  void Start_cb(std_msgs::Bool msg) {
    if (msg.data)
    {
      std::cout << "calibration started" << std::endl;
      time_start_ = get_time_now();
      bringup();
    }
  }

   vector<cv::Scalar> minmaxColourCalibration() {
      // Construct 3 integer arrays for the 3 color channels
      vector<int> channel1, channel2, channel3;
      for (vector<CvScalar>::iterator it = template_color_vec.begin(); it != template_color_vec.end(); ++it) {
         channel1.push_back(it->val[0]);
         channel2.push_back(it->val[1]);
         channel3.push_back(it->val[2]);
      }

      std::sort(channel1.begin(), channel1.end());
      std::sort(channel2.begin(), channel2.end());
      std::sort(channel3.begin(), channel3.end());
      // 30th and 70th percentile
      int len = channel1.size();
      int c1offset = static_cast<int>(len * 0.3);
      int c2offset = static_cast<int>(len * 0.3);
      int c3offset = static_cast<int>(len * 0.3);
      int minc1 = *(channel1.begin()+c1offset)-2;
      int maxc1 = *(channel1.end()-1-c1offset)+2;
      int minc2 = *(channel2.begin()+c2offset)-2;
      int maxc2 = *(channel2.end()-1-c2offset)+2;
      int minc3 = *(channel3.begin()+c3offset)-2;
      int maxc3 = *(channel3.end()-1-c3offset)+2;

      cout << "30th and 70th" << endl;
      cout << "(" << minc1 << ",";
      cout << minc2 << ",";
      cout << minc3 << ")" << endl;
      cout << "(" << maxc1 << ",";
      cout << maxc2 << ",";
      cout << maxc3 << ")" << endl;

//    cv::Scalar min_colour(0, 0, 0);
//    cv::Scalar max_colour(40, 255, 100);

      cv::Scalar min_colour(minc1, minc2, minc3);
      cv::Scalar max_colour(maxc1, maxc2, maxc3);

      vector<cv::Scalar> minmaxColours;
      minmaxColours.push_back(min_colour);
      minmaxColours.push_back(max_colour);
      return minmaxColours;



      // Publish min hue and max val....
      // TODO: don't use lidar cone not very clear 
//      warmup::LidarCone msg;
//      msg.left_limit = minc1;
//      msg.right_limit = maxc3;
//      color_pub_.publish(msg);

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
        if (Calibrator::isScanRangeInCone(i))
        {
          msg.ranges[i] = lidarDists[i];
        }
        else
        {
          msg.ranges[i] = std::numeric_limits<double>::infinity();
        }
      }
    }

  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "calibrator");
  Calibrator calibrator;
  ros::spin();
  return 0;
}
