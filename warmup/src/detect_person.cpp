#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/LaserScan.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/thread/mutex.hpp>
#include <iostream>
#include <warmup/LidarCone.h>
#include <stdio.h>

/* From test_slic.cpp */
#include <stdio.h>
#include <math.h>
#include <vector>
#include <float.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
using namespace std;


class PersonDetect
{
  ros::NodeHandle nh_;

  // start/stop
  ros::Subscriber start_sub_;
  ros::Subscriber stop_sub_;
  bool pub_;

  //Receive an image, publish a twist
  ros::Subscriber laser_sub_;
  ros::Subscriber pos_sub_;
  ros::Publisher twist_pub_;

  float dist_nearest_object;
  geometry_msgs::Point pos_person; 


public:

	PersonDetect () {
		twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
		dist_nearest_object = 0;
		start_sub_ = nh_.subscribe<std_msgs::Bool>("start", 1, &PersonDetect::startCb, this);
		pub_ = false;
	}

	void init(){
		laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &PersonDetect::laserScanCallback, this);
		pos_sub_ = nh_.subscribe<geometry_msgs::Point>("center_of_mass", 1000, &PersonDetect::COMCallback, this);
		stop_sub_ = nh_.subscribe<std_msgs::Bool>("stop", 1, &PersonDetect::stopCb, this);
		pub_ = true;
	}

	void sleep(){
		laser_sub_.shutdown();
		pos_sub_.shutdown();
		stop_sub_.shutdown();
		pub_ = false;
	}

	void startCb(const std_msgs::Bool msg){
		if (msg.data) {
			init();
		}
	}

	void stopCb(const std_msgs::Bool msg){
		if (msg.data) {
			sleep();
		}
	}

	void laserScanCallback (sensor_msgs::LaserScan msg){
		float min = 100;
		float start_angle = -1*M_PI/4;
		float end_angle = M_PI/4;
		int start_index = (int)((start_angle-msg.angle_min)/msg.angle_increment);
		int end_index = (int)((end_angle-msg.angle_min)/msg.angle_increment);

		for (int i = start_index; i <= end_index; i++){
			if (msg.ranges[i] < min){
				min = msg.ranges[i];
			} 
		}

		if (min < 100){
			dist_nearest_object = min;
		}

	}

	void COMCallback (geometry_msgs::Point msg){
		pos_person = msg;
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

	void run (){
		if (pub_) {
			float speed = lim((dist_nearest_object - 1) / 3, -1, 1);
			float angle = lim(-1 * ((float) pos_person.x) / 100, -1, 1);
			std::cout << pos_person.x << std::endl;
			geometry_msgs::Twist output;
			output.linear.x = speed;
			output.angular.z = angle;
			twist_pub_.publish(output);
		}
	}
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "person_detector");

  PersonDetect person_detector;
  ros::Rate r(10);
  while (ros::ok()){
  	person_detector.run();
  	ros::spinOnce();
  	r.sleep();
  }
  return 0;
}
