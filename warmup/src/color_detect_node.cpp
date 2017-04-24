#include <ros/ros.h>
#include <vector>
#include <opencv/cv.h>
#include <std_msgs/Float64.h>
#include <sound_play/SoundRequest.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <map>
#include <string>
#include <tf/transform_listener.h>

//using namespace cv;

class Localizer
{

public:

	ros::NodeHandle nh_;
	ros::Subscriber pose_sub_;
  	ros::Publisher location_pub_;
  	cv::Mat map_colored;
  	int origin[];
  	int location;
  	tf::TransformListener listener;
  	tf::StampedTransform transform;

	Localizer()
	{
		
		pose_sub_ = nh_.subscribe("/amcl_pose", 1, &Localizer::poseCB, this);
		//Change this to use a transform that gives us the position of the person
		map_colored = cv::imread("/home/odroid/colormap.png", CV_LOAD_IMAGE_COLOR);
    	location_pub_ = nh_.advertise<sound_play::SoundRequest>("/robotsound", 1);
    	origin[0] = 2000; origin[1] = 2000; 
    	location = 0;   	
	}

	void localizePerson(const geometry_msgs::PoseWithCovarianceStamped msg)
	{

		try{
        listener.lookupTransform("/person", "/map",  
                                  ros::Time(0), transform);
    	}
       	catch (tf::TransformException ex){
        	ROS_ERROR("%s",ex.what());
        	ros::Duration(1.0).sleep();
        	return;
    	}

		double current_x = transform.getOrigin().x();
		double current_y = transform.getOrigin().y();
		
		std::cout << "Position[x, y] = " << current_x << " " << current_y << std::endl;

		double offset_x = current_x/0.05;
		double offset_y = current_y/0.05;


		double pixels_x = offset_x + 2000;
		double pixels_y = origin[1] - offset_y;

		std::cout << "Pixels[x, y] = " << pixels_x << " " << pixels_y << std::endl;
		
		//imshow( "Display window", map_colored );  
		cv::Vec3b current_color = map_colored.at<cv::Vec3b>(cv::Point((int)pixels_x, (int)pixels_y));

		std::cout << (int)current_color.val[0] << " " << (int)current_color.val[1] << " " << (int)current_color.val[2] << std::endl;
		
		sound_play::SoundRequest new_msg;
		new_msg.sound = -3;
		new_msg.command = 1;
		//new_msg.volume = 0.1;
		if (current_color == cv::Vec3b(0,128,0) && location!=1)
		{
			location = 1; //Hallway
			new_msg.arg = "ENTERING THE HALLWAY";
			location_pub_.publish(new_msg);
		}
		else if (current_color == cv::Vec3b(0,0,255) && location !=2)
		{
			location = 2; //Robosys Area
			new_msg.arg = "ENTERING THE ROBOSYS AREA";
			location_pub_.publish(new_msg);
		}
		else if (location==0)
		{
			new_msg.arg = "ENTERING THE KNOWHERE";
			location_pub_.publish(new_msg);
		}

	}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  Localizer lz;

  ros::Rate rate(10.0)
  while (lz.nh_.ok()){
  	localizePerson();
  	rate.sleep();
  }


  return 0;
}