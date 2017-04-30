#include <ros/ros.h>
#include <vector>
#include <opencv/cv.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <map>
#include <string>
#include <tf/transform_listener.h>
#include <ros/package.h>

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
        std::string path;

	Localizer()
	{
		path = ros::package::getPath("warmup");
		//pose_sub_ = nh_.subscribe("/amcl_pose", 1, &Localizer::poseCB, this);
		//Change this to use a transform that gives us the position of the person
		map_colored = cv::imread(path+"/maps/robolab_map_color.png", CV_LOAD_IMAGE_COLOR);
    	location_pub_ = nh_.advertise<std_msgs::String>("/area_updates", 1);
    	origin[0] = 2000; origin[1] = 2000; 
    	location = 0;   	
	}

	void localizePerson()
	{

		try{
        listener.lookupTransform("person", "map",  
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
		
                std_msgs::String new_msg;
		//new_msg.volume = 0.1;
		if (current_color == cv::Vec3b(135,135,255) && location!=1)
		{
			location = 1; //Inaccessible
			new_msg.data = "Inaccessible";
			location_pub_.publish(new_msg);
		}
		else if (current_color == cv::Vec3b(255,135,0) && location !=2)
		{
			location = 2; //ASV storage
			new_msg.data = "ASVstorage";
			location_pub_.publish(new_msg);
		}
                else if (current_color == cv::Vec3b(255,255,0) && location !=3)
		{
			location = 3; //SCOPE posters
			new_msg.data = "SCOPEpost";
			location_pub_.publish(new_msg);
		}
                else if (current_color == cv::Vec3b(0,255,255) && location !=4)
		{
			location = 4; //Assistive and Adaptive Studio
			new_msg.data = "A+Astudio";
			location_pub_.publish(new_msg);
		}
                else if (current_color == cv::Vec3b(255,0,255) && location !=5)
		{
			location = 5; //robolab hallway
			new_msg.data = "ROBOhall";
			location_pub_.publish(new_msg);
		}
                else if (current_color == cv::Vec3b(0,0,255) && location !=6)
		{
			location = 6; //Drew's side of RoboLab
			new_msg.data = "ROBOdrew";
			location_pub_.publish(new_msg);
		}
                else if (current_color == cv::Vec3b(255,0,0) && location !=7)
		{
			location = 7; //video screen in robolab hallway
			new_msg.data = "ROBOscreen";
			location_pub_.publish(new_msg);
		}
                else if (current_color == cv::Vec3b(0,255,0) && location !=8)
		{
			location = 8; //Dave's side of RoboLab
			new_msg.data = "ROBOdave";
			location_pub_.publish(new_msg);
		}
                else if (current_color == cv::Vec3b(135,255,0) && location !=9)
		{
			location = 9; //Scope Wall
			new_msg.data = "SCOPEwall";
			location_pub_.publish(new_msg);
		}
                else if (current_color == cv::Vec3b(255,135,255) && location !=10)
		{
			location = 10; //plywood carts
			new_msg.data = "plywood";
			location_pub_.publish(new_msg);
		}
                else if (current_color == cv::Vec3b(0,135,255) && location !=11)
		{
			location = 11; //trash
			new_msg.data = "trash";
			location_pub_.publish(new_msg);
		}
                else if (current_color == cv::Vec3b(0,255,135) && location !=12)
		{
			location = 12; //Investigating Normal wall
			new_msg.data = "InvNorm";
			location_pub_.publish(new_msg);
		}
                else if (current_color == cv::Vec3b(135,0,0) && location !=13)
		{
			location = 13; //ADE project dump
			new_msg.data = "ADE";
			location_pub_.publish(new_msg);
		}

		else if (location==0)
		{
			new_msg.data = "ENTERING THE KNOWHERE";
			location_pub_.publish(new_msg);
		}

	}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  Localizer lz;

  ros::Rate rate(10.0);
  while (lz.nh_.ok()){
  	lz.localizePerson();
  	rate.sleep();
  }


  return 0;
}
