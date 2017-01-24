#pragma once
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <vector>

class WaypointFinder
{

public:

	void init(int argc, char* argv[]);

	void InputCallback(const std_msgs::Float64MultiArray::ConstPtr &msg);
	void FindWaypoint(const std_msgs::Float64MultiArray::ConstPtr &msg);

	void GPSCallback(const sensor_msgs::NavSatFix &msg);
	void IMUCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg);

	int Dist2WP(float, float, int);
	float FindGPSHeading(float, float, int);
	float FindNewHeading(float, float, float, double, int);


public:	
	ros::NodeHandle nh;
	ros::Subscriber subGPS;
	ros::Subscriber subWPlist;

	ros::Subscriber subIMU;

	ros::Subscriber subInput;
	ros::Subscriber subWPfinder;

	bool hasInput;
	bool waypointFound;
	int pubInterval;
	int currwp;
	int count;
	int counter;
	int i;

	float R;
	
	//current state
	float currentLat;
	float currentLong;
	float wpLat;
	float wpLong;
	float dLat;
	float dLong;
	float a;
	float calc;
	float arcDist;

	float headingx;
	float headingy;
	float headingz;

	float waypointLat;
	float waypointLong;
	float dremainLat;
	float dremainLong;

	float newHeading;
	float GPSHeading;
	float compassHeading;

	double IMUHeading;

	std_msgs::Float64 dCompassHeading;
	std_msgs::Float64 dHeading;

	int arbArray [7];

	float waypointLats [5]; // The current queue of waypoint latitudes
	float waypointLongs [5]; // The current queue of waypoint longitudes

	std_msgs::String pubdirection;
	std::stringstream direction;

	std_msgs::Float64 pubimu;
	std_msgs::Float64 pubheading;

	int maxturnrad;

	ros::Publisher pubvel;
	ros::Publisher pubIMU;
	ros::Publisher pubHeading;
};