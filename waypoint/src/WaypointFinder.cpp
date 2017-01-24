// Olin College Fundamentals of Robotics 2016
// Last edited by Lydia Z. 12/7/16
// Need to account for waypoints within 3 m of each other.

#include "WaypointFinder.h"
#include <cmath>

/*######################################## HEADING AND DISTANCE CALCULATIONS ##############################################*/

int WaypointFinder::Dist2WP(float passedLat, float passedLong, int passedwp){
	// for ( i=0; i <= 4; i++ ) {
	// 	// For waypoint i in list waypoints, check if waypoint exists (default value is (0,0))
	// 	// Once a valid waypoint is found, set currwp (currentwaypoint) to waypoint i.
	// 	if ((waypointLats[i] == 0.00000) && (waypointLongs[i] == 0.00000)){
	// 		//Nothing happens
	// 	}
	// 	else{ //If waypoint i in waypoints exists, make it our current waypoint.
	// 		// std::cout << "Waypoint # set to: " << i << std::endl;
	// 		currwp = i;
	// 		break;
	// 	}
	// }
	currwp = passedwp;
		
	std::cout << "Current waypoint is waypoint " << currwp << " at (" << waypointLats[currwp] << "N, " << waypointLongs[currwp] << "W)" << std::endl;

	R = 6371.00000; //Radius of the earth in km

	currentLat = passedLat;
	currentLong = passedLong;

	wpLat = waypointLats[currwp];
	wpLong = waypointLongs[currwp];

    dLat = currentLat - wpLat;
    dLong = currentLong - wpLong;

	//Haversine formula- calculate arc distance between two GPS points
	a = std::pow(std::sin(dLat/2.00000),2.00000) + std::cos(wpLat) * std::cos(currentLat) * std::pow(sin(dLong/2.00000),2.00000);
	calc = 2 * (std::atan2(std::sqrt(a), std::sqrt(1.00000-a)));
	arcDist = R * calc;

	std::cout << "Distance remaining: " << arcDist << " m" << std::endl;

	std::cout << "Current GPS: (" << currentLat << "N, " << currentLong << "W)" << std::endl;
	// std::cout << "Waypoint GPS: (" << wpLat << "N, " << wpLong << "W)" << std::endl;

	if (arcDist <= 3.00000){
		waypointFound == false;

		if (counter <= 1){
			std::cout << std::endl << "Waypoint " << currwp << " found. I'm here!" << std::endl << std::endl;

			waypointLats[currwp] == 0.00000;
			waypointLongs[currwp] == 0.00000;
			
			currwp = currwp + 1;
		}
		counter = counter + 1;
	}
	else{
		counter = 1;
	}

	return currwp;
}

float WaypointFinder::FindGPSHeading(float passedLat, float passedLong, int passWP){
	currentLat = passedLat;
	currentLong = passedLong;
	passWP = currwp;

	wpLat = waypointLats[currwp];
	wpLong = waypointLongs[currwp];

	//Find remaining distance to waypoint
	dremainLat = currentLat - wpLat;
	dremainLong = currentLong - wpLong;

	GPSHeading = std::atan(std::abs(dremainLong/dremainLat));
	// std::cout << (GPSHeading * 57.2957795) << " degrees to waypoint" << std::endl;
	return GPSHeading;
}

float WaypointFinder::FindNewHeading(float passedLat, float passedLong, float passedGPSHeading, double passedIMUHeading, int passWP){
	std::cout << std::setprecision(7); // show 16 digits

	currentLat = passedLat;
	currentLong = passedLong;
	currwp = passWP;
	GPSHeading = passedGPSHeading;
	IMUHeading = passedIMUHeading - 14.00000; // Account for magnetic declination

	wpLat = waypointLats[currwp];
	wpLong = waypointLongs[currwp];

	dremainLat = wpLat - currentLat;
	dremainLong = wpLong - currentLong;

	try{
		newHeading = (GPSHeading + IMUHeading)/2.00000;
		// newHeading = GPSHeading;

		if(newHeading <= std::tan(0.523598776)){ //atan(|y/x|) <= tan(30 deg)
			//If the heading angle to the next waypoint is less than 30 deg, hard turn left/right
			if(dremainLat == std::abs(dremainLat)){ //remaining x is positive, must turn right
				direction << "Hard right";
			}
			else{ //remaining x is negative, must turn left
				direction << "Hard left";
			}
		}
		else if(newHeading <= std::tan(1.04719755)){ //atan(|y/x|) <= tan(60 deg) and atan(|y/x|) >= tan(30 deg)
			//If the heading angle is less than 60 deg but greater than 30 deg, normal turn left/right.
			if(dremainLat == std::abs(dremainLat)){ //remaining x is positive, must turn right
				direction << "Mid right";
			}
			else{ //remaining x is negative, must turn left
				direction << "Mid left";
			}
		}
		else if(newHeading <= std::tan(1.48352986)){ //atan(|y/x|) <= tan(85 deg) and atan(|y/x|) >= tan(60 deg)
			//If the heading angle is less than 85 deg but greater than 60 deg, slight turn left/right.
			if(dremainLat == std::abs(dremainLat)){ //remaining x is positive, must turn right
				direction << "Slight right";
			}
			else{ //remaining x is negative, must turn left
				direction << "Slight left";
			}
		}
		else{ //If the waypoint is within a cone +-10 degrees from straight, the rover will drive straight ahead.
			direction << "Straight ahead";
		}
		std::cout << direction.str() << "! Angle is " << newHeading << std::endl << std::endl;
		// std::cout << 

		return newHeading;
	}
	catch (...){
		std::cout << "Error in FindNewHeading" << std::endl;
	}
}


/*######################################## CALLBACK FUNCTIONS ##############################################*/


void WaypointFinder::InputCallback(const std_msgs::Float64MultiArray::ConstPtr &msg){
	if(hasInput == false){
	 	hasInput = true;
	}

	waypointLat = msg->data[0];
	waypointLong = msg->data[1];

	std::cout << "Input exists" << std::endl;
}

void WaypointFinder::IMUCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg){
	headingx = msg->vector.x;
	headingy = msg->vector.y;
	headingz = msg->vector.z;

	compassHeading = (std::atan2(headingy,headingx) * 180.00000) / 3.14159265359;
  
	// Normalize to 0-360
	if (compassHeading < 0)
	{
	compassHeading = 360.00000 + compassHeading;
	}

	dCompassHeading.data = static_cast<double>(compassHeading);
	pubimu = dCompassHeading;
	// std::cout << "Current heading: " << compassHeading << std::endl;
}

void WaypointFinder::GPSCallback(const sensor_msgs::NavSatFix &msg){
	subIMU = nh.subscribe("/imu/mag", 10, &WaypointFinder::IMUCallback, this);
	if (waypointFound == true){ //If a waypoint was found...
		std::cout << "Next WP" << std::endl;
		waypointLats[currwp] = 0.00000; //Set the current waypoint latitude and longitude to default values, clearing them off the list.
		waypointLongs[currwp] = 0.00000;
		// counter = 1; //Reset previous heading weights in arbiter array (arbArray) to 0
		waypointFound = false;
	}
	else{

		currentLat = msg.latitude;
		currentLong = msg.longitude;

		IMUHeading = static_cast<double>(pubimu.data);

		currwp = WaypointFinder::Dist2WP(currentLat, currentLong, currwp);
		GPSHeading = WaypointFinder::FindGPSHeading(currentLat, currentLong, currwp);
		// std::cout << "GPS Heading: " << GPSHeading << std::endl;
		newHeading = WaypointFinder::FindNewHeading(currentLat, currentLong, GPSHeading, IMUHeading, currwp);

		dHeading.data = static_cast<double>(newHeading);
		pubheading = dHeading;

	}
	direction.str(std::string());
	direction.clear();
}


/*######################################## INIT FUNCTION AND MAIN LOOP ##############################################*/


void WaypointFinder::init(int argc, char* argv[]){
	//Initialize input flag
	hasInput = false;
	waypointFound = false;
	maxturnrad = 3;  // meters
	counter = 1;
	wpLat = 0;
	wpLong = 0;

	currwp = 0;

    std::cout << std::setprecision(7); // show 16 digits

	// int arbArray [7] = { 0,0,0,0,0,0,0 };

	waypointLats[0] = 42.29320;
	waypointLats[1] = 42.29338;
	waypointLats[2] = 20.00000;
	waypointLats[3] = 20.00000;
	waypointLats[4] = 20.00000;
	
	waypointLongs[0] = -71.26398;
	waypointLongs[1] = -71.26366;
	waypointLongs[2] = 0.00000;
	waypointLongs[3] = 0.00000;
	waypointLongs[4] = 20.00000;

	// subInput = nh.subscribe("/wplist", 10, &WaypointFinder::InputCallback, this); //You cannot run subInput and subWPfinder at the same time.

	subGPS = nh.subscribe("/fix", 10, &WaypointFinder::GPSCallback, this);

	// subIMU = nh.subscribe("/imu/mag", 10, &WaypointFinder::IMUCallback, this);

	pubvel = nh.advertise<std_msgs::String>("velocity", 1000);
	pubIMU = nh.advertise<std_msgs::Float64>("imu_data", 1000);
	pubHeading = nh.advertise<std_msgs::Float64>("heading", 1000);

	ros::Rate loop_rate(10);

	// int count = 0;
	while (ros::ok())
  	{
  		// direction << "Slight left";
  		// wp_pub.publish("");
  		pubdirection.data = direction.str();
	    pubvel.publish(pubdirection);

	    pubIMU.publish(pubimu);

	    ros::spinOnce();

	    loop_rate.sleep();

	    // ++count;
	    // std::cout << direction.str() << std::endl;
	    // std::cout << pubimu << std::endl;
	}

}