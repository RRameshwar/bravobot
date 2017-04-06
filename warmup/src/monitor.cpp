#include "ros/master.h"
// #include "ros-keyboard.h"
#include <stdio.h>
#include <iostream>
#include <boost/thread/thread.hpp>

// void run_databags(){
// /*Plays a rosbag on loop, while terminal output is silenced to prevent blocking the cout channel.*/

// 	std::cout << std::endl << "Playing rosbag lydia1.bag." << std::endl;
// 	std::system("rosbag play ~/catkin_ws/src/bravobot/warmup/rosbags/lydia1.bag -q"); 
// }

void load_programs(int passed_ran_calib, int passed_ran_track) {
	/*Executes system commands that run rosnodes one at a time, according to set 'ran_X' parameters in main function.*/

	if (passed_ran_calib == 0){
		std::cout << std::endl << "Listener node booting up." << std::endl;
		std::system("rosrun listener listener");
	}

	if (passed_ran_track == 0){
		std::cout << std::endl << "Talker node booting up." << std::endl;
		std::system("rosrun talker talker");
	}

	std::cout << std::endl << "All programs executed." << std::endl;

//WORKING CODE
 //    while(true){
	//     if(passed_stop == 0){
	//         std::cout << "..." << std::endl;
	//         usleep(1000); //Pause after execution to allow main thread to set stop equal to 1 & print a statement.
	//     }
	// }
}

int main(){

	int ran_calib = 0; //Did not run calibration
	int ran_track = 0; //Did not run tracking

	// boost::thread rosbags(run_databags);

	boost::thread programs(load_programs,ran_calib, ran_track);

	std::cout << std::endl << "Running main program thread." << std::endl;

    // Main thread waits for "Enter" keypress.
    while (true){
    	// std::cout << std::endl << "DATA: " << std::cin.get() << std::endl; //Here for debug

	    if (std::cin.get() == '\n'){
			std::cout << std::endl << "Listener node shutting down." << std::endl;
		    std::system("rosnode kill listener listener");
		    ran_calib = 1; // Ran calibration
		}
		if (std::cin.get() == '\n'){
			std::cout << std::endl << "Talker node shutting down." << std::endl;
		    std::system("rosnode kill talker talker");
		    ran_track = 1; // Ran tracking software
		}
		if (ran_calib == 1 && ran_track == 1){
			std::cout << std::endl << "Program completed!" << std::endl;
			break;
		}
	}

	return 0;

}