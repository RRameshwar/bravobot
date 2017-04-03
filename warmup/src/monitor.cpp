#include "ros/master.h"
#include <stdio.h>
#include <iostream>
#include <boost/thread/thread.hpp>

void load_programs(int passed_ran_calib, int passed_ran_track) {

	if (passed_ran_calib == 0){
		std::cout << std::endl << "Talker node booting up." << std::endl;
		std::system("rosrun talker talker");
	}
	if (passed_ran_track == 0){
		std::cout << std::endl << "Listener node booting up." << std::endl;
		std::system("rosrun listener listener");
	}

	std::cout << std::endl << "Program complete." << std::endl;

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

	boost::thread t(load_programs,ran_calib, ran_track);

    // Main thread waits for "Enter" keypress.
    while (true){
	    if (std::cin.get() == '\n'){
			std::cout << std::endl << "Talker node shutting down now." << std::endl;
		    std::system("rosnode kill talker talker");
		}
		if (std::cin.get() == '\n'){
			std::cout << std::endl << "Listener node shutting down now." << std::endl;
		    std::system("rosnode kill listener listener");
		}
	}

	ran_calib = 1; // Set the boolean to true. The loop thread will exit from the loop and terminate.
	return 0;

}