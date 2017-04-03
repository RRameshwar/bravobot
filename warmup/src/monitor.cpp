// #include "ros/ros.h"
#include <stdio.h>
#include <iostream>
#include <boost/thread/thread.hpp>

// int main(int argc, char **argv){
// // ros::NodeHandle nh_;
// // ros::Rate r(10);
// char input[2] == NULL;
// ros::init(argc, argv, "monitor");


////Testing basic keyboard press break statement

// std::cout << "Press Enter to Continue";
// // std::cin.ignore();
// std::cin.get(input,2);
// std::cout << input << std::endl;


// while (ros::ok()){
// 	std::cout << "...";
// 	std::cin.get(input,2);
// 	if(input != NULL){
// 		break;
// 	}
  	
//   	ros::spinOnce();
//   	r.sleep();

// }

// std::cout << "Yayy" << std::endl;

// }


void loop2(int passed_stop) {
    while(true){
	    if(passed_stop == 0){
	        std::cout << "..." << std::endl;
	        usleep(1000); //Pause after execution to allow main thread to set stop equal to 1 & print a statement.
	    }
	}
}


// ...

int main(){
	int stop = 0;
    boost::thread t(loop2,stop); // Separate thread for loop.

    // Main thread waits for input character
    while (true){
	    if (std::cin.get() == '\n'){

	    std::cout << std::endl << std::endl << "Finished!" << std::endl;
	    // Set the boolean to true. The loop thread will exit from the loop and terminate.
	    break;

		}
	}

	stop = 1;
    return 0; 
}