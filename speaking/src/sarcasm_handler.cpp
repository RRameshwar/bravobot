#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "boost/filesystem.hpp"
#include <iterator> // std::distance
#include <sstream>  // for string streams
#include <time.h>
#include <wordexp.h> // turning ~ -> /home/odroid
#include "speaking/PlayVoice.h"


class SarcasmSelect
{
    ros::NodeHandle n;
    ros::Subscriber area_sub;
    ros::ServiceServer service;

    std::string curr_area;
    std::string filepath;
    std::string index_str;
    int index;
    
public:

    SarcasmSelect()
    {
        area_sub = n.subscribe<std_msgs::String>(
            "/area_updates",
            1,
            &SarcasmSelect::areaCallback,
            this);

        service = n.advertiseService("play_sarcasm",
                                     &SarcasmSelect::serviceCallback,
                                     this);
    }

    void areaCallback(const std_msgs::String::ConstPtr& msg)
    {
        ROS_INFO("I heard: [%s]", msg->data.c_str());
        curr_area = msg->data.c_str();
    }

    int numFilesInDir(std::string filepath) {
   	
	// expand ~/catkin_ws... to /home/odroid/catkin_ws... 
	wordexp_t exp_result;
	wordexp(filepath.c_str(), &exp_result, 0);

        int num_files = std::distance(
            boost::filesystem::directory_iterator(exp_result.we_wordv[0]),
            boost::filesystem::directory_iterator());

        return num_files;
    }

    bool generate_filepath(
            std::string foldername,
            std::string length,
            std::string &filepath)
    {
        srand ( time(NULL) );

        filepath = "~/catkin_ws/src/bravobot/speaking/voices/" + foldername + "/" + length + "/";

        std::cout << filepath << std::endl;
        
        // find number of files in the directory
        int num_files;
        try
        {
            num_files = numFilesInDir(filepath);
        }
        catch (boost::filesystem::filesystem_error& e)
        {
            // directory does not exist
            std::cout << e.what() << std::endl;
            return false; // false on failure
        }

        // Generates random file index # from 1 to n-1
        index = (rand() % num_files) + 1;
        // Sending a number as a stream into output string
        std::ostringstream temp_index;
        temp_index << index;
     
        // the str() coverts number into string
        index_str = temp_index.str();
        
        filepath = filepath + index_str + ".wav";

        return true; // true on success
    }

    void play_sound(std::string sound_filepath){
        try{
            std::system(("aplay " + sound_filepath).c_str());
        }
        catch(int e){
            std::cout << "File doesn't exist." << std::endl;
        }
        return;
    }

    bool serviceCallback(speaking::PlayVoice::Request &req,
                         speaking::PlayVoice::Response &res)
    {
        std::string gen_filepath;
        bool success = generate_filepath(curr_area,
                                         req.length,
                                         gen_filepath);
        if (success)
        {
            std::cout << "Playing filepath" << gen_filepath << std::endl;

            play_sound(gen_filepath);

            return true;
        }
        else
        {
            return false;
        } 
    }

};


int main(int argc, char **argv)
{

    ros::init(argc, argv, "sarcasm_handler");

    SarcasmSelect ss;


    ros::spin();

    return 0;
}

