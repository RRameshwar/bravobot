#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "boost/filesystem.hpp"
#include <iterator> // std::distance
#include <sstream>  // for string streams
// #include <tuple>
#include <time.h>
#include <wordexp.h> // turning ~ -> /home/odroid


class SarcasmSelect
{
    ros::NodeHandle n;
    ros::Subscriber area_sub;

    std::string curr_area;
    std::string filepath;
    // std::string foldername;
    // std::string length;
    std::string index_str;
    int index;
    // std::map <char, std::string> color_dict;
    
public:

    SarcasmSelect()
    {
        area_sub = n.subscribe<std_msgs::String>("/area_updates", 1000, &SarcasmSelect::areaCallback, this);
        // ros::Publisher filepath_pub = n.advertise<std_msgs::String>("wav_file", filepath);
    }

    void areaCallback(const std_msgs::String::ConstPtr& msg)
    {
        ROS_INFO("I heard: [%s]", msg->data.c_str());
        curr_area = msg->data.c_str();

        std::string gen_filepath;
        bool success = generate_filepath(curr_area, "short", gen_filepath);
        if (success)
        {
            std::cout << "Playing filepath" << gen_filepath << std::endl;

            play_sound(gen_filepath);
        }
        // ROS_INFO("I'm currently by the [%s]", curr_area);
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

    bool generate_filepath(std::string foldername, std::string length, std::string &filepath){
        srand ( time(NULL) );

        filepath = "~/catkin_ws/src/bravobot/warmup/voices/" + foldername + "/" + length + "/";

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

        
        index = (rand() % num_files) + 1; // Generates random file index # from 1 to n-1
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

};


int main(int argc, char **argv)
{

    ros::init(argc, argv, "sarcasm_handler");

    SarcasmSelect ss;

    // while (ros::ok){
    //     // Insert code for determining long or short comment
    //     // std::string gen_filepath = ss.generate_filepath("Robolab", "short");
    //     std::cout << "Play file: " << gen_filepath << std::endl;
    // }

    ros::spin();

    return 0;
}

// class SarcasmSelect
// {

//     char index[4];
//     std::map <char, string> color_dict;

//     color_dict["212"] = 'Blossom Pink'; // Scrap plywood carts
//     color_dict["002"] = 'Blue';         // Drew's lab
//     color_dict["022"] = 'Cyan';         // A+A studio
//     color_dict["020"] = 'Green';        // Dave's lab
//     color_dict["112"] = 'Lavender';     // Inaccesible areas
//     color_dict["202"] = 'Magenta';      // Sharks + marine life above Robolab
//     color_dict["100"] = 'Maroon';       // ADE project and whiteboard dump
//     color_dict["210"] = 'Orange';       // Boats
//     color_dict["200"] = 'Red';          // Robolab screen
//     color_dict["021"] = 'Sea Green';    // Investigating Normal wall
//     color_dict["012"] = 'Sky Blue';     // Garbage bin
//     color_dict["120"] = 'Spring Green'; // SCOPE wall
//     color_dict["220"] = 'Yellow';       // SCOPE posters

// public:

//     int rgb_to_dict(passed_rgb, passed_color){
//         if(color[passed_rgb] = 0){
//             return 0;
//         }
//         else if(color[passed_rgb] > 0){
//             return 1;
//         }
//         else(color[passed_rgb] > 135){
//             return 2;
//         }
//     }

//     char color_id(passed_zone_rgb){
//         for (int rgb = 0:2){
//             digit = rgb_to_dict(rgb, color);
//             strcat(index,digit);
//         }

//         return color_dict[index];
//     }
// };

// int main(int argc, char** argv) 
// {
//     string hallway_region = color_id(zone_rgb);
// }
