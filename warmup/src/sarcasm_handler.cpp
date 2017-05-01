#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <dirent.h>
#include <sstream>  // for string streams
// #include <tuple>
#include <typeinfo>

using namespace std;

class SarcasmSelect
{
    ros::NodeHandle n;
    ros::Subscriber area_sub;

    std::string curr_area;
    std::string filepath;
    // std::string foldername;
    // std::string length;
    std::string index_str;
    ostringstream temp_index;
    int index;
    // std::map <char, std::string> color_dict;
    
public:

    SarcasmSelect()
    {
        area_sub = n.subscribe<std_msgs::String>("/area_updates", 1000, &SarcasmSelect::areaCallback, this);
        // ros::Publisher filepath_pub = n.advertise<std_msgs::String>("wav_file", filepath);
    }
// int main (void)
// {
//   DIR *dp;
//   int i;
//   struct dirent *ep;     
//   dp = opendir ("./");

//   if (dp != NULL)
//   {
//     while (ep = readdir (dp))
//       i++;

//     (void) closedir (dp);
//   }
//   else
//     perror ("Couldn't open the directory");

//   printf("There's %d files in the current directory.\n", i);

//   return 0;
// }

    void areaCallback(const std_msgs::String::ConstPtr& msg)
    {
        ROS_INFO("I heard: [%s]", msg->data.c_str());
        curr_area = msg->data.c_str();

        std::string gen_filepath = generate_filepath(curr_area, "short");
        std::cout << "Playing filepath" << gen_filepath << std::endl;

        play_sound(gen_filepath);

        // ROS_INFO("I'm currently by the [%s]", curr_area);
    }

    std::string generate_filepath(std::string foldername, std::string length){
        index = rand() % 5; // Generates random file index # from 0-4

        // Sending a number as a stream into output string
        temp_index << index;
     
        // the str() coverts number into string
        index_str = temp_index.str();

        // std::cout << typeid(index_str).name() << std::endl;

        filepath = "./bravobot/warmup/voices/" + foldername + "/" + length + "/" + index_str;

        return filepath;
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


// void areaCallback(const std_msgs::String::ConstPtr& msg)
// {
//    ROS_INFO("I heard: [%s]", msg->data.c_str());
//     curr_area = msg->data.c_str();

//     // ROS_INFO("I'm currently by the [%s]", curr_area);
// }


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
