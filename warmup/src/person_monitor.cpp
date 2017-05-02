#include <cmath>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>

class Monitor{

    double dx;
    double dy;
    double distance;
public:

    ros::NodeHandle nh_;

    ros::Publisher personmonitor_pub_;
    tf::TransformListener listener;
    tf::StampedTransform current_transform;
    tf::StampedTransform old_transform;

    bool tf_success;

    Monitor()
    {
        personmonitor_pub_ = nh_.advertise<std_msgs::Bool>("stop", 10);
    }

    void MonitorPerson(){

        tf_success = true;

        try{
            listener.lookupTransform("/person", "/map",
                                     ros::Time(0), current_transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            tf_success = false;
        }
        try {
            listener.lookupTransform("/person", "/map",
                                     ros::Time::now()-ros::Duration(5), old_transform);
        }
        catch (...){//tf::TransformException ex) {
            std::cout << "error" << std::endl;
            //ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            tf_success = false;
        }

        if (!tf_success) {
            personstopped(false);
            return;
        }

        dx = old_transform.getOrigin().x() - current_transform.getOrigin().x();
        dy = old_transform.getOrigin().y() - current_transform.getOrigin().y();
        distance = sqrt(pow(dx,2)+pow(dy,2));
        if(distance<0.2){   //if person moved less than 0.2 metres
            personstopped(true);
        }
        else{
            personstopped(false);
        }
        return;
    }

    void personstopped(bool msg){
        if(msg){
            std::cout << "person stopped moving" << std::endl;
        }
        std_msgs::Bool pub_msg;
        pub_msg.data = msg;
        personmonitor_pub_.publish(pub_msg);
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "transform_listener");
    Monitor pm;

    ros::Rate rate(10.0);
    while (pm.nh_.ok()){
        pm.MonitorPerson();
        rate.sleep();
    }

    return 0;
}