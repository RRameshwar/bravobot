#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "laser_geometry/laser_geometry.h"
#include "tf/transform_listener.h"

#include <sstream>

ros::Publisher movement_pub;
ros::Subscriber laser_sub;
ros::Publisher lidar_pub;
ros::Publisher pointcloud_pub;
float lastScan[512];

laser_geometry::LaserProjection projector_;
tf::TransformListener listener_;

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  if(!listener_.waitForTransform(
        scan_in->header.frame_id,
        "/base_link",
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(1.0))){
     return;
  }

  sensor_msgs::PointCloud cloud;
  projector_.transformLaserScanToPointCloud("/base_link",*scan_in,
          cloud,listener_);

  pointcloud_pub.publish(cloud);
  // Do something with cloud.
}

void laserScanCallback(sensor_msgs::LaserScan msg)
{
  int size = msg.ranges.size();

  float lidarDists[size];

  for(int i = 0; i < size; i++){
      double delta = fabs(msg.ranges[i] - lastScan[i]);
      if(delta < 0.3){
          lidarDists[i] = msg.ranges[i];
      } else{
          lidarDists[i] = std::numeric_limits<double>::infinity();
      }
      lastScan[i] = msg.ranges[i];
  }

  ROS_INFO("DATA SMOOTHED");

  // sensor_msgs::LaserScan msg;
  // float msg.ranges[size];
  for(unsigned int i = 0; i < size; ++i){
      if (i < size/4)
      {
        msg.ranges[i] = lidarDists[i];
      }
      else
      {
        msg.ranges[i] = std::numeric_limits<double>::infinity();
      }
  }

  ROS_INFO("PUBLISHING!");

  lidar_pub.publish(msg);
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "warmup_node");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  // ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  movement_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  laser_sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, laserScanCallback);
  lidar_pub = n.advertise<sensor_msgs::LaserScan>("/scan_smooth", 1000);
  pointcloud_pub = n.advertise<sensor_msgs::PointCloud>("/scan_pointcloud", 1000);
  ros::Rate loop_rate(10000);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;

  // geometry_msgs::Twist twist_msg;
  // twist_msg.linear.x = 1;
  // movement_pub.publish(twist_msg);

  // loop_rate.sleep();

  // geometry_msgs::Twist twist_msg1;
  // twist_msg1.linear.x = 0;
  // movement_pub.publish(twist_msg1);

  while (ros::ok())
  {
    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = 1;
    // movement_pub.publish(twist_msg);
     
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}