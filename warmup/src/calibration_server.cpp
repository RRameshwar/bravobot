#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <warmup/SensorFusionConfig.h>
#include <warmup/LidarCone.h>

ros::Publisher reconfig_pub;

void callback(warmup::SensorFusionConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d %d", config.left_limit, config.right_limit);
  warmup::LidarCone msg;
  msg.left_limit = config.left_limit;
  msg.right_limit = config.right_limit;
  reconfig_pub.publish(msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "calibration_server");
  
  ros::NodeHandle nh;

  dynamic_reconfigure::Server<warmup::SensorFusionConfig> server;
  dynamic_reconfigure::Server<warmup::SensorFusionConfig>::CallbackType f;

  reconfig_pub = nh.advertise<warmup::LidarCone>("dynamic_reconfigure/sensor_cone", 1);


  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}
