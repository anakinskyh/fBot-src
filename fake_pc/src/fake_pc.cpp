
#include <pluginlib/class_loader.h>
#include <stdio.h>
// ROS
#include <ros/ros.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_field_conversion.h>

#include <sensor_msgs/ChannelFloat32.h>
#include <geometry_msgs/Point32.h>

int main(int argc, char **argv)
{
    ros::Time::init();
    ros::init(argc, argv, "robot_model_to_pointcloud");

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh("~");

    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("robot_cloud2", 1);

    sensor_msgs::PointCloud cloud;
    cloud.points.reserve(5000);
    sensor_msgs::PointCloud2 cloud2;
    geometry_msgs::Point32 pt;
    Eigen::Vector3d vertice;
    Eigen::Vector3d vertice_transformed;
    ros::Duration elapsed;
    ros::Duration sleep_time(1. / publish_frequency);

    sensor_msgs::ChannelFloat32 link_info_channel;
    link_info_channel.name = "intensity";
    link_info_channel.values.reserve(5000);
}