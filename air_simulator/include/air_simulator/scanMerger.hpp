#pragma once 

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
#include <algorithm>

class LidarMerger : public rclcpp::Node{

    public:
    LidarMerger();

    private:
    void lidar_0_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void lidar_1_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void merge_scans();
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr merged_scan_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_0_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_1_sub_;

    sensor_msgs::msg::LaserScan::SharedPtr lidar_0_scan_;
    sensor_msgs::msg::LaserScan::SharedPtr lidar_1_scan_;


};