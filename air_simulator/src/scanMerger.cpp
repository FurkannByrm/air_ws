#include "air_simulator/scanMerger.hpp"


LidarMerger::LidarMerger() : Node{"scan"}
{

    merged_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_merger",10);
    lidar_0_sub_     = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan_0",10, std::bind(&LidarMerger::lidar_0_callback, this, std::placeholders::_1));
    lidar_1_sub_     = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan_1",10, std::bind(&LidarMerger::lidar_1_callback, this, std::placeholders::_1));

}

void LidarMerger::lidar_0_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
        lidar_0_scan_ = msg;
        merge_scans();
}

void LidarMerger::lidar_1_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
        lidar_1_scan_ = msg;
        merge_scans();

}

void LidarMerger::merge_scans()
{
       
        if (!lidar_0_scan_ || !lidar_1_scan_)
            return;

        auto merged_scan = std::make_shared<sensor_msgs::msg::LaserScan>();
        merged_scan->header.stamp = this->get_clock()->now();
        merged_scan->header.frame_id = "base_link"; 

        merged_scan->angle_min = lidar_0_scan_->angle_min;
        merged_scan->angle_max = lidar_1_scan_->angle_max;
        merged_scan->angle_increment = lidar_0_scan_->angle_increment;
        merged_scan->time_increment = lidar_0_scan_->time_increment;
        merged_scan->scan_time = lidar_0_scan_->scan_time;
        merged_scan->range_min = std::min(lidar_0_scan_->range_min, lidar_1_scan_->range_min);
        merged_scan->range_max = std::max(lidar_0_scan_->range_max, lidar_1_scan_->range_max);

        merged_scan->ranges = lidar_0_scan_->ranges;
        merged_scan->ranges.insert(
            merged_scan->ranges.end(),
            lidar_1_scan_->ranges.begin(),
            lidar_1_scan_->ranges.end());

        merged_scan->intensities = lidar_0_scan_->intensities;
        merged_scan->intensities.insert(
            merged_scan->intensities.end(),
            lidar_1_scan_->intensities.begin(),
            lidar_1_scan_->intensities.end());

        merged_scan_pub_->publish(*merged_scan);

}