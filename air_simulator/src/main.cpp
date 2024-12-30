#include "air_simulator/scanMerger.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarMerger>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}