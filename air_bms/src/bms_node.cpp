#include "air_bms/bms_node.hpp"


BatteryStatus::BatteryStatus(): Node{"bms_status"},bms_{"/dev/ttyUSB0"}
{
    if (!bms_.Init())
    {
        RCLCPP_ERROR(this->get_logger(), "BMS initialization failed!");
        rclcpp::shutdown();
        return;
    }
    
    publisher_ = this->create_publisher<sensor_msgs::msg::BatteryState>("bms_status",10);
    timer_     = this->create_wall_timer(std::chrono::seconds(1),std::bind(&BatteryStatus::BatteryStatusCallBack,this));
}

void BatteryStatus::BatteryStatusCallBack()
{
    // if (!bms_.update())
    // {
    //     RCLCPP_WARN(this->get_logger(),"Failed to update BMS data!");
    //     return;
    // }
    bms_.update();
    
    sensor_msgs::msg::BatteryState msg;
    msg.voltage      = bms_.get.packVoltage;
    msg.current     = bms_.get.packCurrent;
    msg.percentage  = bms_.get.packSOC;

    BatteryChargeState state;

    if (msg.current < 0) 
    {
        state = BatteryChargeState::DECHARGING;
    }
    else if (msg.current > 0)
    {
        state = BatteryChargeState::CHARGING;
    }
    else
    {
        state = BatteryChargeState::STATIONARY;
    }
    
    
    switch(state)
    {
        case BatteryChargeState::STATIONARY:
        msg.location = "Stationary";
        break;
        case BatteryChargeState::CHARGING:
        msg.location = "Charging";
        break;
        case BatteryChargeState::DECHARGING:
        msg.location = "Decharging";
        break;
        default:
        msg.location = "Unknown state!";
        break;
        }

    std::ostringstream log_bms;
    log_bms <<"[ Charge Status ]       : "<<msg.location<<"\n"
            <<"[ Voltage ]             : "<<msg.voltage<<"V \n"
            <<"[ Current ]             : "<<msg.current<<"A \n"
            <<"[ State of Charge ]     : "<<msg.percentage<<"%\n";
            

    RCLCPP_INFO(this->get_logger(),"\n%s",log_bms.str().c_str());
    publisher_->publish(msg);

}

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<BatteryStatus>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;    
}