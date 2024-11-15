#include "air_bms/bms_node.hpp"


BatteryStatus::BatteryStatus(): Node{"bms_status_node"},bms_{"/ttyUSB0"}
{
    if (!bms_.Init())
    {
        RCLCPP_ERROR(this->get_logger(), "BMS initialization failed!");
        rclcpp::shutdown();
        return;
    }
    
    publisher_ = this->create_publisher<custom_interfaces::msg::BmsStatus>("bms_status",10);
    timer_     = this->create_wall_timer(std::chrono::seconds(1),std::bind(&BatteryStatus::BatteryStatusCallBack,this));
}

void BatteryStatus::BatteryStatusCallBack()
{
    if (!bms_.update())
    {
        RCLCPP_WARN(this->get_logger(),"Failed to update BMS data!");
        return;
    }
    

    // bool charging   = bms_.get.chargeFetState;
    // bool decharging = bms_.get.disChargeFetState;
    // bool charging   = 1;
    // bool decharging = 0;
    
    
    custom_interfaces::msg::BmsStatus msg;
    msg.battery_voltage     = bms_.get.packVoltage;
    msg.battery_current     = bms_.get.packCurrent;
    msg.battery_soc         = bms_.get.packSOC;
    msg.temp_average        = bms_.get.tempAverage;
    
    BatteryChargeState state;

    if (msg.battery_current < 0)
    {
        state = BatteryChargeState::DECHARGING;
    }
    else if (msg.battery_current > 0)
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
        msg.charge_status = "Stationary";
        break;
        case BatteryChargeState::CHARGING:
        msg.charge_status = "Charging";
        break;
        case BatteryChargeState::DECHARGING:
        msg.charge_status = "Decharging";
        break;
        default:
        msg.charge_status = "Unknown state!";
        break;
        }

    std::ostringstream log_bms;
    log_bms <<"[Basic BMS Data] \n"
            <<"Charge Status: "<<msg.charge_status<<"\n"
            <<"Battery Pack Voltage: "<<msg.battery_voltage<<" V \n"
            <<"Pack Current: "<<msg.battery_current<<" A \n"
            <<"State of Charge: "<<msg.battery_soc<<"%\n"
            <<"Pack Temperature: "<<msg.temp_average<<" °C\n";

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