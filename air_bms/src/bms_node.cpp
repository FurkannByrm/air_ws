#include "air_bms/bms_node.hpp"


BatteryStatus::BatteryStatus(): Node{"bms_status"},MAX_TEMP_THRESHOLD_{45},STATUS_{"UNKNOW"},Battery_capacity_{46},bms_{"/dev/ttyUSB0"}
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
    msg.voltage                 = bms_.get.packVoltage;
    msg.current                 = bms_.get.packCurrent;
    msg.percentage              = bms_.get.packSOC;
    msg.temperature             = bms_.get.tempAverage;
    msg.design_capacity         = Battery_capacity_; 
    msg.charge                  = bms_.get.resCapacitymAh / 1000.0;
    msg.capacity                = Battery_capacity_;
    msg.present                 = true;
    msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    msg.power_supply_status     = bms_.get.chargeState ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING : 
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    msg.cell_voltage.resize(bms_.get.numberOfCells);

    for (int i = 0; i < bms_.get.numberOfCells; ++i)
    {
        msg.cell_voltage[i] = bms_.get.cellVmV[i] / 1000.0;
    }
    
    if (bms_.get.tempMax > MAX_TEMP_THRESHOLD_ )
    {
        msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERHEAT;
    }
    else
    {
        msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    }
    
    

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
        STATUS_ = "STATIONARY";
        break;
        case BatteryChargeState::CHARGING:
        STATUS_ = "CHARGING";
        break;
        case BatteryChargeState::DECHARGING:
        STATUS_ = "DECHARGING";
        break;
        }

    std::ostringstream log_bms;
    log_bms <<"[ Charge Status ]       : "<<STATUS_<<"\n"
            <<"[ Voltage ]             : "<<msg.voltage<<"V \n"
            <<"[ Current ]             : "<<msg.current<<"A \n"
            <<"[ State of Charge ]     : "<<msg.percentage<<"%\n"
            <<"[ Average Temperature ] : "<< msg.temperature << "°C\n";
            

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