#ifndef BMS_NODE_
#define BMS_NODE_

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/bms_status.hpp"
#include "air_bms/bms_uart.hpp"


class BatteryStatus : public rclcpp::Node{


    public:
    BatteryStatus();


    private:
    void BatteryStatusCallBack();
    rclcpp::Publisher<custom_interfaces::msg::BmsStatus>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    BMS_UART bms_; 
    
    enum class BatteryChargeState{
        STATIONARY, 
        CHARGING,
        DECHARGING 

    };
};


#endif //BMS_NODE_