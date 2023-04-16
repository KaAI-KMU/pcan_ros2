#pragma once
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// PCAN-Basic header
#include "linux_interop.h"
#include "PCANBasic.h"

// ROS2 header
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class PcanPublisher : public rclcpp::Node
{
public:
    PcanPublisher();
    ~PcanPublisher();

private:
    rclcpp::TimerBase::SharedPtr mTimer;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mPublisher;

    // PCAN-Basic const value
    const TPCANHandle PcanHandle = PCAN_USBBUS1;
    const TPCANBaudrate Bitrate = PCAN_BAUD_500K;

    // PCAN-Basic function
    void ReadMessages();
    TPCANStatus ReadMessage();
    void ProcessMessageCan(TPCANMsg msg);
    std::string GetIdString(UINT32 id, TPCANMessageType msgType);
    std::string GetDataString(BYTE data[], TPCANMessageType msgType, int dataLength);
};