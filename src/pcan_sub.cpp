#include "pcan_sub.hpp"

using std::placeholders::_1;

PcanSubscriber::PcanSubscriber()
    : Node("PCAN_Subscriber"), mBrake(0.0), mSteering(0.0), mThrottle(0.0)
{
    mSubscription = this->create_subscription<std_msgs::msg::ByteMultiArray>(
        "topic", 10, std::bind(&PcanSubscriber::TopicCallback, this, _1));
}

void PcanSubscriber::TopicCallback(const std_msgs::msg::ByteMultiArray &msg)
{
    // Logging Received data
    const unsigned char *data_ptr = msg.data.data();
    size_t data_size = msg.data.size();
    std::string id = GetIDString(data_ptr);

    std::stringstream ss;
    ss << "Received CAN data " << id << ": ";
    for (size_t i = 0; i < data_size; i++)
    {
        ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(data_ptr[i]) << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());

    // Parsing CAN data
    switch (std::stoi(id))
    {
    case 111: // Brake
        Parser111(data_ptr);
        break;
    case 710: // Steering
        Parser710(data_ptr);
        break;
    case 711: // Throttle
        Parser711(data_ptr);
        break;
    }
}

std::string PcanSubscriber::GetIDString(const unsigned char *data_ptr) const
{
    char strTemp[11] = {0};
    std::string id = "";
    for (int i = 9; i > 7; i--)
    {
        sprintf(strTemp, "%X", data_ptr[i]);
        id.append(strTemp);
    }

    return id;
}

void PcanSubscriber::Parser111(const unsigned char *data_ptr)
{
    unsigned char byte1 = static_cast<unsigned char>(data_ptr[0] & 0xFF);
    double decimalValue = double(byte1) / 255.0;
    mBrake = decimalValue;
    // RCLCPP_INFO(this->get_logger(), "Brake: %f", mBrake);
}

void PcanSubscriber::Parser710(const unsigned char *data_ptr)
{
    // 0 ~ 255
    unsigned char byte1 = static_cast<unsigned char>(data_ptr[1] & 0xFF);
    unsigned char byte2 = static_cast<unsigned char>(data_ptr[2] & 0xFF);

    // Steering값 하나로 합치기
    // unsigned short range: 0 ~ 65535
    unsigned short value = (byte2 << 8) | byte1;
    // std::cout << "value: " << value << std::endl;

    /// byte1이 0~255까지 돌면 byte2가 1 증가
    /// +-90도 기준으로 2700 ~ 840
    /// 중간값은 1770
    /// 값은 실험할때마다 달라지므로 사용시기에 따라 달라질 수 있음 (2023.04.05 기준 정문규)
    /// uc-win에서 steering은 -1 에서 1 사이의 값만 받기 때문에 변경
    /// 840 ~ 2700 -> -1 ~ 1
    /// 왼쪽을 -1, 오른쪽을 1로 설정하기 때문에 마지막 mSteering에 -1 추가
    const unsigned short middle = 1770;
    const unsigned short max = 2700;
    const unsigned short diff = max - middle;
    if (value > 2700)
    {
        mSteering = -1.0;
    }
    else if (value < 840)
    {
        mSteering = 1.0;
    }
    else
    {
        mSteering = -(static_cast<double>(value) - middle) / diff;
    }
    // RCLCPP_INFO(this->get_logger(), "Steering: %f", mSteering);
}

void PcanSubscriber::Parser711(const unsigned char *data_ptr)
{
    unsigned char byte1 = static_cast<unsigned char>(data_ptr[5] & 0xFF);
    unsigned char byte2 = static_cast<unsigned char>(data_ptr[6] & 0xFF);

    // Throttle값 하나로 합치기
    // unsigned short range: 0 ~ 65535
    unsigned short value = (byte2 << 8) | byte1;
    // std::cout << "value: " << value << std::endl;

    /// Throttle 범위가 들쭉날쭉하기 때문에 현재 기준으로 작성 (2023.04.03 기준 정문규)
    /// 620 ~ 3480 이 범위 밖은 0 또는 1 처리
    /// steering과 마찬가지로 0 ~ 1로 변경
    const unsigned short min = 620;
    const unsigned short max = 3480;
    const unsigned short diff = max - min;
    if (value < 620)
    {
        mThrottle = 0;
    }
    else if (value > 3480)
    {
        mThrottle = 1;
    }
    else
    {
        mThrottle = (static_cast<double>(value) - min) / diff;
    }
    // RCLCPP_INFO(this->get_logger(), "Throttle: %f", mThrottle);
}