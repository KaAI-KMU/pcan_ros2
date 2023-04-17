#include "pcan_pub.hpp"

using namespace std::chrono_literals;

PcanPublisher::PcanPublisher()
	: Node("PCAN_Publisher")
{

	TPCANStatus stsResult;

	stsResult = CAN_Initialize(PcanHandle, Bitrate);
	if (stsResult != PCAN_ERROR_OK)
	{
		std::cout << "Can not initialize. Please check the defines in the code.\n";
		std::cout << "\n";
		std::cout << "Closing...\n";
		std::cout << "Press any key to continue...\n";
		_getch();
		return;
	}

	mPublisher = this->create_publisher<std_msgs::msg::ByteMultiArray>("topic", 10);
	mTimer = this->create_wall_timer(
		0ms, std::bind(&PcanPublisher::ReadMessages, this));
}

PcanPublisher::~PcanPublisher()
{
	CAN_Uninitialize(PCAN_NONEBUS);
}

void PcanPublisher::ReadMessages()
{
	TPCANStatus stsResult;

	stsResult = ReadMessage();
	if (stsResult != PCAN_ERROR_OK && stsResult != PCAN_ERROR_QRCVEMPTY)
	{
		std::cout << "Error in ReadMessages()" << std::endl;
		return;
	}
}

TPCANStatus PcanPublisher::ReadMessage()
{
	TPCANMsg CANMsg;
	TPCANTimestamp CANTimeStamp;

	// We execute the "Read" function of the PCANBasic
	TPCANStatus stsResult = CAN_Read(PcanHandle, &CANMsg, &CANTimeStamp);
	if (stsResult != PCAN_ERROR_QRCVEMPTY)
		ValidateID(CANMsg);

	return stsResult;
}

void PcanPublisher::ValidateID(TPCANMsg msg)
{
	const UINT32 id = msg.ID;
	if (id == 0x710 ||
		id == 0x711 ||
		id == 0x111)
	{
		PublishCANMsg(msg);
	}
}

void PcanPublisher::PublishCANMsg(TPCANMsg msg)
{
	// Logging CAN data
	auto id = std_msgs::msg::String();
	auto canData = std_msgs::msg::String();
	id.data = GetIdString(msg.ID, msg.MSGTYPE);
	canData.data = GetDataString(msg.DATA, msg.MSGTYPE, msg.LEN);
	RCLCPP_INFO(this->get_logger(), "%s: %s", id.data.c_str(), canData.data.c_str());

	// Publish CAN data
	auto pubMsg = std_msgs::msg::ByteMultiArray();
	std::vector<BYTE> data_vector(msg.DATA, msg.DATA + msg.LEN);
	data_vector.push_back(static_cast<BYTE>(msg.ID & 0xFF));		// 하위 8비트 추가
	data_vector.push_back(static_cast<BYTE>((msg.ID >> 8) & 0xFF)); // 상위 8비트 추가

	pubMsg.data = data_vector;
	mPublisher->publish(pubMsg);
}

std::string PcanPublisher::GetIdString(UINT32 id, TPCANMessageType msgType)
{
	char result[MAX_PATH] = {0};
	if ((msgType & PCAN_MESSAGE_EXTENDED) == PCAN_MESSAGE_EXTENDED)
	{
		sprintf_s(result, sizeof(result), "%08Xh", id);
		return result;
	}
	sprintf_s(result, sizeof(result), "%03Xh", id);
	return result;
}

std::string PcanPublisher::GetDataString(BYTE data[], TPCANMessageType msgType, int dataLength)
{
	if ((msgType & PCAN_MESSAGE_RTR) == PCAN_MESSAGE_RTR)
		return "Remote Request";
	else
	{
		char strTemp[MAX_PATH] = {0};
		std::string result = "";
		for (int i = 0; i < dataLength; i++)
		{
			sprintf_s(strTemp, sizeof(strTemp), "%02X ", data[i]);
			result.append(strTemp);
		}

		return result;
	}
}