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

  mPublisher = this->create_publisher<std_msgs::msg::String>("topic", 10);
  mTimer = this->create_wall_timer(
    500ms, std::bind(&PcanPublisher::ReadMessages, this));
}

PcanPublisher::~PcanPublisher()
{
	CAN_Uninitialize(PCAN_NONEBUS);
}

void PcanPublisher::ReadMessages()
{
  TPCANStatus stsResult;

	// We read at least one time the queue looking for messages. If a message is found, we look again trying to
	// find more. If the queue is empty or an error occurr, we get out from the dowhile statement.
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
	ProcessMessageCan(CANMsg);

  return stsResult;
}

void PcanPublisher::ProcessMessageCan(TPCANMsg msg)
{
	auto id = std_msgs::msg::String();
	auto message = std_msgs::msg::String();

	id.data = GetIdString(msg.ID, msg.MSGTYPE);
  	message.data = GetDataString(msg.DATA, msg.MSGTYPE, msg.LEN);
	
  	RCLCPP_INFO(this->get_logger(), "%s: CAN: %s", id.data.c_str(), message.data.c_str());
  	mPublisher->publish(message);
}

std::string PcanPublisher::GetIdString(UINT32 id, TPCANMessageType msgType)
{
	char result[MAX_PATH] = { 0 };
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
		char strTemp[MAX_PATH] = { 0 };
		std::string result = "";
		for (int i = 0; i < dataLength; i++)
		{
			sprintf_s(strTemp, sizeof(strTemp), "%02X ", data[i]);
			result.append(strTemp);
		}

		return result;
	}
}