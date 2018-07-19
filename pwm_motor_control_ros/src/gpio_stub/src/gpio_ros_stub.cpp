#include "gpio_ros_stub.h"
#include "std_msgs/UInt8.h"

RosGpio::RosGpio(const uint8_t pin,
                 ros::NodeHandle& node,
                 const std::string& topic_name)
    : GpioPin(pin)
{
    pin_publisher = node.advertise<std_msgs::UInt8>(topic_name, 10);
}

std_msgs::UInt8 CreateMessage(const uint8_t value)
{
    std_msgs::UInt8 msg;
    msg.data = value;
    return msg;
}

void RosGpio::WriteToPin(const uint8_t value)
{
    pin_publisher.publish(CreateMessage(value));
}

void RosGpio::WriteHighToPin()
{
    pin_publisher.publish(CreateMessage(255U));
}

void RosGpio::WriteLowToPin()
{
    pin_publisher.publish(CreateMessage(0U));
}