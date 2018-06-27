#ifndef GPIO_ROS_STUB_H
#define GPIO_ROS_STUB_H

#include <string.h>

#include "ros/ros.h"

#include "gpio.h"

class RosGpio : public GpioPin
{
public:
    RosGpio(const uint8_t pin,
            ros::NodeHandle& node,
            const std::string& topic_name);

private:
    void WriteToPin(const uint8_t value);
    void WriteHighToPin();
    void WriteLowToPin();

    ros::Publisher pin_publisher;
};

#endif