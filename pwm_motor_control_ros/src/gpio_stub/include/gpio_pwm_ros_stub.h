#ifndef GPIO_PWM_ROS_STUB_H
#define GPIO_PWM_ROS_STUB_H

#include <string.h>

#include "ros/ros.h"

#include "gpio_pwm.h"

class RosGpioPwm : public GpioPwmPin
{
public:
    RosGpioPwm(const uint8_t pin,
               ros::NodeHandle& node,
               const std::string& topic_name);

private:
    void WriteDutyCycleToPin(const uint16_t value);
    void SetRangeOnDevice(const uint16_t range);

    ros::Publisher pin_publisher;
};

#endif