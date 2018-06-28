#include "ros/ros.h"
#include "std_msgs/Int8.h"

#include "motor_control.h"

#include "gpio_ros_stub.h"
#include "gpio_pwm_ros_stub.h"

void MotorSpeedCallback(const std_msgs::Int8::ConstPtr& msg)
{
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pwm_motor_control");
    ros::NodeHandle n;

    ros::Subscriber motor_speed_subscibrer = n.subscribe<std_msgs::Int8>("motor_speed",
                                                                         10, 
                                                                         MotorSpeedCallback);

    uint8_t pin_output_pwm = 1U;
    uint8_t pin_output_direction_1= 2U;
    uint8_t pin_output_direction_2= 3U;

    auto output_pwm = std::make_unique<RosGpioPwm>(pin_output_pwm,
                                                   n,
                                                   "pwm/out");
    auto output_direction_1 = std::make_unique<RosGpio>(pin_output_direction_1,
                                                        n,
                                                        "pwm/direction_1");
    auto output_direction_2 = std::make_unique<RosGpio>(pin_output_direction_2,
                                                        n,
                                                        "pwm/direction_2");

    auto motor_control = PwmMotorControl(std::move(output_pwm),
                                         std::move(output_direction_1),
                                         std::move(output_direction_2));

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        motor_control.SetSpeed(0);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
