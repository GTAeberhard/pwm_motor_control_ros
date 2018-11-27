#include <ros/ros.h>

#include <pwm_control_factory_ros.h>
#include <pwm_servo_control_node.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pwm_servo_control");
    ros::NodeHandle nh;

    RosPwmControlFactory factory(&nh);

    uint8_t pin_output_pwm = 1U;
    uint8_t pin_output_direction_1= 2U;
    uint8_t pin_output_direction_2= 3U;

    auto servo_control = factory.CreateServoControl(pin_output_pwm,
                                                    pin_output_direction_1,
                                                    pin_output_direction_2);

    PwmServoControlNode pwm_servo_control_node(&nh, std::move(servo_control));

    ros::spin();

    return 0;
}