#ifndef ROS_PWM_CONTROL_FACTORY_H
#define ROS_PWM_CONTROL_FACTORY_H

#include <pwm_control_factory.h>
#include <gpio_pwm_ros_stub.h>
#include <gpio_ros_stub.h>

class RosPwmControlFactory : public PwmControlFactory
{
public:
    RosPwmControlFactory(ros::NodeHandle* node_handle)
        : nh_(*node_handle)
    {}

    std::unique_ptr<PwmMotorControl>
    CreateMotorControl(const uint8_t pin_pwm,
                       const uint8_t pin_direction_1,
                       const uint8_t pin_direction_2) override
    {
        return CreatePwmControl<PwmMotorControl>(pin_pwm,
                                                 pin_direction_1,
                                                 pin_direction_2);
    }

    std::unique_ptr<PwmServoControl>
    CreateServoControl(const uint8_t pin_pwm,
                       const uint8_t pin_direction_1,
                       const uint8_t pin_direction_2) override
    {
        return CreatePwmControl<PwmServoControl>(pin_pwm,
                                                 pin_direction_1,
                                                 pin_direction_2);
    }

private:
    template<typename T>
    std::unique_ptr<T> CreatePwmControl(const uint8_t pin_pwm,
                                        const uint8_t pin_direction_1,
                                        const uint8_t pin_direction_2)
    {
        auto output_pwm = std::make_unique<RosGpioPwm>(pin_pwm,
                                                       nh_,
                                                       "pwm/out");
        auto output_direction_1 = std::make_unique<RosGpio>(pin_direction_1,
                                                            nh_,
                                                            "pwm/direction_1");
        auto output_direction_2 = std::make_unique<RosGpio>(pin_direction_2,
                                                            nh_,
                                                            "pwm/direction_2");

        return std::make_unique<T>(std::move(output_pwm),
                                   std::move(output_direction_1),
                                   std::move(output_direction_2));
    }

    ros::NodeHandle nh_;
};

#endif