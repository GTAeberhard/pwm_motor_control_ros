#include "pwm_motor_control_node.h"

PwmMotorControlNode::PwmMotorControlNode(ros::NodeHandle* node_handle)
    : nh_(*node_handle)
{
    motor_speed_sub_ = nh_.subscribe<std_msgs::Int8>("motor_speed",
                                                     10,
                                                     &PwmMotorControlNode::MotorSpeedCallback,
                                                     this);
    output_timer_ = nh_.createTimer(pwm_output_period_,
                                    &PwmMotorControlNode::OutputTimerCallback,
                                    this);

    uint8_t pin_output_pwm = 1U;
    uint8_t pin_output_direction_1= 2U;
    uint8_t pin_output_direction_2= 3U;

    auto output_pwm = std::make_unique<RosGpioPwm>(pin_output_pwm,
                                                   nh_,
                                                   "pwm/out");
    auto output_direction_1 = std::make_unique<RosGpio>(pin_output_direction_1,
                                                        nh_,
                                                        "pwm/direction_1");
    auto output_direction_2 = std::make_unique<RosGpio>(pin_output_direction_2,
                                                        nh_,
                                                        "pwm/direction_2");

    motor_control_ = std::make_unique<PwmMotorControl>(std::move(output_pwm),
                                                       std::move(output_direction_1),
                                                       std::move(output_direction_2));
}

PwmMotorControlNode::~PwmMotorControlNode()
{
}

void PwmMotorControlNode::MotorSpeedCallback(const std_msgs::Int8::ConstPtr& msg)
{
    SaveMotorSpeed(msg->data);
    motor_control_->SetSpeed(current_motor_speed_);
}

void PwmMotorControlNode::SaveMotorSpeed(const int& motor_speed)
{
    current_motor_speed_ = motor_speed;
    time_last_motor_speed_received_ = ros::Time::now();
}

void PwmMotorControlNode::OutputTimerCallback(const ros::TimerEvent& event)
{
    ResetMotorSpeedAfterDataSilence();
    motor_control_->SetSpeed(current_motor_speed_);
}

void PwmMotorControlNode::ResetMotorSpeedAfterDataSilence()
{
    ros::Duration time_since_last_motor_speed = ros::Time::now() - time_last_motor_speed_received_;
    if (time_since_last_motor_speed.toSec() > motor_speed_silence_timout_.toSec())
    {
        current_motor_speed_ = 0;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pwm_motor_control");
    ros::NodeHandle nh;

    PwmMotorControlNode pwm_motor_control_node(&nh);

    ros::spin();

    return 0;
}