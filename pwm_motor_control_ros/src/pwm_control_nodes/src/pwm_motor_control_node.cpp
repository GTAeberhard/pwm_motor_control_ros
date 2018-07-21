#include <pwm_motor_control_node.h>

PwmMotorControlNode::PwmMotorControlNode(ros::NodeHandle* node_handle,
                                         std::unique_ptr<PwmMotorControl> motor_control)
    : nh_(*node_handle), motor_control_(std::move(motor_control))
{
    motor_speed_sub_ = nh_.subscribe<std_msgs::Int8>("motor_speed",
                                                     10,
                                                     &PwmMotorControlNode::MotorSpeedCallback,
                                                     this);
    output_timer_ = nh_.createTimer(pwm_output_period_,
                                    &PwmMotorControlNode::OutputTimerCallback,
                                    this);
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
