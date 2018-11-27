#include <pwm_servo_control_node.h>

PwmServoControlNode::PwmServoControlNode(ros::NodeHandle* node_handle,
                                         std::unique_ptr<PwmServoControl> servo_control)
    : nh_(*node_handle), servo_control_(std::move(servo_control))
{
    servo_angle_sub_ = nh_.subscribe<std_msgs::Int8>("servo_angle",
                                                     10,
                                                     &PwmServoControlNode::ServoAngleCallback,
                                                     this);
    // output_timer_ = nh_.createTimer(pwm_output_period_,
    //                                 &PwmMotorControlNode::OutputTimerCallback,
    //                                 this);
}

PwmServoControlNode::~PwmServoControlNode()
{
}

void PwmServoControlNode::ServoAngleCallback(const std_msgs::Int8::ConstPtr& msg)
{
    // SaveMotorSpeed(msg->data);
    servo_control_->SetAngle(msg->data);
}

// void PwmMotorControlNode::SaveMotorSpeed(const int& motor_speed)
// {
//     current_motor_speed_ = motor_speed;
//     time_last_motor_speed_received_ = ros::Time::now();
// }

// void PwmMotorControlNode::OutputTimerCallback(const ros::TimerEvent& event)
// {
//     ResetMotorSpeedAfterDataSilence();
//     motor_control_->SetSpeed(current_motor_speed_);
// }

// void PwmMotorControlNode::ResetMotorSpeedAfterDataSilence()
// {
//     ros::Duration time_since_last_motor_speed = ros::Time::now() - time_last_motor_speed_received_;
//     if (time_since_last_motor_speed.toSec() > motor_speed_silence_timout_.toSec())
//     {
//         current_motor_speed_ = 0;
//     }
// }
