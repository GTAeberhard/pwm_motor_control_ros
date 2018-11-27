#ifndef PWM_MOTOR_CONTROL_NODE_H
#define PWM_MOTOR_CONTROL_NODE_H

#include <ros/ros.h>

#include <std_msgs/Int8.h>

#include <servo_control.h>

class PwmServoControlNode
{
public:
    PwmServoControlNode() = delete;
    PwmServoControlNode(ros::NodeHandle* node_handle,
                        std::unique_ptr<PwmServoControl> servo_control);
    ~PwmServoControlNode();
 
private:
    void ServoAngleCallback(const std_msgs::Int8::ConstPtr& msg);
    // void OutputTimerCallback(const ros::TimerEvent& event);
    // void SaveMotorSpeed(const int& motor_speed);
    // void ResetMotorSpeedAfterDataSilence();

    std::unique_ptr<PwmServoControl> servo_control_;
    // std::int8_t current_motor_speed_{0};
    // ros::Time time_last_motor_speed_received_{0.0F};

    ros::NodeHandle nh_;
    ros::Subscriber servo_angle_sub_;
    // ros::Timer output_timer_;

    // const ros::Duration motor_speed_silence_timout_{1.0F};
    // const ros::Duration pwm_output_period_{0.1F};
};

#endif