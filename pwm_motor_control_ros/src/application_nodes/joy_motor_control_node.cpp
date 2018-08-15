#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/Joy.h>

ros::Publisher motor_speed_pub;

int joy_axis_motor_forward;
float joy_axis_max_motor_forward;
float joy_axis_min_motor_forward;

int joy_axis_motor_backward;
float joy_axis_max_motor_backward;
float joy_axis_min_motor_backward;

bool IsJoyAxisDepressed(const float axis_value,
                        const float axis_max,
                        const float axis_min)
{
        float range = axis_max - axis_min;
        float amount_above_min = axis_value - axis_min;
        float percent_of_range = amount_above_min / range;
        if (percent_of_range > 0.0F)
        {
            return true;
        }
        else
        {
            return false;
        }
}

void JoyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    if (msg->axes.size() > joy_axis_motor_forward
        && IsJoyAxisDepressed(msg->axes[joy_axis_motor_forward],
                              joy_axis_max_motor_forward,
                              joy_axis_min_motor_forward))
    {
        std_msgs::Int8 msg_speed;
        float range = joy_axis_max_motor_forward - joy_axis_min_motor_forward;
        float amount_above_min = msg->axes[joy_axis_motor_forward] - joy_axis_min_motor_forward;
        float percent_of_range = amount_above_min / range;
        msg_speed.data = static_cast<int>(percent_of_range*100.0F);
        motor_speed_pub.publish(msg_speed);
    }
    if (msg->axes.size() > joy_axis_motor_backward
        && IsJoyAxisDepressed(msg->axes[joy_axis_motor_backward],
                              joy_axis_max_motor_backward,
                              joy_axis_min_motor_backward))
    {
        std_msgs::Int8 msg_speed;
        float range = joy_axis_max_motor_backward - joy_axis_min_motor_backward;
        float amount_above_min = msg->axes[joy_axis_motor_backward] - joy_axis_min_motor_backward;
        float percent_of_range = amount_above_min / range;
        msg_speed.data = -1 * static_cast<int>(percent_of_range*100.0F);
        motor_speed_pub.publish(msg_speed);
    }
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_motor_control");
    ros::NodeHandle nh("~");

    ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 10, JoyCallback);
    motor_speed_pub = nh.advertise<std_msgs::Int8>("/motor_speed", 10);

    nh.param("axis_motor_forward", joy_axis_motor_forward, 0);
    nh.param("axis_max_motor_forward", joy_axis_max_motor_forward, 1.0F);
    nh.param("axis_min_motor_forward", joy_axis_min_motor_forward, 0.0F);

    nh.param("axis_motor_backward", joy_axis_motor_backward, 1);
    nh.param("axis_max_motor_backward", joy_axis_max_motor_backward, 1.0F);
    nh.param("axis_min_motor_backward", joy_axis_min_motor_backward, 0.0F);

    ros::spin();

    return 0;
}