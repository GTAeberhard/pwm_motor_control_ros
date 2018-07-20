#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "sensor_msgs/Joy.h"

ros::Publisher motor_speed_pub;

void JoyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    std_msgs::Int8 msg_speed;
    msg_speed.data = static_cast<int>(msg->axes[0]*100.0F);
    motor_speed_pub.publish(msg_speed);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_motor_control");

    ros::NodeHandle nh;

    ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, JoyCallback);
    motor_speed_pub = nh.advertise<std_msgs::Int8>("motor_speed", 10);

    ros::spin();

    return 0;
}