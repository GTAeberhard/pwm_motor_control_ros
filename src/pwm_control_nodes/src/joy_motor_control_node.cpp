#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "sensor_msgs/Joy.h"


void JoyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_motor_control");

    ros::NodeHandle nh;

    ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, JoyCallback);
    ros::Publisher motor_speed_pub = nh.advertise<std_msgs::Int8>("motor_speed", 10);

    ros::spin();

    return 0;
}