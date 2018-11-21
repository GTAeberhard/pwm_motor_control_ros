#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/Joy.h>

#include "joystick_interpreter.h"

ros::Publisher motor_speed_pub;

joystick_interpreter::JoystickInterpreter joy_interpreter;

int joy_axis_id_motor_forward;
int joy_axis_id_motor_backward;

void JoyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    if (msg->axes.size() > joy_axis_id_motor_forward
        && joy_interpreter.IsPressed(joy_axis_id_motor_forward,
                                     msg->axes[joy_axis_id_motor_forward]))
    {
        std_msgs::Int8 msg_speed;
        auto output = joy_interpreter.TransformInput(joy_axis_id_motor_forward,
                                                     msg->axes[joy_axis_id_motor_forward]);
        msg_speed.data = static_cast<int>(output*100.0F);
        motor_speed_pub.publish(msg_speed);
    }
    if (msg->axes.size() > joy_axis_id_motor_backward
        && joy_interpreter.IsPressed(joy_axis_id_motor_backward,
                                     msg->axes[joy_axis_id_motor_backward]))
    {
        std_msgs::Int8 msg_speed;
        auto output = joy_interpreter.TransformInput(joy_axis_id_motor_backward,
                                                     msg->axes[joy_axis_id_motor_backward]);
        msg_speed.data = static_cast<int>(output*100.0F);
        motor_speed_pub.publish(msg_speed);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_motor_control");
    ros::NodeHandle nh("~");

    ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 10, JoyCallback);
    motor_speed_pub = nh.advertise<std_msgs::Int8>("/motor_speed", 10);

    joystick_interpreter::Range joy_axis_range_input_motor_forward;
    joystick_interpreter::Range joy_axis_range_output_motor_forward{0.0F, 1.0F};
    
    nh.param("axis_motor_forward", joy_axis_id_motor_forward, 0);
    nh.param("axis_max_motor_forward", joy_axis_range_input_motor_forward.max, 1.0F);
    nh.param("axis_min_motor_forward", joy_axis_range_input_motor_forward.min, 0.0F);

    joy_interpreter.RegisterAxis(joy_axis_id_motor_forward,
                                 joy_axis_range_input_motor_forward,
                                 joy_axis_range_output_motor_forward);

    joystick_interpreter::Range joy_axis_range_input_motor_backward;
    joystick_interpreter::Range joy_axis_range_output_motor_backward{0.0F, -1.0F};

    nh.param("axis_motor_backward", joy_axis_id_motor_backward, 1);
    nh.param("axis_max_motor_backward", joy_axis_range_input_motor_backward.max, 1.0F);
    nh.param("axis_min_motor_backward", joy_axis_range_input_motor_backward.min, 0.0F);

    joy_interpreter.RegisterAxis(joy_axis_id_motor_backward,
                                 joy_axis_range_input_motor_backward,
                                 joy_axis_range_output_motor_backward);

    ros::spin();

    return 0;
}