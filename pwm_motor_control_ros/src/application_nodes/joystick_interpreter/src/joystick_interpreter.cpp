#include "joystick_interpreter.h"

JoystickInterpreter::JoystickInterpreter()
{
    CalculateSlopeParamter();
}

float JoystickInterpreter::TransformInput(float input_value)
{
    input_value = std::min(input_value, GetAxisInputRangeMax());
    input_value = std::max(input_value, GetAxisInputRangeMin());

    float output_value = param_slope_ * (input_value - input_min_value_) + output_min_value_;

    return output_value;
}

void JoystickInterpreter::CalculateSlopeParamter()
{
    param_slope_ = (GetAxisOutputRangeMax() - GetAxisOutputRangeMin()) / 
                   (GetAxisInputRangeMax() - GetAxisInputRangeMin());
}

size_t JoystickInterpreter::GetAxis() const
{
    return axis_;
}

void JoystickInterpreter::SetAxisOutputRange(const float min_value, const float max_value)
{
    output_max_value_ = max_value;
    output_min_value_ = min_value;
    CalculateSlopeParamter();
}

void JoystickInterpreter::SetAxisInputRange(const float min_value, const float max_value)
{
    input_max_value_ = max_value;
    input_min_value_ = min_value;
    CalculateSlopeParamter();
}

float JoystickInterpreter::GetAxisOutputRangeMax() const
{
    return output_max_value_;
}

float JoystickInterpreter::GetAxisOutputRangeMin() const
{
    return output_min_value_;
}

float JoystickInterpreter::GetAxisInputRangeMax() const
{
    return input_max_value_;
}

float JoystickInterpreter::GetAxisInputRangeMin() const
{
    return input_min_value_;
}