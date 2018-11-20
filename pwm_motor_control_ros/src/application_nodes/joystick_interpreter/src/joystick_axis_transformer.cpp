#include "joystick_axis_transformer.h"

using namespace joystick_interpreter;

JoystickAxisTransformer::JoystickAxisTransformer()
{
    CalculateSlopeParamter();
}

JoystickAxisTransformer::JoystickAxisTransformer(const Range input_range,
                                                 const Range output_range)
    : input_range_(input_range), output_range_(output_range) 
{
    CalculateSlopeParamter();
}

float JoystickAxisTransformer::TransformInput(float input_value)
{
    input_value = std::min(input_value, GetAxisInputRange().max);
    input_value = std::max(input_value, GetAxisInputRange().min);

    float output_value = param_slope_ * (input_value - input_range_.min) + output_range_.min;

    return output_value;
}

void JoystickAxisTransformer::CalculateSlopeParamter()
{
    param_slope_ = (GetAxisOutputRange().max - GetAxisOutputRange().min) / 
                   (GetAxisInputRange().max - GetAxisInputRange().min);
}

void JoystickAxisTransformer::SetAxisOutputRange(const float min_value, const float max_value)
{
    output_range_ = {min_value, max_value};
    CalculateSlopeParamter();
}

void JoystickAxisTransformer::SetAxisInputRange(const float min_value, const float max_value)
{
    input_range_ = {min_value, max_value};
    CalculateSlopeParamter();
}

const Range JoystickAxisTransformer::GetAxisOutputRange() const
{
    return output_range_;
}

const Range JoystickAxisTransformer::GetAxisInputRange() const
{
    return input_range_;
}
