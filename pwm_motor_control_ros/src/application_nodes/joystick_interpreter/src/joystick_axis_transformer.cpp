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

float JoystickAxisTransformer::TransformInput(const float input_value) const
{
    auto output_value = param_slope_ * (input_value - input_range_.min) + output_range_.min;

    if (GetAxisOutputRange().max > GetAxisOutputRange().min)
    {
        output_value = std::min(output_value, GetAxisOutputRange().max);
        output_value = std::max(output_value, GetAxisOutputRange().min);
    }
    else
    {
        output_value = std::min(output_value, GetAxisOutputRange().min);
        output_value = std::max(output_value, GetAxisOutputRange().max);
    }

    return output_value;
}

bool JoystickAxisTransformer::IsPressed(const float input_value) const
{
    auto range = input_range_.max - input_range_.min;
    auto amount_above_min = input_value - input_range_.min;

    if (GetAxisInputRange().max > GetAxisInputRange().min)
    {
        amount_above_min = std::min(amount_above_min, GetAxisInputRange().max);
        amount_above_min = std::max(amount_above_min, GetAxisInputRange().min);
    }
    else
    {
        amount_above_min = std::min(amount_above_min, GetAxisInputRange().min);
        amount_above_min = std::max(amount_above_min, GetAxisInputRange().max);
    }

    auto percent_of_range = amount_above_min / range;

    return (percent_of_range > THESHOLD_PRESSED) ? true : false;
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
