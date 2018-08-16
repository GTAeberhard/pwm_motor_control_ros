#ifndef JOYSTICK_INTERPRETER_H
#define JOYSTICK_INTERPRETER_H

#include <vector>

class JoystickInterpreter
{
public:
    JoystickInterpreter();

    size_t GetAxis() const
    {
        return axis_;
    }

    float GetAxisMaxValue() const
    {
        return max_value_;
    }

    float GetAxisMinValue() const
    {
        return min_value_;
    }

private:
    size_t axis_{0U};
    float max_value_{1.0F};
    float min_value_{0.0F};
};

#endif