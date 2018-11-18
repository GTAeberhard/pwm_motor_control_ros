#ifndef JOYSTICK_INTERPRETER_H
#define JOYSTICK_INTERPRETER_H

#include <vector>

class JoystickInterpreter
{
public:
    JoystickInterpreter();

    float TransformInput(float input_value);

    size_t GetAxis() const;
    
    void SetAxisOutputRange(const float min_value, const float max_value);
    void SetAxisInputRange(const float min_value, const float max_value);

    float GetAxisOutputRangeMax() const;
    float GetAxisOutputRangeMin() const;

    float GetAxisInputRangeMax() const;
    float GetAxisInputRangeMin() const;
 
private:
    void CalculateSlopeParamter();

    size_t axis_{0U};
    float output_max_value_{1.0F};
    float output_min_value_{0.0F};
    float input_max_value_{1.0F};
    float input_min_value_{0.0F};

    float param_slope_;
};

#endif