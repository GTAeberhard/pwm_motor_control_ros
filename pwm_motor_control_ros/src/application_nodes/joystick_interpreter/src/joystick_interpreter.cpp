#include "joystick_interpreter.h"

using namespace joystick_interpreter;

JoystickInterpreter::JoystickInterpreter()
{
}

const size_t JoystickInterpreter::GetNumberOfAxis() const
{
    return axis_map_.size();
}

void JoystickInterpreter::RegisterAxis(const uint32_t id,
                                       Range input_range,
                                       Range output_range)
{
    JoystickAxisTransformer axis_transformer(input_range, output_range);
    axis_map_.insert( std::pair<uint32_t, JoystickAxisTransformer>(id, axis_transformer) );
}

const Range JoystickInterpreter::GetAxisInputRange(const uint32_t id)
{
    return axis_map_[id].GetAxisInputRange();
}

const Range JoystickInterpreter::GetAxisOutputRange(const uint32_t id)
{
    return axis_map_[id].GetAxisOutputRange();
}