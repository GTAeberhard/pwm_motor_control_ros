#ifndef JOYSTICK_INTERPRETER_H
#define JOYSTICK_INTERPRETER_H

#include <map>
#include <cstddef>

#include "joystick_axis_transformer.h"

namespace joystick_interpreter {

class JoystickInterpreter
{
public:
    JoystickInterpreter();

    void RegisterAxis(const uint32_t id, Range input_range, Range output_range);
    const Range GetAxisInputRange(const uint32_t id);
    const Range GetAxisOutputRange(const uint32_t id);

    const size_t GetNumberOfAxis() const;
private:
    std::map<uint16_t, JoystickAxisTransformer> axis_map_;
};

}

#endif
