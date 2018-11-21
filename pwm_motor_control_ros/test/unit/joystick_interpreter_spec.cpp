#include <gtest/gtest.h>

#include "joystick_interpreter.h"

using namespace joystick_interpreter;

TEST(JoystickInterpreter, Initialization)
{
    JoystickInterpreter unit;

    EXPECT_EQ(unit.GetNumberOfAxis(), 0U);
}

TEST(JoystickInterpreter, RegisterAnAxis)
{
    JoystickInterpreter unit;

    constexpr uint32_t AXIS_ID = 1U;
    Range input_range = {-1.0F, 1.0F};
    Range output_range = {0.0F, 1.0F};
    unit.RegisterAxis(AXIS_ID, input_range, output_range);

    EXPECT_EQ(unit.GetNumberOfAxis(), 1U);

    EXPECT_EQ(unit.GetAxisInputRange(AXIS_ID).min, input_range.min);
    EXPECT_EQ(unit.GetAxisInputRange(AXIS_ID).max, input_range.max);
    EXPECT_EQ(unit.GetAxisOutputRange(AXIS_ID).min, output_range.min);
    EXPECT_EQ(unit.GetAxisOutputRange(AXIS_ID).max, output_range.max);
}

TEST(JoystickInterpreter, RegisterTwoAxis)
{
    JoystickInterpreter unit;

    constexpr uint32_t AXIS_1_ID = 1U;
    Range input_range_1 = {-1.0F, 1.0F};
    Range output_range_1 = {0.0F, 1.0F};
    unit.RegisterAxis(AXIS_1_ID, input_range_1, output_range_1);

    constexpr uint32_t AXIS_2_ID = 5U;
    Range input_range_2 = {-1.0F, 1.0F};
    Range output_range_2 = {-1.0F, 0.0F};
    unit.RegisterAxis(AXIS_2_ID, input_range_2, output_range_2);

    EXPECT_EQ(unit.GetNumberOfAxis(), 2U);

    EXPECT_EQ(unit.GetAxisInputRange(AXIS_1_ID).min, input_range_1.min);
    EXPECT_EQ(unit.GetAxisInputRange(AXIS_1_ID).max, input_range_1.max);
    EXPECT_EQ(unit.GetAxisOutputRange(AXIS_1_ID).min, output_range_1.min);
    EXPECT_EQ(unit.GetAxisOutputRange(AXIS_1_ID).max, output_range_1.max);

    EXPECT_EQ(unit.GetAxisInputRange(AXIS_2_ID).min, input_range_2.min);
    EXPECT_EQ(unit.GetAxisInputRange(AXIS_2_ID).max, input_range_2.max);
    EXPECT_EQ(unit.GetAxisOutputRange(AXIS_2_ID).min, output_range_2.min);
    EXPECT_EQ(unit.GetAxisOutputRange(AXIS_2_ID).max, output_range_2.max);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}