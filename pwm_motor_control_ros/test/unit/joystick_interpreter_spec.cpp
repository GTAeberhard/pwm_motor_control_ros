#include <gtest/gtest.h>

#include "joystick_interpreter.h"

using namespace joystick_interpreter;

TEST(JoystickInterpreter, DefaultInitialization)
{
    JoystickInterpreter unit;

    EXPECT_EQ(unit.GetNumberOfAxis(), 0U);
}

class JoystickInterpreterSingleAxis : public testing::Test
{
  protected:
    void SetUp() override
    {
        unit_.RegisterAxis(axis_id_, input_range_, output_range_);
    }

    JoystickInterpreter unit_;
    uint32_t axis_id_{2U};
    Range input_range_{-1.0F, 1.0F};
    Range output_range_{0.0F, 1.0F};
};

TEST_F(JoystickInterpreterSingleAxis, AxisRegistered)
{
    EXPECT_EQ(unit_.GetNumberOfAxis(), 1U);

    EXPECT_EQ(unit_.GetAxisInputRange(axis_id_).min, input_range_.min);
    EXPECT_EQ(unit_.GetAxisInputRange(axis_id_).max, input_range_.max);
    EXPECT_EQ(unit_.GetAxisOutputRange(axis_id_).min, output_range_.min);
    EXPECT_EQ(unit_.GetAxisOutputRange(axis_id_).max, output_range_.max);
}

TEST_F(JoystickInterpreterSingleAxis, TransformInput)
{
    float input_value = 0.0F;
    auto output = unit_.TransformInput(axis_id_, 0.0F);

    EXPECT_EQ(output, 0.5F);
}

TEST_F(JoystickInterpreterSingleAxis, IsPressed)
{
    float input_value = 0.5F;
    auto is_pressed = unit_.IsPressed(axis_id_, input_value);

    EXPECT_EQ(is_pressed, true);
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