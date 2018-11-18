#include <gtest/gtest.h>

#include "joystick_interpreter.h"

TEST(JoyStickInterpreter, Initialization)
{
    JoystickInterpreter unit;
    
    EXPECT_EQ(unit.GetAxis(), 0U);

    EXPECT_EQ(unit.GetAxisOutputRangeMax(), 1.0F);
    EXPECT_EQ(unit.GetAxisOutputRangeMin(), 0.0F);

    EXPECT_EQ(unit.GetAxisInputRangeMax(), 1.0F);
    EXPECT_EQ(unit.GetAxisInputRangeMin(), 0.0F);
}

TEST(JoystickInterpreter, SetOutputInputMinMax)
{
    JoystickInterpreter unit;

    constexpr float output_min = 0.5F;
    constexpr float output_max = 2.0F;
    constexpr float input_min = -1.0F;
    constexpr float input_max = 1.0F;

    unit.SetAxisOutputRange(output_min, output_max);
    unit.SetAxisInputRange(input_min, input_max);

    EXPECT_EQ(unit.GetAxisOutputRangeMin(), output_min);
    EXPECT_EQ(unit.GetAxisOutputRangeMax(), output_max);
    EXPECT_EQ(unit.GetAxisInputRangeMin(), input_min);
    EXPECT_EQ(unit.GetAxisInputRangeMax(), input_max);
}

class TriggerButton : public testing::Test
{
  protected:
    void SetUp() override
    {
        constexpr float output_min = 0.0F;
        constexpr float output_max = 1.0F;
        constexpr float input_min = -1.0F;
        constexpr float input_max = 1.0F;

        unit_.SetAxisOutputRange(output_min, output_max);
        unit_.SetAxisInputRange(input_min, input_max);
    }

    JoystickInterpreter unit_;
};

TEST_F(TriggerButton, Trigger_FullyDepressed)
{
    float output = unit_.TransformInput(1.0F);
    EXPECT_EQ(output, 1.0F);
}

TEST_F(TriggerButton, Trigger_NotPressed)
{
    float output = unit_.TransformInput(-1.0F);
    EXPECT_EQ(output, 0.0F);
}

TEST_F(TriggerButton, Trigger_HalfDepressed)
{
    float output = unit_.TransformInput(0.0F);
    EXPECT_EQ(output, 0.5F);
}

TEST_F(TriggerButton, Trigger_OutOfRangeHigh)
{
    float output = unit_.TransformInput(1.5F);
    EXPECT_EQ(output, 1.0F);
}

TEST_F(TriggerButton, Trigger_OutOfRangeLow)
{
    float output = unit_.TransformInput(-1.5F);
    EXPECT_EQ(output, 0.0F);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}