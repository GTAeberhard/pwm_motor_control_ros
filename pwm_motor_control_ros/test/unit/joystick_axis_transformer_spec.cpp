#include <gtest/gtest.h>

#include "joystick_axis_transformer.h"

using namespace joystick_interpreter;

TEST(JoystickAxisTransformer, Initialization)
{
    JoystickAxisTransformer unit;

    EXPECT_EQ(unit.GetAxisOutputRange().max, 1.0F);
    EXPECT_EQ(unit.GetAxisOutputRange().min, 0.0F);

    EXPECT_EQ(unit.GetAxisInputRange().max, 1.0F);
    EXPECT_EQ(unit.GetAxisInputRange().min, 0.0F);
}

TEST(JoystickAxisTransformer, SetOutputInputMinMax)
{
    JoystickAxisTransformer unit;

    constexpr float output_min = 0.5F;
    constexpr float output_max = 2.0F;
    constexpr float input_min = -1.0F;
    constexpr float input_max = 1.0F;

    unit.SetAxisOutputRange(output_min, output_max);
    unit.SetAxisInputRange(input_min, input_max);

    EXPECT_EQ(unit.GetAxisOutputRange().min, output_min);
    EXPECT_EQ(unit.GetAxisOutputRange().max, output_max);
    EXPECT_EQ(unit.GetAxisInputRange().min, input_min);
    EXPECT_EQ(unit.GetAxisInputRange().max, input_max);
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

    JoystickAxisTransformer unit_;
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