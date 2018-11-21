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

struct TestInputOutput
{
    float input;
    float expected_output;
    bool is_output_positive;
};

class TriggerButton : public testing::TestWithParam<TestInputOutput>
{
  protected:
    void SetUp() override
    {
        constexpr float input_min = -1.0F;
        constexpr float input_max = 1.0F;
        unit_.SetAxisInputRange(input_min, input_max);

        constexpr float output_min = 0.0F;
        float output_max;
        if (GetParam().is_output_positive)
        {
            output_max = 1.0F;
        }
        else
        {
            output_max = -1.0F;
        }
        unit_.SetAxisOutputRange(output_min, output_max);
    }

    JoystickAxisTransformer unit_;
};

TEST_P(TriggerButton, ButtonsPressed)
{
    float output = unit_.TransformInput(GetParam().input);
    EXPECT_EQ(output, GetParam().expected_output);
}

INSTANTIATE_TEST_CASE_P(TriggerButtonPressesNormalWithPositiveOutput,
                        TriggerButton,
                        testing::Values(TestInputOutput{1.0F, 1.0F, true},
                                        TestInputOutput{-1.0F, 0.0F, true},
                                        TestInputOutput{0.0F, 0.5F, true}));

INSTANTIATE_TEST_CASE_P(TriggerButtonPressesOutOfRangeWithPositiveOutput,
                        TriggerButton,
                        testing::Values(TestInputOutput{1.5F, 1.0F, true},
                                        TestInputOutput{-1.5F, 0.0F, true}));

INSTANTIATE_TEST_CASE_P(TriggerButtonPressesNormalWithNegativeOutput,
                        TriggerButton,
                        testing::Values(TestInputOutput{1.0F, -1.0F, false},
                                        TestInputOutput{-1.0F, 0.0F, false},
                                        TestInputOutput{0.0F, -0.5F, false}));

INSTANTIATE_TEST_CASE_P(TriggerButtonPressesOutOfRangeWithNegativeOutput,
                        TriggerButton,
                        testing::Values(TestInputOutput{1.5F, -1.0F, false},
                                        TestInputOutput{-1.5F, 0.0F, false}));

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}