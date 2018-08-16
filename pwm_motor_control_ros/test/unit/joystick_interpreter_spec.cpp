#include <gtest/gtest.h>

#include <joystick_interpreter.h>

TEST(JoyStickInterpreter, Initialization)
{
    JoystickInterpreter unit;
    
    EXPECT_EQ(unit.GetAxis(), 0U);
    EXPECT_EQ(unit.GetAxisMaxValue(), 1.0F);
    EXPECT_EQ(unit.GetAxisMinValue(), 0.0F);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}