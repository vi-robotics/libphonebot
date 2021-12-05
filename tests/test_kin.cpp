#include "PhonebotKinematics.hpp"

#include <gtest/gtest.h>
#include <cstdlib>
#include <iostream>
#include <random>

TEST(PhonebotJointKinematicSolver, ZeroState) {
    const float a = 0.0110;
    const float b = 0.0175;
    const float c = 0.0285;

    float h1 = 1.40429062;
    float h2 = 1.40429062;
    float h3 = -2.465595456021317;
    float h4 = -2.465595418739502;

    // endpoints are approximately @ 0
    float ex = 4.6566129e-10;
    float ey = 0.042138;

    phonebot::control::PhonebotKinematicConfiguration config{0.2, 0.2, 0.2,
                                                             a,   b,   c};
    phonebot::control::JointKinematicState state{h1, h2, h3, h4, ex, ey};
    phonebot::control::PhonebotJointKinematicSolver solver(config);

    solver.Reset(state);

    float h1e, h2e, h3e, h4e, exe, eye;

    solver.SolveActive(ex, ey, &h1e, &h2e);
    solver.SolvePassive(h1, h2, &h3e, &h4e);
    solver.SolveEndpoint(h1, h2, h3, h4, &exe, &eye);

    printf("%f,%f,%f,%f,%f,%f\n", h1, h2, h3, h4, ex, ey);
    printf("%f,%f,%f,%f,%f,%f\n", h1e, h2e, h3e, h4e, exe, eye);

    EXPECT_NEAR(phonebot::control::NormalizeAngle(h1 - h1e), 0.0, 1e-6);
    EXPECT_NEAR(phonebot::control::NormalizeAngle(h2 - h2e), 0.0, 1e-6);
    EXPECT_NEAR(phonebot::control::NormalizeAngle(h3 - h3e), 0.0, 1e-6);
    EXPECT_NEAR(phonebot::control::NormalizeAngle(h4 - h4e), 0.0, 1e-6);
    EXPECT_NEAR(ex, exe, 1e-6);
    EXPECT_NEAR(ey, eye, 1e-6);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
