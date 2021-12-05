#include "PIDController.hpp"

#include <gtest/gtest.h>
#include <cstdlib>
#include <iostream>
#include <random>

TEST(PIDController, Equilibrium) {
    phonebot::control::PIDController pid(1.0, 0.0, 0.0,
                                         std::numeric_limits<float>::max(),
                                         std::numeric_limits<float>::max());
    EXPECT_EQ(pid.Control(0.0, 1.0), 0);
}

TEST(PIDController, StepResponse) {
    const float k_p = 0.8;
    const float k_i = 0.023;
    const float k_d = 5.0;
    phonebot::control::PIDController pid(k_p, k_i, k_d,
                                         std::numeric_limits<float>::max(),
                                         std::numeric_limits<float>::max());
    float x = 0.0;
    float v = 0.0;
    const float dt = 0.1;
    float err = 0.0;
    std::vector<float> ts, xs;
    for (float t = 0; t < 10.0; t += dt) {
        const float y = (t < 1.0 ? 0.0 : 1.0);
        const float a = pid.Control(y - x, dt);
        err += (y - x) * (y - x);
        x += v * dt;
        v += a * dt;
        ts.emplace_back(t);
        xs.emplace_back(x);
    }
    std::cout << err << std::endl;
    EXPECT_LT(err, 0.25 / dt);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
