#include "PhonebotKinematics.hpp"

#include <algorithm>
#include <cstdlib>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

/* #include <pair> */

namespace phonebot {
namespace control {

void PhonebotJointKinematicSolver::SolvePassiveAll(
    const float a, const float b, const float c, const float h1, const float h2,
    std::vector<std::pair<float, float>>* sol) {
    // Calculate all the intermediate expressions..
    // TODO(yycho0108): refactor + optimize
    const float x0 = c * c;
    const float x1 = 2 * x0;
    const float x2 = b * b;
    const float x3 = h1 + h2;
    const float x4 = cos(x3);
    const float x5 = x2 * x4;
    const float x6 = cos(h1);
    const float x7 = 2 * a;
    const float x8 = b * x7;
    const float x9 = x6 * x8;
    const float x10 = cos(h2);
    const float x11 = x10 * x8;
    const float x12 = a * a;
    const float x13 = 2 * x12;
    const float x14 = x13 + x2;
    const float x15 = x11 + x14 + x5 + x9;
    const float x16 = sin(h1);
    const float x17 = sin(x3);
    const float x18 = b * x17;
    const float x19 = x16 * x7 + x18;
    const float x20 = x0 * x15 * x19 * x19;
    const float x21 = sqrt(-x20 * (-x1 + x15));
    const float x22 = 2 * x21;
    const float x23 = b * b * b * c;
    const float x24 = b * c;
    const float x25 = x13 * x24 + x23;
    const float x26 = 4 * a;
    const float x27 = x26 * x6;
    const float x28 = 4 * x21;
    const float x29 = b * x28;
    const float x30 = x29 * x4;
    const float x31 = b * b * b * b * c;
    const float x32 = 2 * x31;
    const float x33 = 3 * h1;
    const float x34 = 3 * h2;
    const float x35 = x33 + x34;
    const float x36 = 2 * h1;
    const float x37 = 2 * h2;
    const float x38 = x36 + x37;
    const float x39 = cos(x38);
    const float x40 = -x31 * x4 + x31 * cos(x35) + x32 * x39 - x32;
    const float x41 = -x29 - x30 + x40;
    const float x42 = 16 * c;
    const float x43 = a * a * a * a * x42;
    const float x44 = c * x12;
    const float x45 = x2 * x44;
    const float x46 = 20 * x45;
    const float x47 = a * a * a;
    const float x48 = x24 * x47;
    const float x49 = x10 * x48;
    const float x50 = a * x10;
    const float x51 = x23 * x50;
    const float x52 = cos(x37);
    const float x53 = 8 * x45;
    const float x54 = 8 * x44;
    const float x55 = x5 * x54;
    const float x56 = c * x2;
    const float x57 = -h2;
    const float x58 = h1 + x57;
    const float x59 = cos(x58);
    const float x60 = 4 * x12;
    const float x61 = x59 * x60;
    const float x62 = h1 + x37;
    const float x63 = cos(x62);
    const float x64 = x23 * x7;
    const float x65 = x34 + x36;
    const float x66 = cos(x65);
    const float x67 = x33 + x37;
    const float x68 = cos(x67);
    const float x69 = a * x23;
    const float x70 = 8 * x48;
    const float x71 = x36 + x57;
    const float x72 = x70 * cos(x71);
    const float x73 = h2 + x36;
    const float x74 = cos(x73);
    const float x75 = h2 + x33;
    const float x76 = cos(x75);
    const float x77 = 12 * x45;
    const float x78 = x12 + x2;
    const float x79 = cos(x36);
    const float x80 = x12 * x42;
    const float x81 = x39 * x77 - x43 - x46 + 24 * x48 * x74 - 32 * x49 -
                      12 * x51 - x52 * x53 - x55 - x56 * x61 - x63 * x64 +
                      x64 * x66 + 6 * x68 * x69 + 10 * x69 * x74 +
                      x70 * cos(x33) + x72 + x76 * x77 + x78 * x79 * x80;
    const float x82 = 1 / (x0 * x15);
    const float x83 = 0.25 * x82 / x19;
    const float x84 = 2 * sqrt(x20 * (x1 - x11 - x13 - x2 - x5 - x9));
    const float x85 = b * x54;
    const float x86 = x26 * x74;
    const float x87 = x60 * x79;
    const float x88 = c * x6 * x7 * (5 * x2 + x60) + x23 * x39 + 4 * x23 * x4 +
                      3 * x23 + x24 * x61 + x24 * x87 + x4 * x85 +
                      8 * x50 * x56 + x56 * x63 * x7 + x56 * x86 + x85;
    const float x89 = 0.5 * x82;
    const float x90 = 8 * x21;
    const float x91 = x50 * x90;
    const float x92 = x2 * x80;
    const float x93 = h1 - x37;
    const float x94 = x56 * x60;
    const float x95 = x23 * x26;
    const float x96 = h1 + x34;
    const float x97 = b * x42 * x47;
    const float x98 = a * x24;
    const float x99 = x23 * x86 + x39 * x92 + x4 * x43 - x43 * x59 - 8 * x49 -
                      8 * x51 - x52 * x94 - x53 + x55 - x56 * x87 - x59 * x92 -
                      8 * x6 * x78 * x98 + x63 * x95 + x63 * x97 + x66 * x95 +
                      x68 * x95 - x70 * cos(x93) - x72 + x74 * x97 + x76 * x94 +
                      x94 * cos(x96);
    const float x100 = sin(h2);
    const float x101 = a * x100 * x90;
    const float x102 = x18 * x28;
    const float x103 = sin(x38);
    const float x104 = sin(x58);
    const float x105 = sin(x62);
    const float x106 = sin(x73);
    const float x107 =
        -x100 * x70 - x100 * x95 + 4 * x103 * x31 + x103 * x92 + x104 * x43 +
        24 * x104 * x45 + 8 * x105 * x69 + x105 * x97 + 16 * x106 * x69 +
        x106 * x97 + 20 * x14 * x16 * x98 + 5 * x17 * x31 + x17 * x43 +
        32 * x17 * x45 + x31 * sin(x35) + x46 * sin(x36) + x70 * sin(x71) +
        x70 * sin(x93) - x94 * sin(x37) + x94 * sin(x75) + x94 * sin(x96) +
        x95 * sin(x65) + x95 * sin(x67);
    const float x108 = x29 + x30 + x40;

    // Populate output.
    sol->clear();
    sol->emplace_back(
        atan2(x83 * (-x27 * (x22 + x25) + x41 + x81), x89 * (-x84 + x88)),
        atan2(x83 * (x41 - x91 + x99), x83 * (-x101 - x102 + x107)));
    sol->emplace_back(
        atan2(x83 * (x108 - x27 * (-x22 + x25) + x81), x89 * (x84 + x88)),
        atan2(x83 * (x108 + x91 + x99), x83 * (x101 + x102 + x107)));
}

void PhonebotJointKinematicSolver::SolveActiveAll(
    const float a, const float b, const float c, const float x, const float y,
    std::vector<std::pair<float, float>>* sol) {
    const float x0 = a * a * a;
    const float x1 = 4 * x * x0;
    const float x2 = b * b;
    const float x3 = c * c;
    const float x4 = x * x;
    const float x5 = y * y;
    const float x6 = x4 + x5;
    const float x7 = -x3 + x6;
    const float x8 = a * x;
    const float x9 = 4 * x8;
    const float x10 = a * a;
    const float x11 = 3 * x4;
    const float x12 = x2 + x3 - x5;
    const float x13 = a * a * a * a + b * b * b * b - 2 * x10 * (-x11 + x12) -
                      2 * x2 * (x3 + x6) + x7 * x7;
    const float x14 = x2 * x5;
    const float x15 = sqrt(-x14 * (x1 + x13 + x9 * (-x2 + x7)));
    const float x16 = a * x15;
    const float x17 = x * x15;
    const float x18 = 2 * x8;
    const float x19 = b * x5;
    const float x20 = x18 * x19;
    const float x21 = b * b * b;
    const float x22 =
        b * y * y * y * y + x10 * x19 - x19 * x3 + x19 * x4 + x21 * x5;
    const float x23 = x20 + x22;
    const float x24 = 1 / y;
    const float x25 = 1 / x2;
    const float x26 = x10 + x6;
    const float x27 = x25 / (x18 + x26);
    const float x28 = x24 * x27;
    const float x29 = a * x21;
    const float x30 = b * x0;
    const float x31 = a * x19;
    const float x32 = b * x3;
    const float x33 = a * x32;
    const float x34 = a * b * x11;
    const float x35 = x29 + x30 + x31 - x33 + x34;
    const float x36 = b * x * x * x;
    const float x37 = x * x21;
    const float x38 = x * x19;
    const float x39 = x * x32;
    const float x40 = 3 * b * x * x10;
    const float x41 = x36 + x37 + x38 - x39 + x40;
    const float x42 = x35 + x41;
    const float x43 = atan2(x28 * (-x16 - x17 + x23), -x27 * (x15 + x42));
    const float x44 = sqrt(-x14 * (-x1 + x13 + x9 * (x12 - x4)));
    const float x45 = x * x44;
    const float x46 = a * x44;
    const float x47 = -x20 + x22;
    const float x48 = x25 / (-x18 + x26);
    const float x49 = x24 * x48;
    const float x50 = atan2(x49 * (x45 - x46 + x47),
                            -x48 * (x35 - x36 - x37 - x38 + x39 - x40 + x44));
    const float x51 = atan2(x49 * (-x45 + x46 + x47),
                            x48 * (-x29 - x30 - x31 + x33 - x34 + x41 + x44));
    const float x52 = atan2(x28 * (x16 + x17 + x23), -x27 * (-x15 + x42));

    // Populate output.
    sol->clear();
    sol->emplace_back(x43, x50);
    sol->emplace_back(x43, x51);
    sol->emplace_back(x52, x50);
    sol->emplace_back(x52, x51);
}

PhonebotJointKinematicSolver::PhonebotJointKinematicSolver(
    const PhonebotKinematicConfiguration& config)
    : config_(config) {
    Reset();
}

void PhonebotJointKinematicSolver::Reset() {
    joint_kinematic_state_.active_front = 0.0;
    joint_kinematic_state_.active_back = 0.0;
    joint_kinematic_state_.passive_front = 0.0;
    joint_kinematic_state_.passive_back = 0.0;
    joint_kinematic_state_.endpoint_x = 0.0;
    joint_kinematic_state_.endpoint_y = 0.0;
}

void PhonebotJointKinematicSolver::Reset(const JointKinematicState& state) {
    joint_kinematic_state_ = state;
}

void PhonebotJointKinematicSolver::SolvePassive(const float h1, const float h2,
                                                float* h3, float* h4) {
    std::vector<std::pair<float, float>> sols;
    SolvePassiveAll(config_.femur_length_, config_.tibia_length_,
                    config_.joint_displacement_, h1, h2, &sols);

    float best_err = std::numeric_limits<float>::max();
    for (const std::pair<float, float>& sol : sols) {
        // debug
        const float dh3 =
            NormalizeAngle(joint_kinematic_state_.passive_front - sol.first);
        const float dh4 =
            NormalizeAngle(joint_kinematic_state_.passive_back - sol.second);
        // NOTE: squared error
        const float err = (dh3 * dh3 + dh4 + dh4);

        if (err < best_err) {
            best_err = err;
            *h3 = sol.first;
            *h4 = sol.second;
        }
    }

    // NOT 100% sure on th eresult from Solve...All, validate
    *h3 = NormalizeAngle(*h3);
    *h4 = NormalizeAngle(*h4);
}

void PhonebotJointKinematicSolver::SolveActive(const float ex, const float ey,
                                               float* h1, float* h2) {
    std::vector<std::pair<float, float>> sols;
    SolveActiveAll(config_.femur_length_, config_.tibia_length_,
                   config_.joint_displacement_, ex, ey, &sols);

    float best_err = std::numeric_limits<float>::max();
    for (const std::pair<float, float>& sol : sols) {
        float ex_sol, ey_sol;
        SolveEndpoint(sol.first, sol.second,
                      joint_kinematic_state_.passive_front,
                      joint_kinematic_state_.passive_back, &ex_sol, &ey_sol);
        const float dx = ex - ex_sol;
        const float dy = ey - ey_sol;
        // NOTE: squared error
        const float err = (dx * dx + dy * dy);
        if (err < best_err) {
            best_err = err;
            *h1 = sol.first;
            *h2 = sol.second;
        }
    }
    *h1 = NormalizeAngle(*h1);
    *h2 = NormalizeAngle(*h2);
}

void PhonebotJointKinematicSolver::SolveEndpoint(const float h1, const float h2,
                                                 const float h3, const float h4,
                                                 float* ex, float* ey) {
    const float a = config_.femur_length_;
    const float b = config_.tibia_length_;
    const float c = config_.joint_displacement_;

    const Eigen::Vector2f u_x{1.0, 0.0};
    const Eigen::Rotation2D<float> R1{-h1};
    const Eigen::Rotation2D<float> R2{h2};
    const Eigen::Rotation2D<float> R3{-h3};
    const Eigen::Rotation2D<float> R4{h4};

    // Solve the kinematic chain.
    // TODO(yycho0108): Make this more efficient at some point, maybe.
    const Eigen::Vector2f ee_1 = a * u_x + R2 * (b * u_x - c * (R4 * u_x));
    const Eigen::Vector2f ee_2 = -a * u_x + R1 * (-b * u_x + c * (R3 * u_x));

    // TODO: Implement error checking here when ee_1 != ee_2
    *ex = 0.5 * (ee_1.x() + ee_2.x());
    *ey = 0.5 * (ee_1.y() + ee_2.y());
}
}
}
