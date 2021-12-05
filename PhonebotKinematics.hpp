#pragma once

#include <cmath>
#include <memory>
#include <vector>

namespace phonebot {
namespace control {

inline float Sign(const float x) { return (0 < x) - (x < 0); }
inline float NormalizeAngle(const float x) {
    return Sign(x) * (fmod(fabs(x) + M_PI, M_PI+M_PI) - M_PI);
}

/**
 * General phonebot kinematic configuration.
 */
struct PhonebotKinematicConfiguration {
    // Cuboid-like phone definition
    // NOTE: collision volume?
    float body_width_;
    float body_length_;
    float body_height_;

    // leg linkage length closer to body
    float femur_length_;
    // leg linkage length closer to foot
    float tibia_length_;
    // horizontal displacement between joint origins
    float joint_displacement_;
};

struct JointKinematicState {
    // Joint states
    float active_front;
    float active_back;
    float passive_front;
    float passive_back;

    // Endpoint states
    float endpoint_x;
    float endpoint_y;
};

/**
 * Phonebot Joint Kinematic Solver.
 *
 * NOTE on linkage name assignments :
 *   --   >> c
 *  /  \  >> a
 *  \  /
 *   \/   >> b
 *
 */
class PhonebotJointKinematicSolver {
   private:
    // Utility functions to return all possible candidates (with symmetry)
    static void SolvePassiveAll(const float a, const float b, const float c,
                                const float h1, const float h2,
                                std::vector<std::pair<float, float>>* sol);
    static void SolveActiveAll(const float a, const float b, const float c,
                               const float x, const float y,
                               std::vector<std::pair<float, float>>* sol);

   public:
    PhonebotJointKinematicSolver(const PhonebotKinematicConfiguration& config);
    // Stateful kinematic solutions.
    // Passive joint positions from active joint positions
    void SolvePassive(const float h1, const float h2, float* h3, float* h4);
    // Active joint positions from endpoints
    void SolveActive(const float ex, const float ey, float* h1, float* h2);
    void SolveEndpoint(const float h1, const float h2, const float h3,
                       const float h4, float* ex, float* ey);

    void Reset();
    void Reset(const JointKinematicState& state);

   private:
    PhonebotKinematicConfiguration config_;
    JointKinematicState joint_kinematic_state_;

    // Maximum inscribed rectange ...
    // const Vector2f max_workspace_rect_[4];

    // Full bounding polygon ...
    // const std::vector<Vector2f> workspace_;
};

/**
 * Phonebot Body Kinematic Solver.
 *
 * TODO(yycho0108): implement this
 *
 * NOTE: coordinate system is defined as follows:
 * +x : forward
 * +y : left
 * +z : up
 *
 */

/*
class PhonebotBodyKinematicSolver {
    enum { FRONT_LEFT, FRONT_RIGHT, HIND_LEFT, HIND_RIGHT };
    PhonebotJointKinematicSolver joint_solvers[4];
};
*/
}
}
