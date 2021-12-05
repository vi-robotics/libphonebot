#pragma once

#include "PIDController.hpp"

namespace phonebot {
namespace control {

class JointTrajectoryController {
   public:
    JointTrajectoryController();

   private:
    PIDController pid_;
};
}
}
