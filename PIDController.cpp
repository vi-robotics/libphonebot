#include "PIDController.hpp"

namespace phonebot {
namespace control {

inline float Clip(const float x, const float min_x, const float max_x) {
    return (x < min_x ? min_x : x > max_x ? max_x : x);
}

void PIDController::Reset() {
    prv_err_ = 0.0;
    net_err_ = 0.0;
}

PIDController::PIDController() {}
PIDController::PIDController(const float k_p, const float k_i, const float k_d,
                             const float max_control, const float max_net_err)
    : k_p_(k_p),
      k_i_(k_i),
      k_d_(k_d),
      max_control_(max_control),
      max_net_err_(max_net_err) {
    Reset();
}

float PIDController::Control(const float error, const float delta_time) {
    if (delta_time <= 0) {
        return 0.0f;
    }

    const float ctrl_p = k_p_ * error;
    const float ctrl_i = k_i_ * net_err_;
    const float ctrl_d = k_d_ * (error - prv_err_) / delta_time;

    // Clamped control output.
    const float output =
        Clip(ctrl_p + ctrl_i + ctrl_d, -max_control_, max_control_);

    // Cache state for next iteration.
    prv_err_ = error;
    net_err_ = Clip(net_err_ + error * delta_time, -max_net_err_, max_net_err_);

    return output;
}
}
}
