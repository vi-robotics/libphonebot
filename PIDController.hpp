#pragma once

namespace phonebot {
namespace control {

class PIDController {
   public:
    /* Processing Logic */
    void Reset();
    float Control(const float error, const float delta_time);

    /* Manipulate parameters */
    inline void SetKp(const float k_p) { k_p_ = k_p; }
    inline void SetKi(const float k_i) { k_i_ = k_i; }
    inline void SetKd(const float k_d) { k_d_ = k_d; }

    inline float GetKp() { return k_p_; }
    inline float GetKi() { return k_i_; }
    inline float GetKd() { return k_d_; }

    PIDController();
    PIDController(const float k_p, const float k_i, const float k_d,
                  const float max_control, const float max_net_err);

   private:
    // PID constants
    float k_p_, k_i_, k_d_;

    // PID state
    float max_control_;
    float max_net_err_;
    float prv_err_;
    float net_err_;
};
}
}
