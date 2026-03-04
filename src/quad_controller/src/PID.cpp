#include "quad_controller/PID.hpp"
#include <algorithm>

PID::PID(float kp, float ki, float kd, float min_out, float max_out)
    : kp_(kp), ki_(ki), kd_(kd), min_(min_out), max_(max_out),
      setpoint_(0.0f), error_m1_(0.0f), integral_(0.0f) {}

void PID::setSetpoint(float target) { setpoint_ = target; }
void PID::setKp(float kp) { kp_ = kp; }
void PID::setKi(float ki) { ki_ = ki; }
void PID::setKd(float kd) { kd_ = kd; }

float PID::update(float measurement, float dt) {
    if (dt <= 0.0f) return 0.0f;

    float error = setpoint_ - measurement;
    
    // 1. Proportional
    float p_term = kp_ * error;

    // 2. Integral (with Anti-Windup)
    integral_ += error * dt;
    float i_term = ki_ * integral_;

    // 3. Derivative
    float derivative = (error - error_m1_) / dt;
    float d_term = kd_ * derivative;

    float output = p_term + i_term + d_term;

    // 4. Clamping & Anti-Windup logic
    if (output > max_) {
        output = max_;
        integral_ -= error * dt; // Undo integration if saturated
    } else if (output < min_) {
        output = min_;
        integral_ -= error * dt; 
    }

    error_m1_ = error;
    return output;
}

void PID::reset() {
    error_m1_ = 0.0f;
    integral_ = 0.0f;
}