#ifndef PID_HPP_
#define PID_HPP_

class PID {
public:
    /**
     * @brief Construct a new PID object
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param min_out Minimum output clamp
     * @param max_out Maximum output clamp
     */
    PID(float kp, float ki, float kd, float min_out, float max_out);

    /**
     * @brief Calculate the PID control signal
     * @param measurement The current sensor reading
     * @param dt Time step since last update
     * @return float The control output
     */
    float update(float measurement, float dt);

    /**
     * @brief Reset the integral and error history
     */
    void reset();

    // Setters for dynamic tuning
    void setSetpoint(float target);
    void setGains(float kp, float ki, float kd);
    void setKp(float kp);
    void setKi(float ki);
    void setKd(float kd);

private:
    // Gains
    float kp_, ki_, kd_;
    
    // Limits
    float min_, max_;

    // Controller State
    float setpoint_;
    float error_m1_;  // Error from the previous step (e[k-1])
    float integral_;  // Accumulated error for the I-term
};

#endif // PID_HPP_