#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "std_msgs/msg/float32_multi_array.hpp"
#include "actuator_msgs/msg/actuators.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "quad_controller/PID.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

struct State
{
    double x, y, z;
    double roll, pitch, yaw;
};

class PIDController : public rclcpp::Node
{
  public:
    PIDController()
        : Node("pid_controller"), pid_roll_(5.0, 0.0, 2.5, -15.0, 15.0),
          pid_pitch_(5.0, 0.0, 2.5, -15.0, 15.0), pid_yaw_(2.0, 0.0, 1.0, -0.5, 0.5),
          pid_alt_(3.0, 0.0, 2.0, -15.0, 15.0)
    {
        // 1. Declare Tuning Parameters
        this->declare_parameter("roll_kp", 5.0);
        this->declare_parameter("roll_ki", 0.0);
        this->declare_parameter("roll_kd", 2.5);

        this->declare_parameter("pitch_kp", 5.0);
        this->declare_parameter("pitch_ki", 0.0);
        this->declare_parameter("pitch_kd", 2.5);

        this->declare_parameter("yaw_kp", 2.5);
        this->declare_parameter("yaw_ki", 0.0);
        this->declare_parameter("yaw_kd", 1.0);

        this->declare_parameter("alt_kp", 2.0);
        this->declare_parameter("alt_ki", 0.0);
        this->declare_parameter("alt_kd", 2.5);

        // 2. Setup Parameter Callback (Interrupt Style)
        callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&PIDController::parameters_callback, this, _1));


        double dx_f = 0.13;  
        double dx_r = 0.13;  
        double dy_l = 0.22;  
        double dy_r = 0.22; 
        double dy_rl = 0.20; 
        double dy_rr = 0.20; 
        double k_m = 0.06;  

        A << 1.0,    1.0,    1.0,    1.0,    
            -dy_r,   dy_rl,  dy_l,  -dy_rr,  
            -dx_f,   dx_r,  -dx_f,   dx_r,   
            -k_m,    -k_m,   k_m,   k_m;   

        A_inv_ = A.inverse();

        motor_speed_pub_ =
            this->create_publisher<actuator_msgs::msg::Actuators>("motor_commands", 10);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&PIDController::odom_callback, this, _1));

        setpoint_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "cmd_pose", 10, std::bind(&PIDController::setpoint_callback, this, _1));

        timer_ = this->create_wall_timer(10ms, std::bind(&PIDController::timer_callback, this));
        last_time_ = this->now();

        // Initial default hover state
        target_state_ = {0.0, 0.0, 1.0, 0.0, 0.0, 0.0};

        pid_alt_.setSetpoint(target_state_.z);
        pid_roll_.setSetpoint(target_state_.roll);
        pid_pitch_.setSetpoint(target_state_.pitch);
        pid_yaw_.setSetpoint(target_state_.yaw);

    }

  private:
    // Interrupt: Runs only when "ros2 param set" is called
    rcl_interfaces::msg::SetParametersResult
    parameters_callback(const std::vector<rclcpp::Parameter> &parameters)
    {
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;
        for (const auto &param : parameters)
        {
            std::string n = param.get_name();
            double v = param.as_double();
            if (n == "roll_kp")
                pid_roll_.setKp(v);
            else if (n == "roll_ki")
                pid_roll_.setKi(v);
            else if (n == "roll_kd")
                pid_roll_.setKd(v);
            else if (n == "pitch_kp")
                pid_pitch_.setKp(v);
            else if (n == "pitch_ki")
                pid_pitch_.setKi(v);
            else if (n == "pitch_kd")
                pid_pitch_.setKd(v);
            else if (n == "yaw_kp")
                pid_yaw_.setKp(v);
            else if (n == "yaw_ki")
                pid_yaw_.setKi(v);
            else if (n == "yaw_kd")
                pid_yaw_.setKd(v);
            else if (n == "alt_kp")
                pid_alt_.setKp(v);
            else if (n == "alt_ki")
                pid_alt_.setKi(v);
            else if (n == "alt_kd")
                pid_alt_.setKd(v);
        }
        return result;
    }

    void setpoint_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // Safety check: ensure the array has enough elements
        // Assuming format: [x, y, z, roll, pitch, yaw]
        if (msg->data.size() < 6) {
            RCLCPP_WARN(this->get_logger(), "Received setpoint array too small! Need 6 values.");
            return;
        }

        // 1. Update internal target state
        target_state_.x     = msg->data[0];
        target_state_.y     = msg->data[1];
        target_state_.z     = msg->data[2];
        target_state_.roll  = msg->data[3];
        target_state_.pitch = msg->data[4];
        target_state_.yaw   = msg->data[5];

        // 2. Update PIDs
        pid_alt_.setSetpoint(target_state_.z);
        pid_roll_.setSetpoint(target_state_.roll);
        pid_pitch_.setSetpoint(target_state_.pitch);
        pid_yaw_.setSetpoint(target_state_.yaw);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        quad_state_.z = msg->pose.pose.position.z;
        tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                          msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3(q).getRPY(quad_state_.roll, quad_state_.pitch, quad_state_.yaw);
    }

    void timer_callback()
    {
        auto now = this->now();
        float dt = (now - last_time_).seconds();
        last_time_ = now;

        if (dt <= 0)
            return;

        // 1. Calculate Outputs (Thrust and Torques)
        // 1. PID Outputs
        float hover_thrust = 1.5f * 9.81f; 
        float T = hover_thrust + pid_alt_.update(quad_state_.z, dt);
        float t_phi = pid_roll_.update(quad_state_.roll, dt);
        float t_theta = pid_pitch_.update(quad_state_.pitch, dt);
        float t_psi = pid_yaw_.update(quad_state_.yaw, dt);

        // 2. Mixing
        Eigen::Vector4d u(T, t_phi, t_theta, t_psi);
        Eigen::Vector4d f = A_inv_ * u;

        const double k_F = 8.54858e-06f;
        auto motor_msg = actuator_msgs::msg::Actuators();
        motor_msg.header.stamp = this->now();
        motor_msg.header.frame_id = "base_link";

        for (int i = 0; i < 4; ++i)
        {
            double force = std::max(0.0, f[i]);
            motor_msg.velocity.push_back(std::sqrt(force / k_F));
        }

        motor_speed_pub_->publish(motor_msg); 

        RCLCPP_INFO(this->get_logger(), "Published Actuators: [%.2f, %.2f, %.2f, %.2f]",
                    motor_msg.velocity[0], motor_msg.velocity[1], motor_msg.velocity[2],
                    motor_msg.velocity[3]);
    }

    PID pid_roll_, pid_pitch_, pid_yaw_, pid_alt_;
    State quad_state_{}, target_state_{};
    Eigen::Matrix4d A;
    Eigen::Matrix4d A_inv_;
    rclcpp::Time last_time_;
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr motor_speed_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr setpoint_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDController>());
    rclcpp::shutdown();
    return 0;
}