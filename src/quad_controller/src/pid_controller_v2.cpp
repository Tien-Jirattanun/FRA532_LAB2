// #include <Eigen/Dense>
// #include <memory>
// #include <vector>
// #include <algorithm>
// #include <iostream>
// #include <map>

// #include "std_msgs/msg/float32_multi_array.hpp"
// #include "actuator_msgs/msg/actuators.hpp"
// #include "nav_msgs/msg/odometry.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "tf2/LinearMath/Matrix3x3.h"
// #include "tf2/LinearMath/Quaternion.h"

// using namespace std::chrono_literals;
// using std::placeholders::_1;

// // ─────────────────────────────────────────────
// //  Inline PID (no header needed)
// // ─────────────────────────────────────────────
// class PID
// {
// public:
//     PID(double kp, double ki, double kd, double i_max, double out_min, double out_max)
//         : kp_(kp), ki_(ki), kd_(kd),
//           i_max_(i_max), out_min_(out_min), out_max_(out_max),
//           integral_(0.0), prev_error_(0.0), first_(true) {}

//     void setGains(double kp, double ki, double kd) {
//         kp_ = kp; ki_ = ki; kd_ = kd;
//         reset();
//     }
//     double getKp() const { return kp_; }
//     double getKi() const { return ki_; }
//     double getKd() const { return kd_; }

//     void reset() {
//         integral_ = 0.0; prev_error_ = 0.0; first_ = true;
//     }

//     double compute(double error, double dt) {
//         if (dt <= 0.0) return 0.0;
//         integral_ += error * dt;
//         integral_  = std::clamp(integral_, -i_max_, i_max_);
//         double derivative = first_ ? 0.0 : (error - prev_error_) / dt;
//         if (first_) first_ = false;
//         prev_error_ = error;
//         return std::clamp(kp_*error + ki_*integral_ + kd_*derivative, out_min_, out_max_);
//     }

// private:
//     double kp_, ki_, kd_;
//     double i_max_, out_min_, out_max_;
//     double integral_, prev_error_;
//     bool   first_;
// };

// // ─────────────────────────────────────────────
// //  State struct
// // ─────────────────────────────────────────────
// struct State {
//     double x, y, z, d_x, d_y, d_z;
//     double roll, pitch, yaw, d_roll, d_pitch, d_yaw;
// };

// // ─────────────────────────────────────────────
// //  PID Controller Node
// // ─────────────────────────────────────────────
// class PIDController : public rclcpp::Node
// {
// public:
//     PIDController()
//         : Node("pid_controller"),
//           //              Kp    Ki    Kd    Imax  outMin  outMax
//           pid_x_    (1.5, 0.0,  0.8,  3.0,  -0.20,  0.20),
//           pid_y_    (1.5, 0.0,  0.8,  3.0,  -0.20,  0.20),
//           pid_z_    (5.0, 0.0,  3.0,  10.0, -5.0,   5.0 ),
//           pid_roll_ (8.0, 0.0,  3.0,  5.0,  -8.0,   8.0 ),
//           pid_pitch_(8.0, 0.0,  3.0,  5.0,  -8.0,   8.0 ),
//           pid_yaw_  (3.0, 0.0,  1.0,  3.0,  -3.0,   3.0 )
//     {
//         target_state_ = {0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//         // ── Declare all tunable parameters ──────────────────────────
//         this->declare_parameter("kp_x",     1.5);  this->declare_parameter("ki_x",     0.0);  this->declare_parameter("kd_x",     0.8);
//         this->declare_parameter("kp_y",     1.5);  this->declare_parameter("ki_y",     0.0);  this->declare_parameter("kd_y",     0.8);
//         this->declare_parameter("kp_z",     5.0);  this->declare_parameter("ki_z",     0.0);  this->declare_parameter("kd_z",     3.0);
//         this->declare_parameter("kp_roll",  8.0);  this->declare_parameter("ki_roll",  0.0);  this->declare_parameter("kd_roll",  3.0);
//         this->declare_parameter("kp_pitch", 8.0);  this->declare_parameter("ki_pitch", 0.0);  this->declare_parameter("kd_pitch", 3.0);
//         this->declare_parameter("kp_yaw",   3.0);  this->declare_parameter("ki_yaw",   0.0);  this->declare_parameter("kd_yaw",   1.0);

//         // ── Live parameter update callback ───────────────────────────
//         param_cb_ = this->add_on_set_parameters_callback(
//             [this](const std::vector<rclcpp::Parameter> & params) {
//                 for (const auto & p : params) {
//                     const std::string & n = p.get_name();
//                     double v = p.as_double();
//                     RCLCPP_INFO(this->get_logger(), "[TUNE] %s = %.4f", n.c_str(), v);

//                     if      (n=="kp_x")     pid_x_.setGains(v,                pid_x_.getKi(),     pid_x_.getKd());
//                     else if (n=="ki_x")     pid_x_.setGains(pid_x_.getKp(),   v,                  pid_x_.getKd());
//                     else if (n=="kd_x")     pid_x_.setGains(pid_x_.getKp(),   pid_x_.getKi(),     v);

//                     else if (n=="kp_y")     pid_y_.setGains(v,                pid_y_.getKi(),     pid_y_.getKd());
//                     else if (n=="ki_y")     pid_y_.setGains(pid_y_.getKp(),   v,                  pid_y_.getKd());
//                     else if (n=="kd_y")     pid_y_.setGains(pid_y_.getKp(),   pid_y_.getKi(),     v);

//                     else if (n=="kp_z")     pid_z_.setGains(v,                pid_z_.getKi(),     pid_z_.getKd());
//                     else if (n=="ki_z")     pid_z_.setGains(pid_z_.getKp(),   v,                  pid_z_.getKd());
//                     else if (n=="kd_z")     pid_z_.setGains(pid_z_.getKp(),   pid_z_.getKi(),     v);

//                     else if (n=="kp_roll")  pid_roll_.setGains(v,                pid_roll_.getKi(),  pid_roll_.getKd());
//                     else if (n=="ki_roll")  pid_roll_.setGains(pid_roll_.getKp(), v,                 pid_roll_.getKd());
//                     else if (n=="kd_roll")  pid_roll_.setGains(pid_roll_.getKp(), pid_roll_.getKi(), v);

//                     else if (n=="kp_pitch") pid_pitch_.setGains(v,                 pid_pitch_.getKi(),  pid_pitch_.getKd());
//                     else if (n=="ki_pitch") pid_pitch_.setGains(pid_pitch_.getKp(), v,                  pid_pitch_.getKd());
//                     else if (n=="kd_pitch") pid_pitch_.setGains(pid_pitch_.getKp(), pid_pitch_.getKi(), v);

//                     else if (n=="kp_yaw")   pid_yaw_.setGains(v,               pid_yaw_.getKi(),   pid_yaw_.getKd());
//                     else if (n=="ki_yaw")   pid_yaw_.setGains(pid_yaw_.getKp(), v,                  pid_yaw_.getKd());
//                     else if (n=="kd_yaw")   pid_yaw_.setGains(pid_yaw_.getKp(), pid_yaw_.getKi(),   v);
//                 }
//                 rcl_interfaces::msg::SetParametersResult result;
//                 result.successful = true;
//                 return result;
//             });

//         // ── Mixer matrix ─────────────────────────────────────────────
//         double dx_f=0.13, dx_r=0.13, dy_l=0.22, dy_r=0.22, dy_rl=0.20, dy_rr=0.20, k_m=0.06;
//         A_ << 1.0,    1.0,    1.0,    1.0,
//              -dy_r,  dy_rl,  dy_l,  -dy_rr,
//              -dx_f,  dx_r,  -dx_f,   dx_r,
//              -k_m,  -k_m,    k_m,    k_m;
//         A_inv_ = A_.inverse();

//         motor_speed_pub_ = this->create_publisher<actuator_msgs::msg::Actuators>("motor_commands", 10);
//         odom_sub_        = this->create_subscription<nav_msgs::msg::Odometry>(
//             "odom", 10, std::bind(&PIDController::odom_callback, this, _1));
//         setpoint_sub_    = this->create_subscription<std_msgs::msg::Float32MultiArray>(
//             "setpoint", 10, std::bind(&PIDController::setpoint_callback, this, _1));
//         timer_     = this->create_wall_timer(10ms, std::bind(&PIDController::timer_callback, this));
//         last_time_ = this->now();

//         RCLCPP_INFO(this->get_logger(), "PID Controller ready. Tune live with:");
//         RCLCPP_INFO(this->get_logger(), "  ros2 param set /pid_controller kp_z 6.0");
//         RCLCPP_INFO(this->get_logger(), "  ros2 param set /pid_controller kp_roll 10.0");
//     }

// private:

//     void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
//         quad_state_.x = msg->pose.pose.position.x;
//         quad_state_.y = msg->pose.pose.position.y;
//         quad_state_.z = msg->pose.pose.position.z;
//         tf2::Quaternion q(
//             msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
//             msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
//         tf2::Matrix3x3(q).getRPY(quad_state_.roll, quad_state_.pitch, quad_state_.yaw);
//         quad_state_.d_x     = msg->twist.twist.linear.x;
//         quad_state_.d_y     = msg->twist.twist.linear.y;
//         quad_state_.d_z     = msg->twist.twist.linear.z;
//         quad_state_.d_roll  = msg->twist.twist.angular.x;
//         quad_state_.d_pitch = msg->twist.twist.angular.y;
//         quad_state_.d_yaw   = msg->twist.twist.angular.z;
//     }

//     void setpoint_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
//         if (msg->data.size() < 3) return;
//         target_state_.x = msg->data[0];
//         target_state_.y = msg->data[1];
//         target_state_.z = msg->data[2];
//     }

//     void timer_callback() {
//         auto now = this->now();
//         double dt = (now - last_time_).seconds();
//         last_time_ = now;
//         if (dt <= 0.0 || dt > 0.5) return;

//         // Position loop → desired attitude
//         double err_x = std::clamp(target_state_.x - quad_state_.x, -2.0, 2.0);
//         double err_y = std::clamp(target_state_.y - quad_state_.y, -2.0, 2.0);
//         double err_z = std::clamp(target_state_.z - quad_state_.z, -2.0, 2.0);

//         double des_pitch    = -pid_x_.compute(err_x, dt);
//         double des_roll     =  pid_y_.compute(err_y, dt);
//         double thrust_delta =  pid_z_.compute(err_z, dt);

//         // Attitude loop → torques
//         double tau_roll  = pid_roll_ .compute(des_roll  - quad_state_.roll,  dt);
//         double tau_pitch = pid_pitch_.compute(des_pitch - quad_state_.pitch, dt);
//         double tau_yaw   = pid_yaw_  .compute(target_state_.yaw - quad_state_.yaw, dt);

//         Eigen::Vector4d u(m_*g_ + thrust_delta, tau_roll, tau_pitch, tau_yaw);
//         Eigen::Vector4d f = A_inv_ * u;

//         auto motor_msg = actuator_msgs::msg::Actuators();
//         motor_msg.header.stamp = this->now();
//         for (int i = 0; i < 4; ++i)
//             motor_msg.velocity.push_back(std::sqrt(std::max(0.0, f[i]) / k_F_));

//         motor_speed_pub_->publish(motor_msg);
//     }

//     // PIDs
//     PID pid_x_, pid_y_, pid_z_;
//     PID pid_roll_, pid_pitch_, pid_yaw_;

//     // Constants
//     const double k_F_ = 8.54858e-06;
//     const double m_   = 1.5;
//     const double g_   = 9.81;

//     // Mixer
//     Eigen::Matrix4d A_, A_inv_;

//     // State
//     State quad_state_{}, target_state_{};
//     rclcpp::Time last_time_;

//     // ROS
//     OnSetParametersCallbackHandle::SharedPtr param_cb_;
//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr motor_speed_pub_;
//     rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr    odom_sub_;
//     rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr setpoint_sub_;
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<PIDController>());
//     rclcpp::shutdown();
//     return 0;
// }