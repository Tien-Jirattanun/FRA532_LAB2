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
    double x, y, z, d_x, d_y, d_z;
    double roll, pitch, yaw, d_roll, d_pitch, d_yaw;
};

class LQRController : public rclcpp::Node
{
  public:
    LQRController()
        : Node("lqr_controller")
    {

        // Initial default hover state
        target_state_ = {0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

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

        K.setZero();
        K << -0.00000000,-0.00000000,5.47722558,-0.00000000, -0.00000000,4.62943590,0.00000000,0.00000000, -0.00000000,-0.00000000,0.00000000,-0.00000000, 0.00000000,-4.47213595,-0.00000000,0.00000000, -3.36841250,-0.00000000,6.96046028,0.00000000, -0.00000000,0.76409403,0.00000000,-0.00000000, 4.47213595,0.00000000,0.00000000,3.59823503, 0.00000000,0.00000000,-0.00000000,8.71652470, -0.00000000,-0.00000000,1.14904894,-0.00000000,-0.00000000,0.00000000,0.00000000,-0.00000000, -0.00000000,0.00000000,-0.00000000,-0.00000000, 1.41421356,-0.00000000,-0.00000000,0.61346339;

        motor_speed_pub_ =
            this->create_publisher<actuator_msgs::msg::Actuators>("motor_commands", 10);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&LQRController::odom_callback, this, _1));

        setpoint_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "setpoint", 10, std::bind(&LQRController::setpoint_callback, this, _1));

        timer_ = this->create_wall_timer(10ms, std::bind(&LQRController::timer_callback, this));
        last_time_ = this->now();

    }

  private:

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        quad_state_.x = msg->pose.pose.position.x;
        quad_state_.y = msg->pose.pose.position.y;
        quad_state_.z = msg->pose.pose.position.z;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3(q).getRPY(quad_state_.roll, quad_state_.pitch, quad_state_.yaw);

        quad_state_.d_x = msg->twist.twist.linear.x;
        quad_state_.d_y = msg->twist.twist.linear.y;
        quad_state_.d_z = msg->twist.twist.linear.z;

        quad_state_.d_roll  = msg->twist.twist.angular.x;
        quad_state_.d_pitch = msg->twist.twist.angular.y;
        quad_state_.d_yaw   = msg->twist.twist.angular.z;
    }

    void setpoint_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        target_state_.x = msg->data[0];
        target_state_.y = msg->data[1];
        target_state_.z = msg->data[2];
    }

    void timer_callback()
    {

        Eigen::Matrix<double, 12, 1> x;
        x << quad_state_.x, quad_state_.y, quad_state_.z,
                quad_state_.d_x, quad_state_.d_y, quad_state_.d_z,
                quad_state_.roll, quad_state_.pitch, quad_state_.yaw,
                quad_state_.d_roll, quad_state_.d_pitch, quad_state_.d_yaw;

        Eigen::Matrix<double, 12, 1> x_ref;
        x_ref << target_state_.x, target_state_.y, target_state_.z,
                 target_state_.d_x, target_state_.d_y, target_state_.d_z,
                target_state_.roll, target_state_.pitch, target_state_.yaw,
                target_state_.d_roll, target_state_.d_pitch, target_state_.d_yaw;

        Eigen::Vector4d u = -K * (x - x_ref);

        // Feed forward
        u(0) += m * g; 

        Eigen::Vector4d f = A_inv_ * u;

        auto motor_msg = actuator_msgs::msg::Actuators();
        motor_msg.header.stamp = this->now();
    
        for (int i = 0; i < 4; ++i) {
            double force = std::max(0.0, f[i]);
            motor_msg.velocity.push_back(std::sqrt(force / k_F));
        }

        motor_speed_pub_->publish(motor_msg);
        
    }

    Eigen::Matrix<double, 4, 12> K;
    const double k_F = 8.54858e-06;
    const double m = 1.5; // kg (Match your URDF)
    const double g = 9.81;
    Eigen::Matrix4d A;
    Eigen::Matrix4d A_inv_;
    State quad_state_{}, target_state_{};
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
    rclcpp::spin(std::make_shared<LQRController>());
    rclcpp::shutdown();
    return 0;
}