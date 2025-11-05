#pragma once

#include <controller_interface/controller_interface.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <tuple>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

struct CtrlInterfaces
{
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_torque_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_position_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_velocity_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_kp_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_kd_command_interface_;


    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_effort_state_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_position_state_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_velocity_state_interface_;

    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    imu_state_interface_;

    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    foot_force_state_interface_;

    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    odom_state_interface_;

    CtrlInterfaces() = default;

    void clear()
    {
        joint_torque_command_interface_.clear();
        joint_position_command_interface_.clear();
        joint_velocity_command_interface_.clear();
        joint_kd_command_interface_.clear();
        joint_kp_command_interface_.clear();

        joint_effort_state_interface_.clear();
        joint_position_state_interface_.clear();
        joint_velocity_state_interface_.clear();

        imu_state_interface_.clear();
        foot_force_state_interface_.clear();
    }
};

namespace swerve_controller
{

class SwerveController : public controller_interface::ControllerInterface
{
public:
  SwerveController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init() override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;

  void update_current_state();
  void move();
  void normalize_angle(float &angle, float &speed);
  void updateOdom(float )

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  std::string mode_ = "move";
  std::string prev_mode_;
  bool is_mode_change_ = false;

  std::string config_path_;

  CtrlInterfaces ctrl_interfaces_;
  std::vector<std::string> joint_names_;
  std::string base_name_ = "base";
  std::vector<std::string> command_interface_types_;
  std::vector<std::string> state_interface_types_;

  std::string command_prefix_;

  // IMU Sensor
  std::string imu_name_;
  std::vector<std::string> imu_interface_types_;

  std::unordered_map<
    std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>*>
  command_interface_map_ = {
    {"position", &ctrl_interfaces_.joint_position_command_interface_},
    {"velocity", &ctrl_interfaces_.joint_velocity_command_interface_},
    {"effort", &ctrl_interfaces_.joint_torque_command_interface_},
    {"kp", &ctrl_interfaces_.joint_kp_command_interface_},
    {"kd", &ctrl_interfaces_.joint_kd_command_interface_}
  };

  std::unordered_map<
    std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>*>
  state_interface_map_ = {
    {"position", &ctrl_interfaces_.joint_position_state_interface_},
    {"velocity", &ctrl_interfaces_.joint_velocity_state_interface_},
    {"effort", &ctrl_interfaces_.joint_effort_state_interface_}
  };

  std::vector<float> cmd_{0.0, 0.0, 0.0};
  std::vector<float> initial_angles_;
  float wheel_b_, t_width_;
  float cmd_scale_;
  float steering_kps_{0.0}, steering_kds_{0.0}, wheel_kps_{0.0}, wheel_kds_{0.0};
  std::vector<float> steering_angles_{0.0, 0.0, 0.0, 0.0}, wheel_vel_{0.0, 0.0, 0.0, 0.0}, ang_vel_{0.0, 0.0, 0.0}, quat_{0.0, 0.0, 0.0, 0.0}; 
  std::vector<float> steering_cmd_{0.0, 0.0, 0.0, 0.0}, wheel_cmd_{0.0, 0.0, 0.0, 0.0}; 

  std::vector<float> x_offset_, y_offset_; 
  Eigen::VectorXd qpos;
  Eigen::VectorXd qvel;
  Eigen::Vector4d default_angles;
  Eigen::Matrix<float, 4, 2> wheel_positions;
  float dt;
  float x, y, yaw;

  std::chrono::steady_clock::time_point last_time_;
};

}  // namespace swerve_controller
