#include "swerve_controller/swerve_controller.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <yaml-cpp/yaml.h>

namespace swerve_controller
{

SwerveController::SwerveController() = default;
using config_type = controller_interface::interface_configuration_type;

controller_interface::InterfaceConfiguration SwerveController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size() * command_interface_types_.size());
  for (const auto& joint_name : joint_names_)
  {
      for (const auto& interface_type : command_interface_types_)
      {
          if (!command_prefix_.empty())
          {
              conf.names.push_back(command_prefix_ + "/" + joint_name + "/" + interface_type);
          }
          else
          {
              conf.names.push_back(joint_name + "/" + interface_type);
          }
      }
  }

  return conf;
}

controller_interface::InterfaceConfiguration SwerveController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size() * state_interface_types_.size());
  for (const auto& joint_name : joint_names_)
  {
      for (const auto& interface_type : state_interface_types_)
      {
          conf.names.push_back(joint_name + "/" + interface_type);
      }
  }

  for (const auto& interface_type : imu_interface_types_)
  {
      conf.names.push_back(imu_name_ + "/" + interface_type);
  }


  return conf;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SwerveController::on_init()
{
  try
  {
    auto node = get_node();

    config_path_ = auto_declare<std::string>("config_path", "");

    cmd_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg)
      {
        cmd_[0] = msg->linear.x;
        cmd_[1] = msg->linear.y;
        cmd_[2] = msg->angular.z;
      });

    // string: move  
    mode_sub_ = node->create_subscription<std_msgs::msg::String>(
      "/mode", 10, [this](const std_msgs::msg::String::SharedPtr msg)
      {
        mode_ = msg->data;
      });

    joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
    command_interface_types_ =
        auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
    state_interface_types_ =
        auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

    command_prefix_ = auto_declare<std::string>("command_prefix", command_prefix_);
    base_name_ = auto_declare<std::string>("base_name", base_name_);

    // imu sensor
    imu_name_ = auto_declare<std::string>("imu_name", imu_name_);
    imu_interface_types_ = auto_declare<std::vector<std::string>>("imu_interfaces", state_interface_types_);
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init() failed: %s", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SwerveController::on_configure(const rclcpp_lifecycle::State &)
{
  try
  {
    auto node = get_node();

    YAML::Node config = YAML::LoadFile(config_path_);

    initial_angles_ = config["default_angles"].as<std::vector<float>>();
    steering_kps_ = config["steering_kps"].as<float>();
    steering_kds_ = config["steering_kds"].as<float>();
    wheel_kps_ = config["wheel_kps"].as<float>();
    wheel_kds_ = config["wheel_kds"].as<float>();
    wheel_b_ = config["wheelbase"].as<float>();
    t_width_ = config["trackwidth"].as<float>();

    cmd_scale_ = config["cmd_scale"].as<float>();
    cmd_ = config["cmd_init"].as<std::vector<float>>();

    // FL FR RL RR
    x_offset_ = {(wheel_b_ / 2), (wheel_b_ / 2), -(wheel_b_ / 2), -(wheel_b_ / 2)}; 
    y_offset_ = {(t_width_ / 2), -(t_width_ / 2), (t_width_ / 2), -(t_width_ / 2)};

  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "on_configure() failed: %s", e.what());
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SwerveController::on_activate(const rclcpp_lifecycle::State &)
{
  try
  { 
    // clear out vectors in case of restart
    ctrl_interfaces_.clear();

    // assign command interfaces
    for (auto& interface : command_interfaces_)
    {
        std::string interface_name = interface.get_interface_name();
        if (const size_t pos = interface_name.find('/'); pos != std::string::npos)
        {
            command_interface_map_[interface_name.substr(pos + 1)]->push_back(interface);
        }
        else
        {
            command_interface_map_[interface_name]->push_back(interface);
        }
    }

    // assign state interfaces
    for (auto& interface : state_interfaces_)
    {
        if (interface.get_prefix_name() == imu_name_)
        {
            ctrl_interfaces_.imu_state_interface_.emplace_back(interface);
        }
        else
        {
            state_interface_map_[interface.get_interface_name()]->push_back(interface);
        }
    }
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "on_activate() failed: %s", e.what());
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}
std::chrono::steady_clock::time_point last_time_;

controller_interface::return_type SwerveController::update(const rclcpp::Time &, const rclcpp::Duration &)
{

  if (mode_ != prev_mode_) {
    is_mode_change_ = true;
    prev_mode_ = mode_;
  } else {
    is_mode_change_ = false;
  }

  if (mode_ == "move") {
    move();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid mode: %s", mode_.c_str());
  }

  return controller_interface::return_type::OK;
}

void SwerveController::update_current_state()
{
  for (int i = 0; i < 4; ++i)
  {
    steering_angles_[i] = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
  }

  for (int i = 0; i < 4; ++i)
  {
    wheel_vel_[i] = ctrl_interfaces_.joint_velocity_state_interface_[i+4].get().get_value();
  }
  
  for (int i = 0; i < 3; ++i)
  {
    ang_vel_[i] = ctrl_interfaces_.imu_state_interface_[i+4].get().get_value();
  }

  for (int i = 0; i < 4; ++i)
  {
    quat_[i] = ctrl_interfaces_.imu_state_interface_[i].get().get_value();
  }
}

void SwerveController::normalize_angle(float &angle, float &speed)
{
  angle = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (angle < 0)
      angle += 2.0 * M_PI;
  angle -= M_PI;

  // if beyond 90°, flip direction
  if (angle > M_PI_2) {
      angle -= M_PI;
      speed *= -1.0;
  } else if (angle < -M_PI_2) {
      angle += M_PI;
      speed *= -1.0;
  }
}

void SwerveController::move()
{
  update_current_state(); 

  for (int i = 0; i < 4; ++i)
  {
    float delta_vx = -cmd_[2] * y_offset_[i];
    float delta_vy = cmd_[2] * x_offset_[i];

    float total_vx = cmd_[0] + delta_vx;
    float total_vy = cmd_[1] + delta_vy;

    float speed = std::sqrt(total_vx * total_vx + total_vy * total_vy);
    float angle = std::atan2(total_vy, total_vx);
    normalize_angle(angle, speed);
    steering_cmd_[i] = angle;
    wheel_cmd_[i] = speed;
  }
  
  // Steering motor control
  for (int i = 0; i < 4; ++i)
  {
    ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(steering_cmd_[i]);
    ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(steering_kps_);
    ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(steering_kds_);
  }

  // Wheel motor control
  for (int i = 0; i < 4; ++i)
  {
    ctrl_interfaces_.joint_velocity_command_interface_[i+4].get().set_value(wheel_cmd_[i] * cmd_scale_);
    ctrl_interfaces_.joint_kp_command_interface_[i+4].get().set_value(wheel_kps_);
    ctrl_interfaces_.joint_kd_command_interface_[i+4].get().set_value(wheel_kds_);
  }

}

} // namespace swerve_controller

PLUGINLIB_EXPORT_CLASS(swerve_controller::SwerveController, controller_interface::ControllerInterface)
