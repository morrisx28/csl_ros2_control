#include "biped_wheel_controller/biped_wheel_controller.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <yaml-cpp/yaml.h>

namespace biped_wheel_controller
{

BipedWheelController::BipedWheelController() = default;
using config_type = controller_interface::interface_configuration_type;

controller_interface::InterfaceConfiguration BipedWheelController::command_interface_configuration() const
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

controller_interface::InterfaceConfiguration BipedWheelController::state_interface_configuration() const
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

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BipedWheelController::on_init()
{
  try
  {
    auto node = get_node();

    policy_path_ = auto_declare<std::string>("policy_path", "");
    config_path_ = auto_declare<std::string>("config_path", "");

    cmd_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg)
      {
        cmd_[0] = msg->linear.x;
        cmd_[1] = msg->linear.y;
        cmd_[2] = msg->angular.z;
      });

    // string: sit, stand, move  
    mode_sub_ = node->create_subscription<std_msgs::msg::String>(
      "/mode", 10, [this](const std_msgs::msg::String::SharedPtr msg)
      {
        mode_ = msg->data;
      });

    joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
    feet_names_ = auto_declare<std::vector<std::string>>("feet_names", feet_names_);
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

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BipedWheelController::on_configure(const rclcpp_lifecycle::State &)
{
  try
  {
    auto node = get_node();

    policy_ = torch::jit::load(policy_path_);
    YAML::Node config = YAML::LoadFile(config_path_);

    sit_angles_ = config["sit_angles"].as<std::vector<float>>();
    default_angles_ = config["default_angles"].as<std::vector<float>>();
    kps_ = config["kps"].as<std::vector<float>>();
    kds_ = config["kds"].as<std::vector<float>>();

    pos_action_scale_ = config["pos_action_scale"].as<float>();
    vel_action_scale_ = config["vel_action_scale"].as<float>();
    cmd_scale_ = config["cmd_scale"].as<std::vector<float>>();
    ang_vel_scale_ = config["ang_vel_scale"].as<float>();
    dof_pos_scale_ = config["dof_pos_scale"].as<float>();
    dof_vel_scale_ = config["dof_vel_scale"].as<float>();

    cmd_ = config["cmd_init"].as<std::vector<float>>();

    obs_buf_size_ = config["obs_buffer_size"].as<int>();
    one_step_obs_size_ = config["one_step_obs_size"].as<int>();

    obs_buffer_ = torch::zeros({1, obs_buf_size_ * one_step_obs_size_});
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "on_configure() failed: %s", e.what());
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BipedWheelController::on_activate(const rclcpp_lifecycle::State &)
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

    current_pos_ = get_current_pos();
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "on_activate() failed: %s", e.what());
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}
std::chrono::steady_clock::time_point last_time_;

controller_interface::return_type BipedWheelController::update(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Check the frequency
  // auto now = std::chrono::steady_clock::now();
  // if (last_time_.time_since_epoch().count() != 0) {
  //   auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_time_);
  //   double frequency = 1.0 / duration.count();  // 計算頻率（Hz）
  //   // std::cout << "Update frequency: " << frequency << " Hz" << std::endl;
  // }
  // last_time_ = now;

  if (mode_ != prev_mode_) {
    is_mode_change_ = true;
    step_ = 0;
    current_pos_ = get_current_pos();
    prev_mode_ = mode_;
  } else {
    is_mode_change_ = false;
  }

  if (mode_ == "sit") {
    sit(step_);
  } else if (mode_ == "sit_down") {
    sit_down(step_, current_pos_);
  } else if (mode_ == "move") {
    move();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid mode: %s", mode_.c_str());
  }

  if (step_ < steps_) {
    step_++;
  }

  return controller_interface::return_type::OK;
}

std::vector<float> BipedWheelController::get_current_pos()
{
  std::vector<float> pos(6);
  for (int i = 0; i < 6; ++i)
  {
    pos[i] = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
  }
  return pos;
}

void BipedWheelController::sit(int step) 
{
  double phase = float(step)/float(steps_);
  if (step < steps_) {
    for (int i = 0; i < 6; ++i)
    {
      ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(default_angles_[i]);
      ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(kps_[i]);
      ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(kds_[i]);
    }
  } 
  else {
    for (int i = 0; i < 6; ++i)
    {
      float target_pos = default_angles_[i] * float(1 - phase) + sit_angles_[i] * phase;
      ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(sit_angles_[i]);
      ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(kps_[i]);
      ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(kds_[i]);
    }
  }
}

void BipedWheelController::sit_down(int step, std::vector<float> current_pos) 
{
  if (step < steps_) {
    double phase = float(step)/float(steps_);
    for (int i = 0; i < 6; ++i)
    {
      float target_pos = current_pos[i] * float(1 - phase) + default_angles_[i] * phase;
      ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(target_pos);
      ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(kps_[i]);
      ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(kds_[i]);
    }
  } else {
    for (int i = 0; i < 6; ++i)
    {
      ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(default_angles_[i]);
      ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(kps_[i]);
      ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(kds_[i]);
    }
  }
}

void BipedWheelController::move()
{
  std::vector<float> pos(6), vel(6), ang_vel(3), quat(4);

  for (int i = 0; i < 6; ++i)
  {
    pos[i] = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
  }
  for (int i = 0; i < 6; ++i)
  {
    vel[i] = ctrl_interfaces_.joint_velocity_state_interface_[i].get().get_value();
  }

  for (int i = 0; i < 4; ++i)
  {
    quat[i] = ctrl_interfaces_.imu_state_interface_[i].get().get_value();
  }
  for (int i = 0; i < 3; ++i)
  {
    ang_vel[i] = ctrl_interfaces_.imu_state_interface_[i + 4].get().get_value();
  }

  std::vector<float> gravity(3);
  gravity[0] = 2 * (quat[1] * quat[3] - quat[0] * quat[2]);
  gravity[1] = -2 * (quat[2] * quat[3] + quat[0] * quat[1]);
  gravity[2] = -1 - 2 * (quat[1] * quat[1] + quat[2] * quat[2]);

  std::vector<torch::Tensor> obs_parts = {
    torch::tensor(latest_cmd_) * torch::tensor(cmd_scale_),
    torch::tensor(ang_vel) * ang_vel_scale_ * 0.3,
    torch::tensor(gravity),
    (torch::tensor(pos) - torch::tensor(default_angles_)) * dof_pos_scale_,
    torch::tensor(vel) * dof_vel_scale_,
    prev_action_
  };

  auto obs = torch::cat(obs_parts).unsqueeze(0);
  
  obs_buffer_ = torch::cat({
    obs, 
    obs_buffer_.slice(1, 0, obs_buffer_.size(1) - one_step_obs_size_)
  }, 1);

  obs = torch::clamp(obs, -100, 100);

  auto action = policy_.forward({obs_buffer_}).toTensor().squeeze();
  prev_action_ = action;

  // left thigh pos cmd
  ctrl_interfaces_.joint_position_command_interface_[0].get().set_value(action[0].item<float>() * pos_action_scale_ + default_angles_[0]);
  ctrl_interfaces_.joint_kp_command_interface_[0].get().set_value(kps_[0]);
  ctrl_interfaces_.joint_kd_command_interface_[0].get().set_value(kds_[0]);
  // left calf pos cmd
  ctrl_interfaces_.joint_position_command_interface_[1].get().set_value(action[1].item<float>() * pos_action_scale_ + default_angles_[1]);
  ctrl_interfaces_.joint_kp_command_interface_[1].get().set_value(kps_[1]);
  ctrl_interfaces_.joint_kd_command_interface_[1].get().set_value(kds_[1]);
  // left wheel vel cmd
  ctrl_interfaces_.joint_velocity_command_interface_[2].get().set_value(action[2].item<float>() * vel_action_scale_ + default_angles_[2]);
  ctrl_interfaces_.joint_kp_command_interface_[2].get().set_value(kps_[2]);
  ctrl_interfaces_.joint_kd_command_interface_[2].get().set_value(kds_[2]);
  // right thigh pos cmd
  ctrl_interfaces_.joint_position_command_interface_[3].get().set_value(action[3].item<float>() * pos_action_scale_ + default_angles_[3]);
  ctrl_interfaces_.joint_kp_command_interface_[3].get().set_value(kps_[3]);
  ctrl_interfaces_.joint_kd_command_interface_[3].get().set_value(kds_[3]);
  // right calf pos cmd
  ctrl_interfaces_.joint_position_command_interface_[4].get().set_value(action[4].item<float>() * pos_action_scale_ + default_angles_[4]);
  ctrl_interfaces_.joint_kp_command_interface_[4].get().set_value(kps_[4]);
  ctrl_interfaces_.joint_kd_command_interface_[4].get().set_value(kds_[4]);
  // right wheel vel cmd
  ctrl_interfaces_.joint_velocity_command_interface_[5].get().set_value(action[5].item<float>() * vel_action_scale_ + default_angles_[5]);
  ctrl_interfaces_.joint_kp_command_interface_[5].get().set_value(kps_[5]);
  ctrl_interfaces_.joint_kd_command_interface_[5].get().set_value(kds_[5]);

  for (int i = 0; i < 3; ++i)
  {
    latest_cmd_[i] = cmd_[i];
  }
}

} // namespace biped_wheel_controller

PLUGINLIB_EXPORT_CLASS(biped_wheel_controller::BipedWheelController, controller_interface::ControllerInterface)
