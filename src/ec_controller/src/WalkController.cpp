// =============================================================================
//  Copyright (c) 2026 Phanerox. All rights reserved.
// =============================================================================

#include <pluginlib/class_list_macros.hpp>
#include "ec_controller/ControllerBase.h"
#include "ec_controller/AmpPolicy.h"
#include "ec_controller/ImuReader.h"
#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ec_radio/msg/radio.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <system_error>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace ec
{

static const std::vector<std::string> kImuInterfaces = {
  "imu_sensor/orientation.x", "imu_sensor/orientation.y",
  "imu_sensor/orientation.z", "imu_sensor/orientation.w",
  "imu_sensor/angular_velocity.x", "imu_sensor/angular_velocity.y",
  "imu_sensor/angular_velocity.z", "imu_sensor/linear_acceleration.x",
  "imu_sensor/linear_acceleration.y", "imu_sensor/linear_acceleration.z",
};

class WalkController : public ControllerBase {
public:
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override
  {
    auto ret = ControllerBase::on_configure(previous_state);
    if (ret != controller_interface::CallbackReturn::SUCCESS) {return ret;}
    auto node = get_node();
    const auto share = std::filesystem::path(ament_index_cpp::get_package_share_directory("ec_controller"));
    const auto policy_path = node->declare_parameter<std::string>("policy_path", "");
    const auto controllers_file = node->declare_parameter<std::string>("controllers_file", "config/policy/amp.yaml");
    try {
      declarePolicyParameters((share / controllers_file).string());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger(), "Failed to configure AMP model parameters: %s", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }
    node->declare_parameter<double>("imu_pitch_offset_deg", 0.0);

    default_pos_ = node->get_parameter("walk_pos").as_double_array();
    walk_kp_ = node->get_parameter("walk_kp").as_double_array();
    walk_kd_ = node->get_parameter("walk_kd").as_double_array();
    policy_joints_ = node->get_parameter("policy_joints").as_string_array();
    action_size_ = node->get_parameter("action.action_size").as_int();
    obs_size_ = node->get_parameter("action.observation_size").as_int();
    num_history_ = node->get_parameter("action.num_history").as_int();
    const int policy_freq = node->get_parameter("action.policy_freq").as_int();
    action_scale_ = node->get_parameter("action.action_scale").as_double_array();
    const double imu_pitch_offset_deg = node->get_parameter("imu_pitch_offset_deg").as_double();
    if (!std::isfinite(imu_pitch_offset_deg)) {
      RCLCPP_ERROR(logger(), "imu_pitch_offset_deg must be finite");
      return controller_interface::CallbackReturn::ERROR;
    }
    imu_.setPitchOffset(imu_pitch_offset_deg);

    const size_t n = jointCount();
    if (n == 0 || action_size_ != static_cast<int>(n) ||
      obs_size_ != 9 + 3 * static_cast<int>(n) || num_history_ <= 0 || policy_freq <= 0 ||
      default_pos_.size() != n || walk_kp_.size() != n || walk_kd_.size() != n ||
      policy_joints_.size() != n || action_scale_.size() != n)
    {
      RCLCPP_ERROR(logger(), "Inconsistent joint arrays or policy dimensions/frequency");
      return controller_interface::CallbackReturn::ERROR;
    }
    for (size_t i = 0; i < n; ++i) {
      if (!std::isfinite(default_pos_[i]) || !std::isfinite(walk_kp_[i]) ||
        !std::isfinite(walk_kd_[i]) || !std::isfinite(action_scale_[i]) ||
        walk_kp_[i] < 0.0 || walk_kd_[i] < 0.0 || action_scale_[i] <= 0.0)
      {
        RCLCPP_ERROR(logger(), "Invalid PD parameters or action scale at index %zu", i);
        return controller_interface::CallbackReturn::ERROR;
      }
    }
    inference_period_ = 1.0 / policy_freq;

    // Policy order comes from the exported training configuration, never an interleaved table.
    std::unordered_map<std::string, size_t> policy_index;
    for (size_t i = 0; i < n; ++i) {
      if (!policy_index.emplace(policy_joints_[i], i).second) {
        RCLCPP_ERROR(logger(), "Duplicate policy joint: %s", policy_joints_[i].c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
    }
    joint_to_policy_.resize(n);
    policy_to_joint_.resize(n);
    std::vector<bool> matched(n, false);
    for (size_t i = 0; i < n; ++i) {
      auto it = policy_index.find(joint_names_[i]);
      if (it == policy_index.end() || matched[it->second]) {
        RCLCPP_ERROR(logger(), "Unmatched or duplicate controller joint: %s", joint_names_[i].c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
      joint_to_policy_[i] = it->second;
      policy_to_joint_[it->second] = i;
      matched[it->second] = true;
    }

    if (policy_path.empty()) {
      RCLCPP_ERROR(logger(), "policy_path must identify an exported ONNX policy");
      return controller_interface::CallbackReturn::ERROR;
    }
    sample_.position.resize(n);
    sample_.velocity.resize(n);
    std::vector<double> policy_default(n);
    for (size_t p = 0; p < n; ++p) {policy_default[p] = default_pos_[policy_to_joint_[p]];}
    try {
      policy_ = std::make_unique<amp::Policy>();
      policy_->load((share / policy_path).string(), policy_default, action_scale_, num_history_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger(), "Failed to configure AMP policy: %s", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    radio_sub_ = node->create_subscription<ec_radio::msg::Radio>(
      "/radio", 10,
      [this](const ec_radio::msg::Radio::SharedPtr msg) {
        radio_vx_.store(msg->vx);
        radio_vy_.store(msg->vy);
        radio_wz_.store(msg->wz);
      });
    node->declare_parameter<std::string>("log_dir", "/home/milkyway/log/hunter130");
    log_dir_ = node->get_parameter("log_dir").as_string();

    RCLCPP_INFO(logger(), "Loaded policy: %s (frame=%d, actions=%d, history=%d)",
      policy_path.c_str(), obs_size_, action_size_, num_history_);
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override
  {
    auto ret = ControllerBase::on_activate(previous_state);
    if (ret != controller_interface::CallbackReturn::SUCCESS) {return ret;}
    try {
      imu_.bind(state_interfaces_);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(logger(), "AMP activation failed: %s", error.what());
      return controller_interface::CallbackReturn::ERROR;
    }
    policy_->reset();
    inference_accumulator_ = 0.0;
    openLogFile();
    RCLCPP_INFO(logger(), "WalkController activated; first observation will initialize all history frames");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override
  {
    policy_->reset();
    closeLogFile();
    return ControllerBase::on_deactivate(previous_state);
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    auto config = ControllerBase::state_interface_configuration();
    config.names.insert(config.names.end(), kImuInterfaces.begin(), kImuInterfaces.end());
    return config;
  }

protected:
  void onUpdate(const rclcpp::Time & time, const rclcpp::Duration & period) override
  {
    inference_accumulator_ += period.seconds();
    if (!policy_->initialized() || inference_accumulator_ >= inference_period_) {
      inference_accumulator_ = std::fmod(inference_accumulator_, inference_period_);
      if (buildSample()) {
        try {
          policy_->infer(sample_);
        } catch (const std::exception & e) {
          RCLCPP_ERROR_THROTTLE(logger(), *get_node()->get_clock(), 1000,
            "AMP inference failed: %s", e.what());
        }
      } else {
        RCLCPP_ERROR_THROTTLE(logger(), *get_node()->get_clock(), 1000,
          "Cannot build policy observation: missing or non-finite IMU/joint/command data");
      }
    }

    // Hold the latest policy targets between 50 Hz inference steps.
    for (size_t i = 0; i < jointCount(); ++i) {
      const size_t policy_index = joint_to_policy_[i];
      auto & cmd = jointCommand(i);
      cmd.position = policy_->target()[policy_index];
      cmd.velocity = 0.0;
      cmd.effort = 0.0;
      cmd.kp = walk_kp_[i];
      cmd.kd = walk_kd_[i];
      cmd.torque_mode = isCoupledJoint(joint_names_[i]) ? 2.0 : 0.0;
    }
    writeLogRow(time);
  }

private:
  void declarePolicyParameters(const std::string & controllers_file)
  {
    std::vector<double> positions, kp, kd, scales;
    const auto config = YAML::LoadFile(controllers_file)["amp_model"]["ros__parameters"];
    const auto names = config["joint_names"].as<std::vector<std::string>>();
    const auto joints = config["joints"];
    for (const auto & name : names) {
      scales.push_back(joints[name]["action_scale"].as<double>());
    }
    // The controller's PD and pose arrays follow hardware order, scales follow policy order.
    for (const auto & name : joint_names_) {
      const auto joint = joints[name];
      positions.push_back(joint["default_position"].as<double>());
      kp.push_back(joint["kp"].as<double>());
      kd.push_back(joint["kd"].as<double>());
    }
    const int actions = config["output_shape"][1].as<int>();
    const int frame_size = config["frame_size"].as<int>();
    const int history = config["history_length"].as<int>();
    const int frequency = config["policy_frequency_hz"].as<int>();
    auto node = get_node();
    node->declare_parameter("walk_pos", positions);
    node->declare_parameter("walk_kp", kp);
    node->declare_parameter("walk_kd", kd);
    node->declare_parameter("policy_joints", names);
    node->declare_parameter("action.action_size", actions);
    node->declare_parameter("action.observation_size", frame_size);
    node->declare_parameter("action.num_history", history);
    node->declare_parameter("action.policy_freq", frequency);
    node->declare_parameter("action.action_scale", scales);
  }

  // One file per activation, with columns in the configured controller joint order.
  void openLogFile()
  {
    closeLogFile();
    log_file_.clear();
    namespace fs = std::filesystem;
    const fs::path dir = log_dir_.empty() ? fs::path("/home/milkyway/log/hunter130") : fs::path(log_dir_);
    std::error_code error;
    fs::create_directories(dir, error);
    if (error) {
      RCLCPP_WARN(logger(), "Failed to create walk log directory %s: %s",
        dir.c_str(), error.message().c_str());
      return;
    }
    size_t run_id = 1;
    fs::path path;
    do {
      path = dir / ("walk_log_" + std::to_string(run_id++) + ".csv");
    } while (fs::exists(path, error) && !error);
    if (error) {
      RCLCPP_WARN(logger(), "Failed to select walk log file: %s", error.message().c_str());
      return;
    }
    log_path_ = path.string();
    log_file_.open(log_path_, std::ios::out | std::ios::trunc);
    if (!log_file_.is_open()) {
      RCLCPP_WARN(logger(), "Failed to open walk log file: %s", log_path_.c_str());
      return;
    }
    log_file_ << std::setprecision(17) << "time_ns";
    for (const char * field : {"mpos", "mvel", "meff", "cpos", "ckp", "ckd"}) {
      for (size_t i = 0; i < jointCount(); ++i) {
        log_file_ << "," << field << i;
      }
    }
    log_file_ << "\n";
    RCLCPP_INFO(logger(), "Walk log file: %s", log_path_.c_str());
  }

  void closeLogFile()
  {
    if (log_file_.is_open()) {
      log_file_.close();
      RCLCPP_INFO(logger(), "Walk log file closed: %s", log_path_.c_str());
    }
  }

  void writeLogRow(const rclcpp::Time & time)
  {
    if (!log_file_.is_open()) {return;}
    const size_t n = jointCount();
    log_file_ << time.nanoseconds();
    for (size_t i = 0; i < n; ++i) {log_file_ << "," << jointState(i).position;}
    for (size_t i = 0; i < n; ++i) {log_file_ << "," << jointState(i).velocity;}
    for (size_t i = 0; i < n; ++i) {log_file_ << "," << jointState(i).effort;}
    for (size_t i = 0; i < n; ++i) {log_file_ << "," << jointCommand(i).position;}
    for (size_t i = 0; i < n; ++i) {log_file_ << "," << jointCommand(i).kp;}
    for (size_t i = 0; i < n; ++i) {log_file_ << "," << jointCommand(i).kd;}
    // Buffered output; deactivation closes the stream and flushes the remaining rows.
    log_file_ << "\n";
    if (!log_file_) {
      RCLCPP_WARN(logger(), "Walk CSV write failed; logging disabled for this activation: %s",
        log_path_.c_str());
      log_file_.close();
    }
  }

  bool buildSample()
  {
    if (!imu_.read(state_interfaces_, sample_.orientation, sample_.angular_velocity)) {return false;}
    sample_.command = {radio_vx_.load(), radio_vy_.load(), radio_wz_.load()};
    for (size_t p = 0; p < policy_joints_.size(); ++p) {
      const size_t joint = policy_to_joint_[p];
      sample_.position[p] = jointState(joint).position;
      sample_.velocity[p] = jointState(joint).velocity;
    }
    return true;
  }

  std::vector<double> default_pos_, walk_kp_, walk_kd_, action_scale_;
  std::vector<std::string> policy_joints_;
  std::vector<size_t> joint_to_policy_, policy_to_joint_;
  int action_size_{25};
  int obs_size_{84};
  int num_history_{4};
  double inference_period_{0.02};
  double inference_accumulator_{0.0};
  ImuReader imu_;
  amp::Sample sample_;
  std::unique_ptr<amp::Policy> policy_;
  std::ofstream log_file_;
  std::string log_path_;
  std::string log_dir_;
  rclcpp::Subscription<ec_radio::msg::Radio>::SharedPtr radio_sub_;
  std::atomic<float> radio_vx_{0.0f}, radio_vy_{0.0f}, radio_wz_{0.0f};
};

} // namespace ec

PLUGINLIB_EXPORT_CLASS(ec::WalkController, controller_interface::ControllerInterface)
