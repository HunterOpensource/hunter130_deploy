#include "ec_controller/MimicPolicy.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace ec::mimic
{
namespace
{
std::vector<std::string> split(const std::string & line)
{
  std::istringstream stream(line);
  std::vector<std::string> result;
  std::string cell;
  while (std::getline(stream, cell, ',')) {result.push_back(cell);}
  return result;
}

Joints jointVector(const YAML::Node & config, const std::string & name)
{
  const auto values = config[name].as<std::vector<double>>();
  if (values.size() != kJoints) {throw std::runtime_error(name + " must have 25 values");}
  Joints result;
  std::copy(values.begin(), values.end(), result.data());
  if (!result.allFinite()) {throw std::runtime_error(name + " contains non-finite values");}
  return result;
}
}  // namespace

bool Sample::valid() const
{
  return position.allFinite() && velocity.allFinite() && angular_velocity.allFinite() &&
         orientation.coeffs().allFinite() && orientation.squaredNorm() > 1e-12;
}

Eigen::Matrix3d projectedYaw(const Eigen::Quaterniond & orientation)
{
  const Eigen::Matrix3d rotation = orientation.normalized().toRotationMatrix();
  return Eigen::AngleAxisd(std::atan2(rotation(1, 0), rotation(0, 0)), Eigen::Vector3d::UnitZ())
         .toRotationMatrix();
}

Eigen::Matrix3d Policy::torsoRotation(
  const Eigen::Quaterniond & root, const Joints & position) const
{
  // The exporter checks the training model's pelvis -> torso chain and axes.
  return root.normalized().toRotationMatrix() *
         Eigen::AngleAxisd(position[metadata_.waist_indices[0]], Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(position[metadata_.waist_indices[1]], Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(position[metadata_.waist_indices[2]], Eigen::Vector3d::UnitY());
}

void Policy::load(
  const std::string & model, const std::string & controllers_file, const std::string & motion)
{
  reset_ = false;
  external_reference_ = false;
  // Drop wrappers before resizing buffers, including when reloading 600/609 actors.
  inputs_ = {Ort::Value{nullptr}, Ort::Value{nullptr}};
  output_ = Ort::Value{nullptr};
  const YAML::Node config = YAML::LoadFile(controllers_file)["mimic_model"]["ros__parameters"];
  metadata_.joints = config["joint_names"].as<std::vector<std::string>>();
  metadata_.anchor_obs = config["anchor_obs"].as<bool>();
  const auto target_clip = config["joint_target_clip"].as<std::string>("joint_limits");
  if (target_clip != "joint_limits" && target_clip != "none") {
    throw std::runtime_error("Unsupported SuperMimic joint_target_clip: " + target_clip);
  }
  metadata_.clip_joint_targets = target_clip == "joint_limits";
  metadata_.goal_size = metadata_.anchor_obs ? kLegacyGoalSize : kGoalSize;
  const std::vector<std::string> waist_names{
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"};
  const std::vector<double> waist_axes{0., 0., 1., 1., 0., 0., 0., 1., 0.};
  const bool supported_goal = metadata_.anchor_obs ?
    config["position_feedback"].as<std::string>() == "zero" &&
    config["torso_joint_names"].as<std::vector<std::string>>() == waist_names &&
    config["torso_joint_axes"].as<std::vector<double>>() == waist_axes &&
    config["goal_layout"].as<std::string>() ==
    "10x60_then_zero_position3_then_torso_rotation_first_two_columns_row_major6" :
    config["goal_layout"].as<std::string>() == "future_frame_major" &&
    config["position_feedback"].as<std::string>() == "none";
  const bool supported_layout = supported_goal &&
    config["schema_version"].as<int>() == 2 && config["model_type"].as<std::string>() == "supermimic" &&
    config["action_mode"].as<std::string>() == "default_offset" &&
    config["goal_size"].as<size_t>() == metadata_.goal_size && config["goal_start_offset"].as<int>() == 0 &&
    config["reference_joint_position"].as<std::string>() == "relative_to_default" &&
    config["history_layout"].as<std::string>() == "term_major_oldest_first" &&
    config["normalization"].as<std::string>() == "embedded_in_onnx" &&
    config["input_size"].as<size_t>() == kObservationSize && config["joint_count"].as<size_t>() == kJoints &&
    config["history_frames"].as<size_t>() == kHistory && config["reference_frames"].as<size_t>() == kFuture &&
    config["proprio_width"].as<int>() == 81 && config["reference_width"].as<int>() == 60;
  if (!supported_layout || metadata_.joints.size() != kJoints ||
    std::set<std::string>(metadata_.joints.begin(), metadata_.joints.end()).size() != kJoints ||
    std::any_of(metadata_.joints.begin(), metadata_.joints.end(),
      [](const std::string & name) {return name.empty();}))
  {
    throw std::runtime_error("Unsupported Hunter130 SuperMimic metadata layout");
  }
  for (size_t i = 0; i < waist_names.size(); ++i) {
    const auto found = std::find(metadata_.joints.begin(), metadata_.joints.end(), waist_names[i]);
    if (found == metadata_.joints.end()) {throw std::runtime_error("Missing waist joint: " + waist_names[i]);}
    metadata_.waist_indices[i] = std::distance(metadata_.joints.begin(), found);
  }
  metadata_.default_position = jointVector(config, "stand_position");
  metadata_.kp = jointVector(config, "kp");
  metadata_.kd = jointVector(config, "kd");
  metadata_.scale = jointVector(config, "action_scale");
  // New exports retain checkpoint motor parameters. Reject hand-edited gain or
  // scale arrays that would silently change the actor's training contract.
  if (const auto motors = config["motor_parameters"]; motors && !motors.IsNull()) {
    if (!motors.IsMap() || motors.size() != kJoints) {
      throw std::runtime_error("Expected 25 saved motor_parameters");
    }
    for (size_t j = 0; j < kJoints; ++j) {
      const auto motor = motors[metadata_.joints[j]];
      if (motor["stiffness"].as<double>() != metadata_.kp[j] ||
        motor["damping"].as<double>() != metadata_.kd[j] ||
        motor["action_scale"].as<double>() != metadata_.scale[j])
      {
        throw std::runtime_error("PD/action_scale differ from saved motor_parameters: " + metadata_.joints[j]);
      }
    }
  }
  metadata_.lower = jointVector(config, "lower");
  metadata_.upper = jointVector(config, "upper");
  metadata_.control_dt = config["policy_dt"].as<double>();
  metadata_.stand_height = config["stand_height"].as<double>();
  if (!std::isfinite(metadata_.control_dt) || std::abs(metadata_.control_dt - .02) > 1e-9 ||
    !std::isfinite(metadata_.stand_height) || metadata_.stand_height < .35 || metadata_.stand_height > 1.1 ||
    (metadata_.kp.array() <= 0).any() || (metadata_.kd.array() < 0).any() ||
    (metadata_.scale.array() <= 0).any() || (metadata_.lower.array() >= metadata_.upper.array()).any() ||
    (metadata_.default_position.array() < metadata_.lower.array()).any() ||
    (metadata_.default_position.array() > metadata_.upper.array()).any())
  {
    throw std::runtime_error("Invalid SuperMimic timing, limits or control parameters");
  }
  command_.assign(metadata_.goal_size, 0.f);

  if (!motion.empty()) {
    std::ifstream csv(motion);
    std::string line;
    if (!csv || !std::getline(csv, line)) {throw std::runtime_error("Cannot load motion CSV: " + motion);}
    std::vector<std::string> expected{"time", "root_tx", "root_ty", "root_tz",
      "root_qw", "root_qx", "root_qy", "root_qz"};
    expected.insert(expected.end(), metadata_.joints.begin(), metadata_.joints.end());
    for (const auto & name : metadata_.joints) {expected.push_back("velocity_" + name);}
    for (const auto * name : {"root_vx", "root_vy", "root_vz", "root_wx", "root_wy", "root_wz"}) {
      expected.push_back(name);
    }
    if (split(line) != expected) {throw std::runtime_error("SuperMimic motion CSV header/joint order mismatch");}
    frames_.clear();
    while (std::getline(csv, line)) {
      const auto cells = split(line);
      if (cells.size() != expected.size()) {throw std::runtime_error("Motion CSV row width mismatch");}
      std::array<double, 64> values{};
      for (size_t i = 0; i < cells.size(); ++i) {
        size_t consumed = 0;
        values[i] = std::stod(cells[i], &consumed);
        if (consumed != cells[i].size() || !std::isfinite(values[i])) {
          throw std::runtime_error("Invalid motion CSV number");
        }
      }
      if (std::abs(values[0] - frames_.size() * metadata_.control_dt) > 1e-6) {
        throw std::runtime_error("Motion must be uniformly sampled at 50Hz starting at time zero");
      }
      Frame frame;
      frame.root_position = Eigen::Vector3d(values[1], values[2], values[3]);
      frame.orientation = Eigen::Quaterniond(values[4], values[5], values[6], values[7]);
      if (std::abs(frame.orientation.norm() - 1.) > .001) {
        throw std::runtime_error("Motion quaternion is not unit length");
      }
      frame.orientation.normalize();
      std::copy_n(values.begin() + 8, kJoints, frame.position.data());
      std::copy_n(values.begin() + 33, kJoints, frame.velocity.data());
      frame.linear_velocity = Eigen::Vector3d(values[58], values[59], values[60]);
      frame.angular_velocity = Eigen::Vector3d(values[61], values[62], values[63]);
      if ((frame.position.array() < metadata_.lower.array() - 1e-5).any() ||
        (frame.position.array() > metadata_.upper.array() + 1e-5).any())
      {
        throw std::runtime_error("Reference joint position exceeds robot limits");
      }
      frames_.push_back(frame);
    }
    if (frames_.size() < 2) {throw std::runtime_error("Motion needs at least two frames");}
  } else {
    const Frame standing{Eigen::Vector3d(0, 0, metadata_.stand_height), Eigen::Quaterniond::Identity(),
      metadata_.default_position, Joints::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    frames_.assign(kFuture, standing);
  }

  Ort::SessionOptions options;
  options.SetIntraOpNumThreads(1);
  options.SetInterOpNumThreads(1);
  options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  session_ = Ort::Session(env_, model.c_str(), options);
  if (session_.GetInputCount() != 2 || session_.GetOutputCount() != 1) {
    throw std::runtime_error("Expected two SuperMimic inputs and one output");
  }
  Ort::AllocatorWithDefaultOptions allocator;
  std::set<std::string> inputs;
  for (size_t i = 0; i < 3; ++i) {
    const bool output = i == 2;
    const auto name = output ? session_.GetOutputNameAllocated(0, allocator) :
      session_.GetInputNameAllocated(i, allocator);
    auto type = output ? session_.GetOutputTypeInfo(0) : session_.GetInputTypeInfo(i);
    const auto info = type.GetTensorTypeAndShapeInfo();
    const auto dims = info.GetShape();
    const std::string key(name.get());
    const int width = output ? (key == "action" ? 25 : -1) :
      (key == "policy" ? static_cast<int>(kObservationSize) :
      (key == "goal" ? static_cast<int>(metadata_.goal_size) : -1));
    if (width < 0 || dims.size() != 2 || (dims[0] != -1 && dims[0] != 1) || dims[1] != width ||
      info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
    {
      throw std::runtime_error("ONNX names, shapes or types do not match Hunter130 SuperMimic");
    }
    if (!output) {inputs.insert(key);}
  }
  if (inputs != std::set<std::string>{"policy", "goal"}) {
    throw std::runtime_error("Duplicate or missing SuperMimic input");
  }
  const std::array<int64_t, 2> policy_shape{1, kObservationSize},
    goal_shape{1, static_cast<int64_t>(metadata_.goal_size)}, action_shape{1, kJoints};
  inputs_[0] = Ort::Value::CreateTensor<float>(memory_, observations_.data(),
    observations_.size(), policy_shape.data(), 2);
  inputs_[1] = Ort::Value::CreateTensor<float>(memory_, command_.data(),
    command_.size(), goal_shape.data(), 2);
  output_ = Ort::Value::CreateTensor<float>(memory_, output_buffer_.data(),
    output_buffer_.size(), action_shape.data(), 2);
}

void Policy::setReference(const std::array<Frame, kFuture> & horizon)
{
  for (const auto & frame : horizon) {
    if (!frame.root_position.allFinite() || !frame.position.allFinite() ||
      !frame.velocity.allFinite() || !frame.linear_velocity.allFinite() ||
      !frame.angular_velocity.allFinite() || !frame.orientation.coeffs().allFinite() ||
      std::abs(frame.orientation.norm() - 1.) > .001 ||
      (frame.position.array() < metadata_.lower.array() - 1e-5).any() ||
      (frame.position.array() > metadata_.upper.array() + 1e-5).any())
    {
      throw std::runtime_error("Invalid live SuperMimic reference horizon");
    }
  }
  reference_ = horizon;
  for (auto & frame : reference_) {frame.orientation.normalize();}
  external_reference_ = true;
}

void Policy::reset(const Sample & sample, bool align_heading)
{
  if (!sample.valid() || frames_.empty()) {throw std::runtime_error("Invalid initial robot state");}
  history_.fill(sample);
  actions_.fill(Action{});
  last_action_.fill(0);
  heading_offset_ = Eigen::Matrix3d::Identity();
  const auto & first = external_reference_ ? reference_.front() : frames_.front();
  if (align_heading && metadata_.anchor_obs) {
    heading_offset_ = projectedYaw(first.orientation) * projectedYaw(sample.orientation).transpose();
  }
  applied_ = (first.position - metadata_.default_position).cwiseQuotient(metadata_.scale);
  reset_ = true;
}

void Policy::buildObservation(const Sample & sample, size_t frame_index)
{
  for (size_t h = 1; h < kHistory; ++h) {
    history_[h - 1] = history_[h];
    actions_[h - 1] = actions_[h];
  }
  history_.back() = sample;
  actions_.back() = last_action_;
  size_t cursor = 0;
  for (const auto & state : history_) {
    for (size_t j = 0; j < kJoints; ++j) {
      observations_[cursor++] = static_cast<float>(state.position[j] - metadata_.default_position[j]);
    }
  }
  for (const auto & state : history_) {
    for (size_t j = 0; j < kJoints; ++j) {observations_[cursor++] = static_cast<float>(state.velocity[j]);}
  }
  for (const auto & state : history_) {
    for (int j = 0; j < 3; ++j) {observations_[cursor++] = static_cast<float>(state.angular_velocity[j]);}
  }
  for (const auto & state : history_) {
    const Eigen::Vector3d gravity = state.orientation.normalized().conjugate() * Eigen::Vector3d(0, 0, -1);
    for (int j = 0; j < 3; ++j) {observations_[cursor++] = static_cast<float>(gravity[j]);}
  }
  for (const auto & action : actions_) {
    for (float value : action) {observations_[cursor++] = value;}
  }

  cursor = 0;
  for (size_t offset = 0; offset < kFuture; ++offset) {
    const auto & reference = external_reference_ ? reference_[offset] :
      frames_[std::min(frame_index + offset, frames_.size() - 1)];
    for (size_t j = 0; j < kJoints; ++j) {
      command_[cursor++] = static_cast<float>(reference.position[j] - metadata_.default_position[j]);
    }
    for (size_t j = 0; j < kJoints; ++j) {command_[cursor++] = static_cast<float>(reference.velocity[j]);}
    const Eigen::Vector3d gravity = reference.orientation.conjugate() * Eigen::Vector3d(0, 0, -1);
    const Eigen::Vector3d angular = reference.orientation.conjugate() * reference.angular_velocity;
    const Eigen::Vector3d linear = projectedYaw(reference.orientation).transpose() * reference.linear_velocity;
    for (const auto & value : {gravity, angular, linear}) {
      for (int j = 0; j < 3; ++j) {command_[cursor++] = static_cast<float>(value[j]);}
    }
    command_[cursor++] = static_cast<float>(reference.root_position.z());
  }
  if (metadata_.anchor_obs) {
    // No robot odometry: translation feedback is explicitly ablated in metadata.
    for (int j = 0; j < 3; ++j) {command_[cursor++] = 0.;}
    const auto & reference = external_reference_ ? reference_.front() : frames_[frame_index];
    const Eigen::Matrix3d robot = heading_offset_ * torsoRotation(sample.orientation, sample.position);
    const Eigen::Matrix3d relative = robot.transpose() * torsoRotation(reference.orientation, reference.position);
    // Training uses matrix[:, :2].reshape(6): two columns, flattened row-major.
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 2; ++col) {command_[cursor++] = static_cast<float>(relative(row, col));}
    }
  }
}

Action Policy::infer(const Sample & sample, size_t frame_index)
{
  if (!reset_ || !sample.valid() ||
    (external_reference_ ? frame_index != 0 : frame_index >= frames_.size()))
  {
    throw std::runtime_error("Missing reset, invalid robot state or out-of-range frame");
  }
  buildObservation(sample, frame_index);
  if (!std::all_of(observations_.begin(), observations_.end(), [](float x) {return std::isfinite(x);}) ||
    !std::all_of(command_.begin(), command_.end(), [](float x) {return std::isfinite(x);}))
  {
    throw std::runtime_error("Non-finite SuperMimic observation");
  }
  constexpr std::array<const char *, 2> names{"policy", "goal"};
  constexpr std::array<const char *, 1> outputs{"action"};
  // Input/output types and widths were checked at load. Fixed caller-owned
  // tensors avoid per-tick wrapper, result-vector and output-buffer allocation.
  session_.Run(Ort::RunOptions{nullptr}, names.data(), inputs_.data(), 2,
    outputs.data(), &output_, 1);
  const Action action = output_buffer_;
  if (!std::all_of(action.begin(), action.end(), [](float x) {return std::isfinite(x);})) {
    throw std::runtime_error("Non-finite SuperMimic action");
  }
  last_action_ = action;
  for (size_t j = 0; j < kJoints; ++j) {applied_[j] = action[j];}
  return action;
}

Joints Policy::target() const
{
  if (!reset_) {throw std::runtime_error("SuperMimic policy has not been reset");}
  const Joints target = metadata_.default_position + metadata_.scale.cwiseProduct(applied_);
  if (metadata_.clip_joint_targets) {
    return target.cwiseMax(metadata_.lower).cwiseMin(metadata_.upper);
  }
  return target;
}
}  // namespace ec::mimic
