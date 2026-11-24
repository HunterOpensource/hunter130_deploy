#ifndef EC_CONTROLLER_AMP_POLICY_H_
#define EC_CONTROLLER_AMP_POLICY_H_

#include <array>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <onnxruntime_cxx_api.h>

namespace ec::amp
{
// All joint vectors use policy order. IMU values are in the pelvis frame.
struct Sample
{
  std::vector<double> position, velocity;
  Eigen::Quaterniond orientation {Eigen::Quaterniond::Identity()};
  Eigen::Vector3d angular_velocity {Eigen::Vector3d::Zero()};
  Eigen::Vector3d command {Eigen::Vector3d::Zero()};
};

// Owns the AMP contract and inference buffers, with no ROS or hardware access.
// One control thread owns each instance; load/reset are lifecycle operations.
class Policy
{
public:
  Policy() = default;
  Policy(const Policy &) = delete;
  Policy & operator=(const Policy &) = delete;
  // Tensor wrappers point into this object's buffers.
  Policy(Policy &&) = delete;
  Policy & operator=(Policy &&) = delete;

  void load(
    const std::string & model, const std::vector<double> & default_position,
    const std::vector<double> & action_scale, size_t history_length);
  void reset();
  void infer(const Sample & sample);
  bool initialized() const {return history_filled_;}
  const std::vector<float> & actions() const
  {
    return actions_;
  }
  const std::vector<float> & observations() const
  {
    return history_;
  }
  const std::vector<double> & target() const
  {
    return target_;
  }

private:
  void buildObservation(const Sample & sample);
  void updateHistory();
  size_t history_length_ {0};
  bool history_filled_ {false};
  std::vector<double> default_position_, action_scale_, target_;
  std::vector<float> frame_, history_, actions_, output_buffer_;
  Ort::Env env_ {ORT_LOGGING_LEVEL_WARNING, "Hunter130AMP"};
  Ort::Session session_ {nullptr};
  Ort::MemoryInfo memory_ {Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)};
  std::string input_name_, output_name_;
  Ort::Value input_ {nullptr}, output_ {nullptr};
};
}  // namespace ec::amp
#endif
