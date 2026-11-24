#include "ec_controller/AmpPolicy.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace ec::amp
{
namespace
{
template<typename T>
bool finite(const std::vector<T> & values)
{
  return std::all_of(values.begin(), values.end(), [](T value) {return std::isfinite(value);});
}
}  // namespace

void Policy::load(
  const std::string & model, const std::vector<double> & default_position,
  const std::vector<double> & action_scale, size_t history_length)
{
  // Release wrappers before replacing buffers, including on repeated loads.
  input_ = Ort::Value{nullptr};
  output_ = Ort::Value{nullptr};
  session_ = Ort::Session{nullptr};
  history_filled_ = false;
  if (default_position.empty() || default_position.size() != action_scale.size() ||
    history_length == 0 || !finite(default_position) || !finite(action_scale) ||
    std::any_of(action_scale.begin(), action_scale.end(), [](double scale) {return scale <= 0.;}))
  {
    throw std::runtime_error("Invalid AMP default positions, action scales or history length");
  }
  default_position_ = default_position;
  action_scale_ = action_scale;
  history_length_ = history_length;
  const size_t joints = default_position_.size();
  frame_.assign(9 + 3 * joints, 0.f);
  history_.assign(history_length_ * frame_.size(), 0.f);
  actions_.assign(joints, 0.f);
  output_buffer_.assign(joints, 0.f);
  target_ = default_position_;

  Ort::SessionOptions options;
  options.SetIntraOpNumThreads(1);
  Ort::Session session(env_, model.c_str(), options);
  if (session.GetInputCount() != 1 || session.GetOutputCount() != 1) {
    throw std::runtime_error("AMP requires one observation input and one action output");
  }
  const auto input_type = session.GetInputTypeInfo(0);
  const auto output_type = session.GetOutputTypeInfo(0);
  const auto input_info = input_type.GetTensorTypeAndShapeInfo();
  const auto output_info = output_type.GetTensorTypeAndShapeInfo();
  const auto has_shape = [](const std::vector<int64_t> & dims, size_t features) {
      return dims.size() == 2 && (dims[0] == -1 || dims[0] == 1) &&
             dims[1] == static_cast<int64_t>(features);
    };
  if (input_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
    output_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
    !has_shape(input_info.GetShape(), history_.size()) || !has_shape(output_info.GetShape(),
      joints))
  {
    throw std::runtime_error("ONNX shapes/types do not match AMP float32 observations/actions");
  }
  Ort::AllocatorWithDefaultOptions allocator;
  input_name_ = session.GetInputNameAllocated(0, allocator).get();
  output_name_ = session.GetOutputNameAllocated(0, allocator).get();
  const std::array<int64_t, 2> input_shape{1, static_cast<int64_t>(history_.size())};
  const std::array<int64_t, 2> output_shape{1, static_cast<int64_t>(joints)};
  input_ = Ort::Value::CreateTensor<float>(memory_, history_.data(), history_.size(),
      input_shape.data(), 2);
  output_ = Ort::Value::CreateTensor<float>(memory_, output_buffer_.data(), joints,
      output_shape.data(), 2);
  session_ = std::move(session);
}

void Policy::reset()
{
  history_filled_ = false;
  std::fill(actions_.begin(), actions_.end(), 0.f);
  std::fill(history_.begin(), history_.end(), 0.f);
  std::copy(default_position_.begin(), default_position_.end(), target_.begin());
}

void Policy::buildObservation(const Sample & sample)
{
  const size_t n = default_position_.size();
  if (sample.position.size() != n || sample.velocity.size() != n ||
    !finite(sample.position) || !finite(sample.velocity) || !sample.angular_velocity.allFinite() ||
    !sample.command.allFinite() || !sample.orientation.coeffs().allFinite() ||
    !std::isfinite(sample.orientation.squaredNorm()) || sample.orientation.squaredNorm() < 1e-12)
  {
    throw std::runtime_error("Invalid AMP joint, IMU or velocity-command input");
  }
  const auto q = sample.orientation.normalized();
  const double x = q.x(), y = q.y(), z = q.z(), w = q.w();
  for (size_t i = 0; i < 3; ++i) {
    frame_[i] = static_cast<float>(sample.angular_velocity[i]);
    frame_[6 + i] = static_cast<float>(sample.command[i]);
  }
  // R_WB^T * [0, 0, -1], matching the training unit projected gravity.
  frame_[3] = static_cast<float>(-2. * (x * z - w * y));
  frame_[4] = static_cast<float>(-2. * (y * z + w * x));
  frame_[5] = static_cast<float>(-(w * w - x * x - y * y + z * z));
  for (size_t p = 0; p < n; ++p) {
    frame_[9 + p] = static_cast<float>(sample.position[p] - default_position_[p]);
    frame_[9 + n + p] = static_cast<float>(sample.velocity[p]);
    frame_[9 + 2 * n + p] = actions_[p];
  }
  if (!finite(frame_)) {throw std::runtime_error("AMP observation exceeds float32 range");}
}

void Policy::updateHistory()
{
  if (!history_filled_) {
    for (size_t h = 0; h < history_length_; ++h) {
      std::copy(frame_.begin(), frame_.end(), history_.begin() + h * frame_.size());
    }
    history_filled_ = true;
  } else {
    std::memmove(history_.data(), history_.data() + frame_.size(),
      (history_length_ - 1) * frame_.size() * sizeof(float));
    std::copy(frame_.begin(), frame_.end(), history_.end() - frame_.size());
  }
}

void Policy::infer(const Sample & sample)
{
  if (!session_) {throw std::runtime_error("AMP policy is not loaded");}
  buildObservation(sample);
  updateHistory();
  const char * input_name = input_name_.c_str();
  const char * output_name = output_name_.c_str();
  // Reuse tensor wrappers and output storage; keep the previous successful
  // action if inference fails. Raw actions are neither scaled nor clipped here.
  session_.Run(Ort::RunOptions{nullptr}, &input_name, &input_, 1, &output_name, &output_, 1);
  if (!finite(output_buffer_)) {throw std::runtime_error("AMP returned non-finite actions");}
  std::copy(output_buffer_.begin(), output_buffer_.end(), actions_.begin());
  for (size_t p = 0; p < actions_.size(); ++p) {
    target_[p] = default_position_[p] + action_scale_[p] * actions_[p];
  }
}
}  // namespace ec::amp
