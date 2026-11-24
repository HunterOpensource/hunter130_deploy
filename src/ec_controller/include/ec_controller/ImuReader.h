#ifndef EC_CONTROLLER_IMU_READER_H_
#define EC_CONTROLLER_IMU_READER_H_

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <hardware_interface/loaned_state_interface.hpp>

namespace ec
{
// Bind once after interfaces are assigned on every activation. The update path
// reads seven cached indices, with no string matching or allocation.
class ImuReader
{
public:
  inline static const std::array<std::string, 7> fields {
    "orientation.w", "orientation.x", "orientation.y", "orientation.z",
    "angular_velocity.x", "angular_velocity.y", "angular_velocity.z"
  };

  void setPitchOffset(double degrees)
  {
    if (!std::isfinite(degrees)) {throw std::runtime_error("IMU pitch offset must be finite");}
    correction_ = Eigen::Quaterniond(Eigen::AngleAxisd(
      degrees * std::acos(-1.) / 180., Eigen::Vector3d::UnitY()));
  }

  void bind(const std::vector<hardware_interface::LoanedStateInterface> & interfaces)
  {
    bound_ = false;
    indices_.fill(std::numeric_limits<size_t>::max());
    for (size_t i = 0; i < interfaces.size(); ++i) {
      if (interfaces[i].get_prefix_name() != "imu_sensor") {continue;}
      for (size_t j = 0; j < fields.size(); ++j) {
        if (interfaces[i].get_interface_name() != fields[j]) {continue;}
        if (indices_[j] != std::numeric_limits<size_t>::max()) {
          throw std::runtime_error("Duplicate IMU interface: " + fields[j]);
        }
        indices_[j] = i;
      }
    }
    for (size_t j = 0; j < fields.size(); ++j) {
      if (indices_[j] == std::numeric_limits<size_t>::max()) {
        throw std::runtime_error("Missing IMU interface: " + fields[j]);
      }
    }
    interface_count_ = interfaces.size();
    bound_ = true;
  }

  bool read(
    const std::vector<hardware_interface::LoanedStateInterface> & interfaces,
    Eigen::Quaterniond & orientation, Eigen::Vector3d & angular_velocity) const
  {
    if (!bound_ || interfaces.size() != interface_count_) {return false;}
    std::array<double, 7> values;
    for (size_t i = 0; i < values.size(); ++i) {
      const auto value = interfaces[indices_[i]].get_optional();
      if (!value || !std::isfinite(*value)) {return false;}
      values[i] = *value;
    }
    const Eigen::Quaterniond raw(values[0], values[1], values[2], values[3]);
    if (!std::isfinite(raw.squaredNorm()) || raw.squaredNorm() < 1e-12) {return false;}
    // Driver already supplies pelvis rad/s. Keep local vectors and orientation
    // consistent when applying the optional local +Y mounting correction.
    orientation = (raw.normalized() * correction_).normalized();
    angular_velocity = correction_.conjugate() * Eigen::Vector3d(values[4], values[5], values[6]);
    return orientation.coeffs().allFinite() && angular_velocity.allFinite();
  }

private:
  bool bound_ {false};
  size_t interface_count_ {0};
  std::array<size_t, 7> indices_ {};
  Eigen::Quaterniond correction_ {Eigen::Quaterniond::Identity()};
};
}  // namespace ec
#endif
