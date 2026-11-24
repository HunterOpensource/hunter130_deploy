#include "ec_imu_hardware/ec_imu_hardware.hpp"

#include <encos/bus/bus.h>
#include <encos/encos_driver.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <string_view>
#include <thread>

namespace encos::ec_imu_hardware {
namespace {

constexpr std::size_t kImuStateInterfaceCount = 10;
constexpr std::array<std::string_view, kImuStateInterfaceCount> kImuStateInterfaceNames = {
    "orientation.x",         "orientation.y",         "orientation.z",      "orientation.w",
    "angular_velocity.x",    "angular_velocity.y",    "angular_velocity.z", "linear_acceleration.x",
    "linear_acceleration.y", "linear_acceleration.z",
};
constexpr auto kActivationPollDelay = std::chrono::milliseconds{5};
constexpr auto kInitialFeedbackTimeout = std::chrono::milliseconds{1000};
constexpr double kDegreesToRadians = 3.141592653589793238462643383279502884 / 180.0;
constexpr double kMinQuaternionNorm = 1.0e-12;
constexpr int kMaxImuIndex = std::numeric_limits<std::uint16_t>::max();
constexpr int kMaxSlaveIndex = std::numeric_limits<int>::max() >> 16;
constexpr int kMaxBusIndex = 0xFF;

bool IsEthercatAdapter(const std::string& adapter_type) {
  return adapter_type == "Ethercat" || adapter_type == "EthercatIGH";
}

bool ConvertStatusToState(const encos::ImuStatus& status, std::array<double, kImuStateInterfaceCount>& state) {
  if (!status.quaternion || !status.angular_velocity || !status.acceleration) {
    return false;
  }

  Eigen::Quaterniond orientation{status.quaternion->qw, status.quaternion->qx, status.quaternion->qy,
                                 status.quaternion->qz};
  const double norm = orientation.norm();
  if (!orientation.coeffs().allFinite() || !std::isfinite(norm) || norm <= kMinQuaternionNorm) {
    return false;
  }
  orientation.normalize();

  Eigen::Matrix<double, kImuStateInterfaceCount, 1> candidate;
  candidate << orientation.x(), orientation.y(), orientation.z(), orientation.w(),
      static_cast<double>(status.angular_velocity->x) * kDegreesToRadians,
      static_cast<double>(status.angular_velocity->y) * kDegreesToRadians,
      static_cast<double>(status.angular_velocity->z) * kDegreesToRadians, static_cast<double>(status.acceleration->x),
      static_cast<double>(status.acceleration->y), static_cast<double>(status.acceleration->z);
  if (!candidate.allFinite()) {
    return false;
  }

  std::copy_n(candidate.data(), state.size(), state.begin());
  return true;
}

}  // namespace

CallbackReturn EcImuHardware::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SensorInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  adapter_ = nullptr;
  imu_ = nullptr;
  configured_ = false;
  active_ = false;
  ResetState();

  try {
    if (info_.sensors.size() != 1) {
      return CallbackReturn::ERROR;
    }

    const auto& sensor = info_.sensors.front();
    sensor_name_ = sensor.name;
    adapter_type_ = info_.hardware_parameters.at("adapter_type");
    interface_name_ = info_.hardware_parameters.at("interface_name");
    can_bitrate_.reset();
    can_sample_point_.reset();
    const auto bitrate = info_.hardware_parameters.find("can_bitrate");
    if (bitrate != info_.hardware_parameters.end()) {
      std::size_t consumed = 0;
      const int value = std::stoi(bitrate->second, &consumed, 10);
      if (consumed != bitrate->second.size() || value <= 0) {
        RCLCPP_ERROR(logger_, "invalid IMU '%s' can_bitrate '%s': expected a positive int-range integer in bit/s",
                     sensor_name_.c_str(), bitrate->second.c_str());
        return CallbackReturn::ERROR;
      }
      can_bitrate_ = value;
    }
    const auto sample_point = info_.hardware_parameters.find("can_sample_point");
    if (sample_point != info_.hardware_parameters.end()) {
      std::size_t consumed = 0;
      const double value = std::stod(sample_point->second, &consumed);
      if (consumed != sample_point->second.size() || !std::isfinite(value) || value <= 0.0 || value > 1.0) {
        RCLCPP_ERROR(logger_, "invalid IMU '%s' can_sample_point '%s': expected a finite number in (0, 1]",
                     sensor_name_.c_str(), sample_point->second.c_str());
        return CallbackReturn::ERROR;
      }
      can_sample_point_ = value;
    }
    imu_idx_ = std::stoi(sensor.parameters.at("imu_idx"));
    slave_idx_ = 0;
    bus_idx_ = 0;
    if (IsEthercatAdapter(adapter_type_)) {
      slave_idx_ = std::stoi(sensor.parameters.at("slave_idx"));
      bus_idx_ = std::stoi(sensor.parameters.at("bus_idx"));
    }
    if (imu_idx_ < 0 || imu_idx_ > kMaxImuIndex ||
        (IsEthercatAdapter(adapter_type_) &&
         (slave_idx_ < 0 || slave_idx_ > kMaxSlaveIndex || bus_idx_ < 0 || bus_idx_ > kMaxBusIndex))) {
      return CallbackReturn::ERROR;
    }
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(logger_, "invalid IMU '%s' configuration: %s", sensor_name_.c_str(), ex.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn EcImuHardware::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  configured_ = false;
  active_ = false;
  adapter_ = nullptr;
  imu_ = nullptr;
  ResetState();

  const std::string bus_key =
      IsEthercatAdapter(adapter_type_) ? std::to_string(slave_idx_) + ":" + std::to_string(bus_idx_) : "0";
  try {
    nlohmann::json extend = nlohmann::json::object();
    if (can_bitrate_) {
      extend["buses"][bus_key]["bitrate"] = *can_bitrate_;
    }
    if (can_sample_point_) {
      extend["buses"][bus_key]["sample_point"] = *can_sample_point_;
    }
    adapter_ = encos::MakeAdapter(adapter_type_, interface_name_, logger_.get_name(), encos::LogLevel::Info, extend);
    encos::Bus* bus =
        adapter_ ? (IsEthercatAdapter(adapter_type_) ? adapter_->GetBus(slave_idx_, bus_idx_) : adapter_->GetBus(0))
                 : nullptr;
    imu_ = bus ? bus->GetImu(imu_idx_) : nullptr;
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(logger_, "failed to configure IMU '%s' on interface '%s', bus %s: %s", sensor_name_.c_str(),
                 interface_name_.c_str(), bus_key.c_str(), ex.what());
    adapter_ = nullptr;
    return CallbackReturn::ERROR;
  }

  if (!imu_) {
    RCLCPP_ERROR(logger_, "failed to configure IMU '%s' on interface '%s', bus %s: driver returned no device",
                 sensor_name_.c_str(), interface_name_.c_str(), bus_key.c_str());
    return CallbackReturn::ERROR;
  }

  configured_ = true;
  return CallbackReturn::SUCCESS;
}

CallbackReturn EcImuHardware::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  if (!configured_) {
    RCLCPP_ERROR(logger_, "IMU hardware is not configured");
    return CallbackReturn::ERROR;
  }

  active_ = false;
  ResetState();
  const auto deadline = std::chrono::steady_clock::now() + kInitialFeedbackTimeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (RefreshState()) {
      active_ = true;
      return CallbackReturn::SUCCESS;
    }
    std::this_thread::sleep_for(kActivationPollDelay);
  }

  RCLCPP_ERROR(logger_, "IMU feedback unavailable during activation");
  return CallbackReturn::ERROR;
}

CallbackReturn EcImuHardware::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  active_ = false;
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> EcImuHardware::export_state_interfaces() {
  static_assert(kStateInterfaceCount == kImuStateInterfaceCount);

  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(kStateInterfaceCount);
  for (std::size_t index = 0; index < kStateInterfaceCount; ++index) {
    state_interfaces.emplace_back(sensor_name_, std::string{kImuStateInterfaceNames[index]}, &state_.at(index));
  }
  return state_interfaces;
}

hardware_interface::return_type EcImuHardware::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  if (!active_) {
    return hardware_interface::return_type::OK;
  }
  RefreshState();
  return hardware_interface::return_type::OK;
}

bool EcImuHardware::RefreshState() {
  if (!imu_) {
    return false;
  }

  std::array<double, kStateInterfaceCount> candidate{};
  if (!ConvertStatusToState(imu_->GetStatus(), candidate)) {
    return false;
  }

  state_ = candidate;
  return true;
}

void EcImuHardware::ResetState() {
  state_.fill(std::numeric_limits<double>::quiet_NaN());
}

}  // namespace encos::ec_imu_hardware

PLUGINLIB_EXPORT_CLASS(encos::ec_imu_hardware::EcImuHardware, hardware_interface::SensorInterface)
