#include <encos/adapter/base_adapter.h>
#include <encos/bus/bus.h>
#include <encos/device/motor/types.h>
#include <encos/platform/log.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace encos::ec_imu_hardware::test {
namespace {

class TestCanAdapter final : public encos::BaseAdapter {
public:
  TestCanAdapter(const std::string& interface_name, const std::string& logger_name, encos::LogLevel log_level,
                 const std::string& extend = "")
      : BaseAdapter(interface_name, logger_name, log_level), extend_(extend) {
    std::lock_guard<std::mutex> lock(adapters_mutex_);
    adapters_[interface_name] = this;
  }

  ~TestCanAdapter() override {
    std::lock_guard<std::mutex> lock(adapters_mutex_);
    const auto it = adapters_.find(GetInterfaceName());
    if (it != adapters_.end() && it->second == this) {
      adapters_.erase(it);
    }
  }

  std::unordered_map<int, encos::Bus*> GetBuses() override {
    auto buses = GetKnownBusesSnapshot();
    buses.emplace(0, GetBus(0));
    return buses;
  }

  bool UpdateBusExtend(const std::string& extend) override {
    if (reject_updates_) {
      throw std::runtime_error("injected CAN configuration failure");
    }
    extend_ = extend;
    ++update_count_;
    return true;
  }

  const std::string& GetExtend() const { return extend_; }

  int GetUpdateCount() const { return update_count_; }

  void RejectUpdates() { reject_updates_ = true; }

  bool Ok() override { return true; }

  void InjectMessage(const encos::MotorMessage& message) { OnMessage({message}); }

protected:
  void Send(const encos::MotorMessage& /*message*/) override {}

private:
  std::string extend_;
  int update_count_{0};
  bool reject_updates_{false};

  static std::mutex adapters_mutex_;
  static std::unordered_map<std::string, TestCanAdapter*> adapters_;

  friend TestCanAdapter* FindAdapter(const std::string& interface_name);
};

std::mutex TestCanAdapter::adapters_mutex_;
std::unordered_map<std::string, TestCanAdapter*> TestCanAdapter::adapters_;

TestCanAdapter* FindAdapter(const std::string& interface_name) {
  std::lock_guard<std::mutex> lock(TestCanAdapter::adapters_mutex_);
  const auto it = TestCanAdapter::adapters_.find(interface_name);
  return it == TestCanAdapter::adapters_.end() ? nullptr : it->second;
}

}  // namespace
}  // namespace encos::ec_imu_hardware::test

extern "C" __attribute__((visibility("default"))) encos::BaseAdapter* MakeAdapter(const char* interface_name,
                                                                                  const char* logger_name,
                                                                                  int log_level) {
  return new encos::ec_imu_hardware::test::TestCanAdapter(interface_name == nullptr ? "" : interface_name,
                                                          logger_name == nullptr ? "TestCanAdapter" : logger_name,
                                                          encos::LogLevelFromInt(log_level));
}

extern "C" __attribute__((visibility("default"))) encos::BaseAdapter* MakeAdapterV2(const char* interface_name,
                                                                                    const char* logger_name,
                                                                                    int log_level, const char* extend) {
  return new encos::ec_imu_hardware::test::TestCanAdapter(
      interface_name == nullptr ? "" : interface_name, logger_name == nullptr ? "TestCanAdapter" : logger_name,
      encos::LogLevelFromInt(log_level), extend == nullptr ? "" : extend);
}

extern "C" __attribute__((visibility("default"))) const char* EcImuHardwareTestCanPluginGetExtend(
    const char* interface_name) {
  const auto* adapter = encos::ec_imu_hardware::test::FindAdapter(interface_name);
  return adapter == nullptr ? nullptr : adapter->GetExtend().c_str();
}

extern "C" __attribute__((visibility("default"))) int EcImuHardwareTestCanPluginGetUpdateCount(
    const char* interface_name) {
  const auto* adapter = encos::ec_imu_hardware::test::FindAdapter(interface_name);
  return adapter == nullptr ? -1 : adapter->GetUpdateCount();
}

extern "C" __attribute__((visibility("default"))) void EcImuHardwareTestCanPluginRejectUpdates(
    const char* interface_name) {
  if (auto* adapter = encos::ec_imu_hardware::test::FindAdapter(interface_name)) {
    adapter->RejectUpdates();
  }
}

extern "C" __attribute__((visibility("default"))) bool EcImuHardwareTestCanPluginInjectRawMessage(
    const char* interface_name, int bus_idx, std::uint32_t can_id, const std::uint8_t* data, std::size_t size) {
  if (interface_name == nullptr || data == nullptr || size > sizeof(encos::MotorPackMsg::data)) {
    return false;
  }

  auto* adapter = encos::ec_imu_hardware::test::FindAdapter(interface_name);
  if (adapter == nullptr) {
    return false;
  }

  encos::MotorMessage message{};
  message.bus_idx = bus_idx;
  message.data.id = can_id;
  message.data.frame_flags = encos::kCanFrameFlagEff;
  message.data.len = static_cast<std::uint8_t>(size);
  std::copy(data, data + size, message.data.data);
  adapter->InjectMessage(message);
  return true;
}
