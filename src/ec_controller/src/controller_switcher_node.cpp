// =============================================================================
//  Copyright (c) 2026 Phanerox. All rights reserved.
// =============================================================================

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include "ec_controller/ControllerModes.h"
#include <ec_radio/msg/radio.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <map>

#include <memory>
#include <string>
#include <vector>
#include <utility>

using namespace std::chrono_literals;

namespace ec
{

/**
 * @brief Controller Switcher Node
 *
 * 监听遥控器输出，自动切换控制器
 * 常规切换顺序为 Default → Stand → AMP Walk → Mimic 动作或 Quest 跟随 → AMP Walk。
 * 两种 Mimic 模式正常退出时，先等待控制器完成平滑回站。
 */
class ControllerSwitcherNode : public rclcpp::Node {
public:
  explicit ControllerSwitcherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("controller_switcher", options)
  {
    // 声明参数
    this->declare_parameter<std::string>("controller_manager_ns", "/controller_manager");
    this->declare_parameter<std::string>("initial_controller", "default_controller");

    // 获取参数
    controller_manager_ns_ = this->get_parameter("controller_manager_ns").as_string();
    initial_controller_ = this->get_parameter("initial_controller").as_string();
    active_controller_ = initial_controller_;

    // 创建服务客户端
    switch_controller_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
      controller_manager_ns_ + "/switch_controller");
    list_controllers_client_ = this->create_client<controller_manager_msgs::srv::ListControllers>(
      controller_manager_ns_ + "/list_controllers");

    // 订阅遥控器话题
    radio_sub_ = this->create_subscription<ec_radio::msg::Radio>(
      "/radio", 10,
      std::bind(&ControllerSwitcherNode::radioCallback, this, std::placeholders::_1));

    // 订阅控制器切换话题（手动切换）
    default_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/controller_switch/default_controller", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        if (msg->data > 0.5f) {switchController("default_controller");}
      });

    stand_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/controller_switch/stand_controller", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        if (msg->data > 0.5f) {switchController("stand_controller");}
      });

    walk_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/controller_switch/walk_controller", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        if (msg->data > 0.5f) {switchController("walk_controller");}
      });

    mimic_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/controller_switch/mimic_controller", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        if (msg->data > .5f) {switchController("mimic_controller");}
      });
    quest_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/controller_switch/quest_controller", 10,
      [this](std_msgs::msg::Float32::ConstSharedPtr msg) {
        if (msg->data > .5f) {switchController("quest_controller");}
      });
    quest_ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/quest/ready", 1, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        quest_ready_ = msg->data;
        quest_ready_time_ = std::chrono::steady_clock::now();
      });
    quest_stop_pub_ = this->create_publisher<std_msgs::msg::Bool>("/quest/stop", 10);
    quest_state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/quest/state", 10, [this](std_msgs::msg::String::ConstSharedPtr msg) {
        handleState("quest_controller", msg->data);
      });
    mimic_stop_pub_ = this->create_publisher<std_msgs::msg::Bool>("/mimic/stop", 10);
    mimic_state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/mimic/state", 10, [this](std_msgs::msg::String::ConstSharedPtr msg) {
        handleState("mimic_controller", msg->data);
      });

    // 等待服务可用
    RCLCPP_INFO(this->get_logger(), "Waiting for switch_controller service...");
    switch_controller_client_->wait_for_service(30s);

    RCLCPP_INFO(this->get_logger(), "Controller Switcher Node started");
    RCLCPP_INFO(this->get_logger(), "Modes: 1 damping, 2 preparation pose, 3 AMP balance/walk, 4 SuperMimic, 5 Quest full body");
  }

private:
  void handleState(const std::string & controller, const std::string & state)
  {
    controller_states_[controller] = state;
    if (!return_pending_ || active_controller_ != controller) {return;}
    if (state == "fault") {
      return_ready_ = true;  // A fault cannot execute the neural return trajectory.
      switchController("walk_controller");
      return;
    }
    if (state == "returning") {return_observed_ = true;}
    if (state == "standing_after" && return_observed_) {
      return_ready_ = true;
      switchController("walk_controller");
    }
  }
  void radioCallback(const ec_radio::msg::Radio::SharedPtr msg)
  {
    if (msg->mode != last_mode_) {
      last_mode_ = msg->mode;

      switchController(std::string(controllerForMode(msg->mode)));
    }
  }

  bool isTransitionAllowed(const std::string & from, const std::string & to)
  {
    return transitionAllowed(from, to);
  }

  void clearReturn()
  {
    return_pending_ = return_observed_ = return_ready_ = false;
  }

  void finishSwitch()
  {
    switch_in_flight_ = false;
    if (!pending_controller_.empty()) {
      const auto pending = std::exchange(pending_controller_, std::string{});
      switchController(pending);
    }
  }

  // STRICT switching is not transactional: the old controller may already be
  // inactive when the new controller fails. Always query manager state before
  // constructing a request, including before recovery and emergency damping.
  void switchController(const std::string & target, bool allow_recovery = true)
  {
    if (target == "default_controller") {clearReturn();}
    if (switch_in_flight_) {
      if (pending_controller_ != "default_controller") {pending_controller_ = target;}
      return;
    }
    if (!list_controllers_client_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "Cannot query controller states; no switch sent");
      return;
    }
    switch_in_flight_ = true;
    list_controllers_client_->async_send_request(
      std::make_shared<controller_manager_msgs::srv::ListControllers::Request>(),
      [this, target, allow_recovery](rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture result) {
        try {
          std::string actual;
          lifecycle_states_.clear();
          for (const auto & controller : result.get()->controller) {
            const auto & name = controller.name;
            const bool managed = name == "default_controller" || name == "stand_controller" ||
              name == "walk_controller" || name == "mimic_controller" || name == "quest_controller";
            if (managed) {lifecycle_states_[name] = controller.state;}
            if (managed && controller.state == "active") {
              if (!actual.empty()) {throw std::runtime_error("Multiple motion controllers active");}
              actual = name;
            }
          }
          active_controller_ = actual;
          if (actual == "walk_controller") {walk_has_run_ = true;}
          // A damping request arriving during discovery takes priority.
          if (pending_controller_ == "default_controller") {
            pending_controller_.clear();
            executeSwitch("default_controller", false);
          } else {
            executeSwitch(target, allow_recovery);
          }
        } catch (const std::exception & error) {
          RCLCPP_ERROR(get_logger(), "Cannot reconcile controller states: %s", error.what());
          finishSwitch();
        }
      });
  }

  void executeSwitch(const std::string & target, bool allow_recovery)
  {
    if (active_controller_ == target) {
      if (target == "mimic_controller" || target == "quest_controller") {clearReturn();}
      finishSwitch();
      return;
    }
    if (lifecycle_states_[target] != "inactive") {
      RCLCPP_ERROR(get_logger(), "Controller %s is '%s', not configured/inactive; keeping the active controller",
        target.c_str(), lifecycle_states_[target].c_str());
      finishSwitch();
      return;
    }
    const bool no_controller_recovery = active_controller_.empty() &&
      (target == "default_controller" || (target == "walk_controller" && walk_has_run_));
    if (!no_controller_recovery && !isTransitionAllowed(active_controller_, target)) {
      RCLCPP_WARN(get_logger(), "Transition not allowed: %s -> %s. Use Default -> Stand -> AMP Walk -> Mimic or Quest -> AMP Walk",
        active_controller_.c_str(), target.c_str());
      finishSwitch();
      return;
    }
    if (target == "quest_controller" && (!quest_ready_ ||
      std::chrono::duration<double>(std::chrono::steady_clock::now() - quest_ready_time_).count() > .15))
    {
      RCLCPP_INFO(get_logger(), "Quest reference not ready: entering the Quest actor and waiting for valid data");
    }
    const bool leaving_mimic = active_controller_ == "mimic_controller" || active_controller_ == "quest_controller";
    const bool faulted = leaving_mimic && controller_states_[active_controller_] == "fault";
    if (leaving_mimic && target == "walk_controller" && !return_ready_ && !faulted)
    {
      if (!return_pending_) {
        return_pending_ = true;
        return_observed_ = false;
        std_msgs::msg::Bool stop;
        stop.data = true;
        (active_controller_ == "quest_controller" ? quest_stop_pub_ : mimic_stop_pub_)->publish(stop);
        RCLCPP_INFO(get_logger(), "Returning the neural reference before changing controller");
      }
      finishSwitch();
      return;
    }
    if (faulted) {RCLCPP_WARN(get_logger(), "Mimic fault: switching without waiting for neural return");}
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    const auto previous = active_controller_;
    if (!previous.empty()) {request->deactivate_controllers.push_back(previous);}
    request->activate_controllers.push_back(target);
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
    request->timeout = rclcpp::Duration::from_seconds(5.0);
    RCLCPP_INFO(get_logger(), "Switching from '%s' to: %s", previous.c_str(), target.c_str());
    switch_controller_client_->async_send_request(request,
      [this, target, previous, allow_recovery](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture result) {
        bool success = false;
        try {success = result.get()->ok;}
        catch (const std::exception & error) {RCLCPP_ERROR(get_logger(), "Switch service failed: %s", error.what());}
        clearReturn();
        if (success) {
          active_controller_ = target;
          controller_states_.erase(target);
          RCLCPP_INFO(get_logger(), "Switched to: %s", target.c_str());
          finishSwitch();
          return;
        }
        active_controller_.clear();  // Unknown until the next list_controllers response.
        RCLCPP_ERROR(get_logger(), "Failed to switch to %s; previous controller may already be stopped", target.c_str());
        if (allow_recovery && previous == "walk_controller" &&
          (target == "mimic_controller" || target == "quest_controller"))
        {
          const std::string recovery = pending_controller_ == "default_controller" ?
            "default_controller" : "walk_controller";
          pending_controller_.clear();
          switch_in_flight_ = false;
          RCLCPP_WARN(get_logger(), "Reconciling actual controller states and recovering %s", recovery.c_str());
          switchController(recovery, false);  // One recovery attempt, never an automatic retry of Mimic.
          return;
        }
        finishSwitch();
      });
  }

  std::string controller_manager_ns_;
  std::string initial_controller_;

  std::string active_controller_;
  bool walk_has_run_{false};
  std::map<std::string, std::string> controller_states_;
  std::map<std::string, std::string> lifecycle_states_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_controllers_client_;
  uint8_t last_mode_{0};
  bool quest_ready_{false};
  std::chrono::steady_clock::time_point quest_ready_time_{};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr quest_ready_sub_;
  bool switch_in_flight_{false}, return_pending_{false}, return_observed_{false}, return_ready_{false};
  std::string pending_controller_;

  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr
    switch_controller_client_;

  rclcpp::Subscription<ec_radio::msg::Radio>::SharedPtr radio_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr default_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr stand_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr walk_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr mimic_sub_, quest_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr quest_stop_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr quest_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mimic_stop_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mimic_state_sub_;

};

} // namespace ec

#ifndef EC_CONTROLLER_SWITCHER_TEST
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ec::ControllerSwitcherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif
