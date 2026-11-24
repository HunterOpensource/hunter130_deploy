#include "ec_controller/ControllerBase.h"
#include "ec_controller/MimicPolicy.h"
#include "ec_controller/MimicWorker.h"
#include "ec_controller/ImuReader.h"
#include "ec_controller/MimicHandover.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <ec_teleop_msgs/msg/quest_reference.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <unordered_map>

namespace ec
{
class MimicController : public ControllerBase
{
public:
  ~MimicController() override {if (worker_) {worker_->stop();}}

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override
  {
    if (ControllerBase::on_configure(previous_state) != controller_interface::CallbackReturn::SUCCESS) {
      return controller_interface::CallbackReturn::ERROR;
    }
    auto node = get_node();
    node->declare_parameter<std::string>("policy_path", "config/policy/mimic.onnx");
    node->declare_parameter<std::string>("controllers_file", "config/policy/mimic.yaml");
    node->declare_parameter<std::string>("motion_path", "config/motions/disco.csv");
    node->declare_parameter<double>("preparation_duration", 3.0);
    node->declare_parameter<double>("stand_before_duration", 1.0);
    node->declare_parameter<double>("return_duration", 3.0);
    node->declare_parameter<std::vector<double>>("stand_pos", std::vector<double>{});
    // Zero selects the model's standing height, avoiding a second stale default.
    node->declare_parameter<double>("stand_height", 0.0);
    // History-based handover is opt-in: some hardware/controller-manager paths
    // have no valid last-command snapshot when activation takes place.
    node->declare_parameter<double>("activation_duration", 0.0);
    node->declare_parameter<double>("start_position_tolerance", 0.35);
    node->declare_parameter<double>("imu_pitch_offset_deg", 0.0);
    node->declare_parameter<std::string>("log_dir", "/home/milkyway/log/hunter130");
    preparation_duration_ = node->get_parameter("preparation_duration").as_double();
    stand_before_duration_ = node->get_parameter("stand_before_duration").as_double();
    return_duration_ = node->get_parameter("return_duration").as_double();
    start_tolerance_ = node->get_parameter("start_position_tolerance").as_double();
    stand_height_ = node->get_parameter("stand_height").as_double();
    activation_duration_ = node->get_parameter("activation_duration").as_double();
    imu_pitch_offset_deg_ = node->get_parameter("imu_pitch_offset_deg").as_double();
    log_dir_ = node->get_parameter("log_dir").as_string();
    const auto resolve = [](const std::string & path) {
        return std::filesystem::path(path).is_absolute() ? path :
               ament_index_cpp::get_package_share_directory("ec_controller") + "/" + path;
      };
    try {
      if (!std::isfinite(imu_pitch_offset_deg_)) {
        throw std::runtime_error("imu_pitch_offset_deg must be finite");
      }
      imu_.setPitchOffset(imu_pitch_offset_deg_);
      if (!std::isfinite(preparation_duration_) || preparation_duration_ < .5 ||
        !std::isfinite(stand_before_duration_) || stand_before_duration_ < .5 ||
        !std::isfinite(return_duration_) || return_duration_ < .5 ||
        !std::isfinite(start_tolerance_) || start_tolerance_ <= 0 || start_tolerance_ > .5 ||
        !std::isfinite(activation_duration_) || activation_duration_ < 0 || activation_duration_ > 1. ||
        !std::isfinite(stand_height_) || (stand_height_ != 0 && (stand_height_ < .35 || stand_height_ > 1.1)))
      {
        throw std::runtime_error("Transitions >=0.5s; activation [0,1]s; tolerance (0,0.5]rad; height 0 (model) or [0.35,1.1]m");
      }
      worker_.reset();  // Join before replacing the policy it references.
      policy_ = std::make_unique<mimic::Policy>();
      model_path_ = resolve(node->get_parameter("policy_path").as_string());
      controllers_file_ = resolve(node->get_parameter("controllers_file").as_string());
      policy_->load(model_path_, controllers_file_,
        liveReference() ? "" : resolve(node->get_parameter("motion_path").as_string()));
      worker_ = std::make_unique<mimic::Worker>(*policy_);
      if (stand_height_ == 0) {stand_height_ = policy_->metadata().stand_height;}
      const auto & names = policy_->metadata().joints;
      if (jointCount() != mimic::kJoints) {throw std::runtime_error("SuperMimic requires 25 joints");}
      std::vector<bool> matched(mimic::kJoints, false);
      for (size_t i = 0; i < jointCount(); ++i) {
        const auto found = std::find(names.begin(), names.end(), joint_names_[i]);
        if (found == names.end() || matched[std::distance(names.begin(), found)]) {
          throw std::runtime_error("Duplicate or unknown controller joint: " + joint_names_[i]);
        }
        joint_to_policy_[i] = static_cast<size_t>(std::distance(names.begin(), found));
        matched[joint_to_policy_[i]] = true;
      }
      stand_position_ = policy_->metadata().default_position;
      const auto stand = node->get_parameter("stand_pos").as_double_array();
      if (!stand.empty()) {
        if (stand.size() != mimic::kJoints) {throw std::runtime_error("stand_pos requires 25 values");}
        for (size_t j = 0; j < mimic::kJoints; ++j) {stand_position_[joint_to_policy_[j]] = stand[j];}
      }
      for (size_t j = 0; j < mimic::kJoints; ++j) {
        if (!std::isfinite(stand_position_[j]) ||
          stand_position_[j] < policy_->metadata().lower[j] ||
          stand_position_[j] > policy_->metadata().upper[j])
        {
          throw std::runtime_error("Standing pose exceeds robot limits: " + names[j] + "=" +
            std::to_string(stand_position_[j]) + ", allowed [" +
            std::to_string(policy_->metadata().lower[j]) + ", " +
            std::to_string(policy_->metadata().upper[j]) + "]");
        }
      }
      standing_reference_ = mimic::Frame{};
      standing_reference_.position = stand_position_;
      standing_reference_.root_position.z() = stand_height_;
      if ((stand_position_ - policy_->metadata().default_position).cwiseAbs().maxCoeff() > 1e-8 ||
        std::abs(stand_height_ - policy_->metadata().stand_height) > 1e-8)
      {
        RCLCPP_WARN(logger(), "Standing reference overrides the model default; verify this pose and height in simulation");
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(logger(), "SuperMimic configuration failed: %s", error.what());
      return controller_interface::CallbackReturn::ERROR;
    }
    if (liveReference()) {
      reference_subscription_ = node->create_subscription<ec_teleop_msgs::msg::QuestReference>(
        "/quest/reference", rclcpp::QoS(1).best_effort(),
        [this](ec_teleop_msgs::msg::QuestReference::ConstSharedPtr message) {receiveReference(*message);});
    }
    const std::string topic = liveReference() ? "/quest" : "/mimic";
    stop_subscription_ = node->create_subscription<std_msgs::msg::Bool>(
      topic + "/stop", 10, [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        if (message->data && active_.load()) {stop_requested_.store(true);}
      });
    if (liveReference()) {ready_publisher_ = node->create_publisher<std_msgs::msg::Bool>("/quest/ready", 1);}
    state_publisher_ = node->create_publisher<std_msgs::msg::String>(topic + "/state", 10);
    state_timer_ = node->create_wall_timer(std::chrono::milliseconds(50), [this]() {
        std_msgs::msg::String state;
        if (!active_.load()) {state.data = "inactive";}
        else {
          switch (phase_.load()) {
            case Phase::StandingBefore: state.data = "standing_before"; break;
            case Phase::Preparing: state.data = "preparing"; break;
            case Phase::Tracking: state.data = "tracking"; break;
            case Phase::Paused: state.data = "paused"; break;
            case Phase::Returning: state.data = "returning"; break;
            case Phase::StandingAfter: state.data = "standing_after"; break;
            case Phase::Fault: state.data = "fault"; break;
          }
        }
        state_publisher_->publish(state);
        if (ready_publisher_) {
          std_msgs::msg::Bool ready;
          std::lock_guard<std::mutex> lock(reference_mutex_);
          ready.data = reference_.valid && reference_.enabled && fresh(reference_);
          ready_publisher_->publish(ready);
        }
      });
    RCLCPP_INFO(logger(), "Loaded Hunter130 SuperMimic: 810+%zu inputs, 25 actions, %zu motion frames",
      policy_->metadata().goal_size, policy_->frameCount());
    RCLCPP_INFO(logger(), "Model %s, configuration %s, activation handover %.3fs",
      model_path_.c_str(), controllers_file_.c_str(), activation_duration_);
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    auto interfaces = ControllerBase::state_interface_configuration();
    if (activation_duration_ > 0) {
      for (const auto & name : joint_names_) {
        for (const auto * field : {"position", "velocity", "effort", "kp", "kd", "torque_mode", "age"}) {
          interfaces.names.push_back(name + "/last_command_" + field);
        }
      }
    }
    for (const auto & name : ImuReader::fields) {interfaces.names.push_back("imu_sensor/" + name);}
    return interfaces;
  }

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override
  {
    if (ControllerBase::on_activate(previous_state) != controller_interface::CallbackReturn::SUCCESS) {
      return controller_interface::CallbackReturn::ERROR;
    }
    std::unordered_map<std::string, size_t> indices;
    bool waiting_for_reference = false;
    for (size_t i = 0; i < state_interfaces_.size(); ++i) {indices[state_interfaces_[i].get_name()] = i;}
    try {
      for (size_t i = 0; i < mimic::kJoints; ++i) {
        const auto & name = policy_->metadata().joints[i];
        position_indices_[i] = indices.at(name + "/position");
        velocity_indices_[i] = indices.at(name + "/velocity");
      }
      imu_.bind(state_interfaces_);
      // Hardware clears command slots after write. Read its separate record of
      // successfully submitted PVT commands, never those cleared command slots.
      const auto read_command = [&](const std::string & key) {
          const auto value = state_interfaces_[indices.at(key)].get_optional();
          if (!value || !std::isfinite(*value)) {throw std::runtime_error("Invalid preceding command: " + key);}
          return *value;
        };
      initial_command_ = mimic::PdCommand{};
      initial_command_.kp = policy_->metadata().kp;
      initial_command_.kd = policy_->metadata().kd;
      for (size_t j = 0; j < mimic::kJoints; ++j) {
        const auto & name = policy_->metadata().joints[j];
        if (activation_duration_ == 0) {
          // Until the first actor result, use current feedback and model PD.
          // No predecessor command or command-history state is required.
          initial_command_.position[j] = std::clamp(read_command(name + "/position"),
            policy_->metadata().lower[j], policy_->metadata().upper[j]);
          continue;
        }
        initial_command_.position[j] = read_command(name + "/last_command_position");
        initial_command_.velocity[j] = read_command(name + "/last_command_velocity");
        initial_command_.effort[j] = read_command(name + "/last_command_effort");
        initial_command_.kp[j] = read_command(name + "/last_command_kp");
        initial_command_.kd[j] = read_command(name + "/last_command_kd");
        const double mode = read_command(name + "/last_command_torque_mode");
        const double age = read_command(name + "/last_command_age");
        if (age < 0 || age > .1) {throw std::runtime_error("Preceding command is stale; enter Walk first: " + name);}
        if (initial_command_.kp[j] < 0 || initial_command_.kd[j] < 0 ||
          initial_command_.position[j] < policy_->metadata().lower[j] ||
          initial_command_.position[j] > policy_->metadata().upper[j] ||
          ((initial_command_.kp[j] > 0 || initial_command_.kd[j] > 0 || initial_command_.effort[j] != 0) &&
          mode != (isCoupledJoint(name) ? 2. : 0.)))
        {
          throw std::runtime_error("Cannot hand over invalid command or different torque mode: " + name);
        }
      }
      if (liveReference()) {
        std::lock_guard<std::mutex> lock(reference_mutex_);
        waiting_for_reference = !reference_.valid || !reference_.enabled || !fresh(reference_);
        live_reference_ = reference_;
        live_session_ = waiting_for_reference ? 0 : reference_.session;
      }
      worker_->start();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(logger(), "SuperMimic activation failed: %s", error.what());
      return controller_interface::CallbackReturn::ERROR;
    }
    phase_ = waiting_for_reference ? Phase::Paused : Phase::StandingBefore;
    stop_requested_ = false;
    active_ = true;
    first_update_ = true;
    job_pending_ = false;
    pending_elapsed_ = 0;
    phase_elapsed_ = tracking_elapsed_ = inference_elapsed_ = 0;
    policy_reset_needed_ = true;
    policy_target_ready_ = false;
    activation_elapsed_ = 0;
    reference_generation_ = 0;
    standing_reference_.orientation = Eigen::Quaterniond::Identity();
    standing_reference_.root_position.head<2>().setZero();
    current_reference_ = transition_start_ = standing_reference_;
    reference_heading_ = Eigen::Matrix3d::Identity();
    reference_translation_.setZero();
    if (waiting_for_reference) {
      RCLCPP_INFO(logger(), "Quest mode active: standing while waiting for fresh calibrated references; tracking remains disabled");
    }
    if (!log_dir_.empty()) {
      std::error_code error;
      std::filesystem::create_directories(log_dir_, error);
      if (!error) {
        const auto stamp = std::chrono::system_clock::now().time_since_epoch().count();
        log_.open(std::filesystem::path(log_dir_) / ("mimic_log_" + std::to_string(stamp) + ".csv"));
        if (log_) {
          log_ << std::setprecision(17) << "time_ns,motion_time_s,frame";
          for (const char * field : {"position", "velocity", "action", "target"}) {
            for (const auto & name : policy_->metadata().joints) {log_ << ',' << field << '_' << name;}
          }
          log_ << ",policy_phase,imu_qx,imu_qy,imu_qz,imu_qw,imu_wx,imu_wy,imu_wz,gravity_x,gravity_y,gravity_z,imu_pitch_offset_deg";
          log_ << ",input_time_ns,activation_weight,input_qx,input_qy,input_qz,input_qw,input_wx,input_wy,input_wz,reference_qx,reference_qy,reference_qz,reference_qw";
          for (const char * field : {"input_position", "input_velocity", "reference_position", "actor_target", "applied_kp", "applied_kd"}) {
            for (const auto & name : policy_->metadata().joints) {log_ << ',' << field << '_' << name;}
          }
          log_ << '\n';
          const auto vector = [](const mimic::Joints & values) {
              return std::vector<double>(values.data(), values.data() + mimic::kJoints);
            };
          const auto & meta = policy_->metadata();
          const nlohmann::json manifest{
            {"model_path", model_path_}, {"controllers_file", controllers_file_},
            {"goal_size", meta.goal_size}, {"joint_names", meta.joints}, {"kp", vector(meta.kp)},
            {"kd", vector(meta.kd)}, {"action_scale", vector(meta.scale)},
            {"joint_target_clip", meta.clip_joint_targets ? "joint_limits" : "none"},
            {"stand_position", vector(stand_position_)}, {"stand_height", stand_height_},
            {"imu_pitch_offset_deg", imu_pitch_offset_deg_}, {"activation_duration", activation_duration_},
            {"initial_target", vector(initial_command_.position)}, {"initial_kp", vector(initial_command_.kp)},
            {"initial_kd", vector(initial_command_.kd)}};
          std::ofstream manifest_file(std::filesystem::path(log_dir_) / ("mimic_log_" + std::to_string(stamp) + ".json"));
          manifest_file << manifest.dump(2) << '\n';
          if (!manifest_file) {RCLCPP_WARN(logger(), "Could not write SuperMimic log metadata");}
        }
      }
      if (!log_) {RCLCPP_WARN(logger(), "Could not open SuperMimic CSV log in %s", log_dir_.c_str());}
    }
    RCLCPP_INFO(logger(), "SuperMimic neural standing: stand %.2fs -> prepare %.2fs -> play -> return %.2fs -> stand",
      stand_before_duration_, preparation_duration_, return_duration_);
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override
  {
    worker_->stop();
    log_.close();
    log_.clear();
    active_ = false;
    stop_requested_ = false;
    phase_ = Phase::StandingBefore;
    return ControllerBase::on_deactivate(previous_state);
  }

protected:
  virtual bool liveReference() const {return false;}

  void onUpdate(const rclcpp::Time & time, const rclcpp::Duration & period) override
  {
    mimic::Sample sample;
    if (phase_ == Phase::Fault) {damping(); return;}
    try {
      const auto read = [this](size_t index) {
          const auto value = state_interfaces_[index].get_optional();
          if (!value || !std::isfinite(*value)) {throw std::runtime_error("Invalid joint/IMU feedback");}
          return *value;
        };
      for (size_t j = 0; j < mimic::kJoints; ++j) {
        sample.position[j] = read(position_indices_[j]);
        sample.velocity[j] = read(velocity_indices_[j]);
      }
      if (!imu_.read(state_interfaces_, sample.orientation, sample.angular_velocity)) {
        throw std::runtime_error("Invalid IMU feedback");
      }
      const Eigen::Vector3d projected_gravity = sample.orientation.conjugate() * Eigen::Vector3d(0, 0, -1);
      const double dt = period.seconds();
      if (!sample.valid() || !std::isfinite(dt) || dt <= 0 || dt > .1) {
        throw std::runtime_error("Invalid orientation or control interval");
      }
      sample.orientation.normalize();
      if (first_update_) {
        last_policy_target_ = sample.position;
        first_update_ = false;
      }
      if (job_pending_) {
        pending_elapsed_ += dt;
        if (pending_elapsed_ > policy_->metadata().control_dt) {
          throw std::runtime_error("SuperMimic inference exceeded the 20ms policy deadline");
        }
      }
      PolicyResult result;
      if (policy_target_ready_) {activation_elapsed_ += dt;}
      const bool ready = worker_->takeResult(result);
      if (ready) {
        job_pending_ = false;
        pending_elapsed_ = 0;
      }
      bool inferred = false;
      mimic::Action action{};
      size_t frame_index = 0;
      if (ready) {
        last_policy_target_ = result.target;
        policy_target_ready_ = true;
        action = result.action;
        frame_index = result.frame;
        inferred = true;
      }
      if (liveReference()) {
        std::unique_lock<std::mutex> lock(reference_mutex_, std::try_to_lock);
        if (lock.owns_lock()) {live_reference_ = reference_;}
        const bool usable = live_reference_.valid && live_reference_.enabled && fresh(live_reference_);
        if ((!usable || (live_reference_.session != live_session_ && phase_ != Phase::Paused)) &&
          phase_ != Phase::Returning && phase_ != Phase::StandingAfter)
        {
          if (phase_ != Phase::Paused) {
            beginReturn(Phase::Paused);
            RCLCPP_WARN(logger(), "Quest reference interrupted; neural standing until fresh calibrated data resumes");
          }
        } else if (usable && phase_ == Phase::Paused) {
          live_session_ = live_reference_.session;
          beginPreparation();
          RCLCPP_INFO(logger(), "Quest reference restored; preparing before resuming tracking");
        }
      }
      // A new stop must acknowledge returning -> standing_after even when the
      // motion already finished. The switcher waits for this fresh handshake.
      if (stop_requested_.exchange(false) && phase_ != Phase::Returning) {
        beginReturn(Phase::Returning);
      }
      // All phases feed reference frames to the same actor. Once the activation
      // handover ends, only references are interpolated, never actor commands.
      const double activation_weight = policy_target_ready_ ?
        mimic::handoverWeight(activation_elapsed_, activation_duration_) : 0.;
      if (policy_target_ready_ && activation_weight == 1.) {phase_elapsed_ += dt;}
      if (phase_ == Phase::StandingBefore) {
        if (phase_elapsed_ >= stand_before_duration_ &&
          (sample.position - stand_position_).cwiseAbs().maxCoeff() <= start_tolerance_)
        {
          beginPreparation();
        }
      } else if (phase_ == Phase::Preparing) {
        if (phase_elapsed_ >= preparation_duration_ && ready &&
          result.generation == reference_generation_ && result.phase == Phase::Preparing &&
          result.reference_elapsed >= preparation_duration_ &&
          (sample.position - current_reference_.position).cwiseAbs().maxCoeff() <= start_tolerance_)
        {
          phase_ = Phase::Tracking;
          tracking_elapsed_ = 0;
          inference_elapsed_ = policy_->metadata().control_dt;
          RCLCPP_INFO(logger(), "SuperMimic motion started at 50Hz");
        }
      } else if (phase_ == Phase::Tracking) {
        tracking_elapsed_ += dt;
        if (!liveReference() && ready && result.generation == reference_generation_ &&
          result.phase == Phase::Tracking && result.frame == policy_->frameCount() - 1)
        {
          beginReturn(Phase::Returning);
          RCLCPP_INFO(logger(), "Motion complete; returning to neural standing");
        }
      } else if (phase_ == Phase::Returning) {
        if (phase_elapsed_ >= return_duration_ && ready && result.phase == Phase::Returning &&
          result.generation == reference_generation_ && result.reference_elapsed >= return_duration_ &&
          (sample.position - stand_position_).cwiseAbs().maxCoeff() <= start_tolerance_)
        {
          phase_ = Phase::StandingAfter;
          RCLCPP_INFO(logger(), "SuperMimic finished; maintaining neural standing until deactivation");
        }
      }
      inference_elapsed_ += dt;
      if ((policy_reset_needed_ || inference_elapsed_ + 1e-9 >= policy_->metadata().control_dt) &&
        submit(sample, time.nanoseconds()))
      {
        inference_elapsed_ = std::fmod(inference_elapsed_, policy_->metadata().control_dt);
      }
      const auto & config = policy_->metadata();
      mimic::PdCommand next;
      // Also preserve legacy clipping while holding feedback before the first result.
      next.position = last_policy_target_;
      if (config.clip_joint_targets) {
        next.position = next.position.cwiseMax(config.lower).cwiseMin(config.upper);
      }
      next.kp = config.kp;
      next.kd = config.kd;
      // Preserve support while the first inference is pending. Only activation
      // blends commands; after it, PD and targets are exactly the actor contract.
      const auto applied = mimic::blendPd(initial_command_, next, activation_weight);
      const auto & target = applied.position;
      for (size_t j = 0; j < jointCount(); ++j) {
        const auto p = joint_to_policy_[j];
        auto & command = jointCommand(j);
        command.position = target[p];
        command.velocity = applied.velocity[p];
        command.effort = applied.effort[p];
        command.kp = applied.kp[p];
        command.kd = applied.kd[p];
        command.torque_mode = isCoupledJoint(joint_names_[j]) ? 2. : 0.;
      }
      if (inferred && log_) {
        log_ << time.nanoseconds() << ',' << tracking_elapsed_ << ',' << frame_index;
        for (double value : sample.position) {log_ << ',' << value;}
        for (double value : sample.velocity) {log_ << ',' << value;}
        for (float value : action) {log_ << ',' << value;}
        for (double value : target) {log_ << ',' << value;}
        log_ << ',' << static_cast<int>(result.phase);
        log_ << ',' << sample.orientation.x() << ',' << sample.orientation.y() <<
          ',' << sample.orientation.z() << ',' << sample.orientation.w();
        for (double value : sample.angular_velocity) {log_ << ',' << value;}
        for (double value : projected_gravity) {log_ << ',' << value;}
        log_ << ',' << imu_pitch_offset_deg_ << ',' << result.input_time_ns << ',' << activation_weight;
        for (double value : result.input.orientation.coeffs()) {log_ << ',' << value;}
        for (double value : result.input.angular_velocity) {log_ << ',' << value;}
        for (double value : result.reference.orientation.coeffs()) {log_ << ',' << value;}
        for (const auto & values : {result.input.position, result.input.velocity, result.reference.position,
          result.target, applied.kp, applied.kd})
        {for (double value : values) {log_ << ',' << value;}}
        log_ << '\n';
      }
    } catch (const std::exception & error) {
      phase_ = Phase::Fault;
      RCLCPP_ERROR(logger(), "SuperMimic fault latched; switch to damping before reactivation: %s", error.what());
      damping();
    }
  }

private:
  using Phase = mimic::Phase;
  using PolicyResult = mimic::PolicyResult;
  struct LiveReference
  {
    std::array<mimic::Frame, mimic::kFuture> frames;
    std::chrono::steady_clock::time_point received{};
    uint64_t session{0}, sequence{0};
    bool valid{false}, enabled{false};
  };
  static bool fresh(const LiveReference & reference)
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - reference.received).count() < .15;
  }
  void receiveReference(const ec_teleop_msgs::msg::QuestReference & message)
  {
    std::lock_guard<std::mutex> lock(reference_mutex_);
    // Reordered packets never refresh the watchdog or overwrite newer state.
    if (reference_.session != 0 && message.session_id == reference_.session &&
      message.sequence <= reference_.sequence) {return;}
    reference_.valid = false;
    const double age = (get_node()->now() - rclcpp::Time(message.header.stamp)).seconds();
    if (message.protocol_version != 1 || message.header.frame_id != "quest_robot_world" || message.session_id == 0) {
      RCLCPP_WARN_THROTTLE(logger(), *get_node()->get_clock(), 2000, "Quest reference rejected: protocol, frame_id or session mismatch");
      return;
    }
    if (!std::isfinite(age) || age < -.05 || age > .15) {
      RCLCPP_WARN_THROTTLE(logger(), *get_node()->get_clock(), 2000,
        "Quest reference rejected: ROS timestamp age %.3fs (allowed -0.050..0.150s); check clock synchronization", age);
      return;
    }
    if (!message.enabled) {
      // Buffering/paused status messages intentionally contain no pose window.
      // They advance sequence state but can never enable tracking.
      reference_.session = message.session_id;
      reference_.sequence = message.sequence;
      reference_.received = std::chrono::steady_clock::now();
      reference_.enabled = false;
      RCLCPP_INFO_THROTTLE(logger(), *get_node()->get_clock(), 2000,
        "Quest waiting: bridge references disabled (calibrated=%d, source_age=%.3fs)", message.calibrated, message.source_age);
      return;
    }
    // Quest's relaxed bridge holds its last measured window for at most 1s.
    // source_age must keep increasing during that hold; the independent ROS
    // receive/timestamp watchdog above still requires live bridge messages.
    if (!std::isfinite(message.source_age) || message.source_age < 0 || message.source_age >= 1.) {
      RCLCPP_WARN_THROTTLE(logger(), *get_node()->get_clock(), 2000,
        "Quest reference rejected: source_age %.3fs outside [0, 1.000)s", message.source_age);
      return;
    }
    if (!std::isfinite(message.sample_dt) || std::abs(message.sample_dt - .02) > 1e-6 || !message.calibrated) {
      RCLCPP_WARN_THROTTLE(logger(), *get_node()->get_clock(), 2000, "Quest reference rejected: calibration or sample interval invalid");
      return;
    }
    LiveReference next;
    for (size_t j = 0; j < mimic::kJoints; ++j) {
      if (message.joint_names[j] != policy_->metadata().joints[j]) {
        RCLCPP_WARN_THROTTLE(logger(), *get_node()->get_clock(), 2000, "Quest reference rejected: joint name mismatch");
        return;
      }
    }
    for (size_t i = 0; i < mimic::kFuture; ++i) {
      auto & f = next.frames[i];
      const auto & p = message.root_poses[i];
      const auto & v = message.root_velocities[i];
      f.root_position = {p.position.x, p.position.y, p.position.z};
      f.orientation = Eigen::Quaterniond(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
      f.linear_velocity = {v.linear.x, v.linear.y, v.linear.z};
      f.angular_velocity = {v.angular.x, v.angular.y, v.angular.z};
      for (size_t j = 0; j < mimic::kJoints; ++j) {
        f.position[j] = message.joint_positions[i * mimic::kJoints + j];
        f.velocity[j] = message.joint_velocities[i * mimic::kJoints + j];
      }
      if (!f.root_position.allFinite() || f.root_position.head<2>().norm() > 100 ||
        f.root_position.z() < .35 || f.root_position.z() > 1.1 ||
        !f.orientation.coeffs().allFinite() || std::abs(f.orientation.norm() - 1) > .001 ||
        !f.position.allFinite() || !f.velocity.allFinite() || f.velocity.cwiseAbs().maxCoeff() > 10 ||
        !f.linear_velocity.allFinite() || f.linear_velocity.norm() > 3 ||
        !f.angular_velocity.allFinite() || f.angular_velocity.norm() > 8 ||
        (f.position.array() < policy_->metadata().lower.array()).any() ||
        (f.position.array() > policy_->metadata().upper.array()).any()) {
        RCLCPP_WARN_THROTTLE(logger(), *get_node()->get_clock(), 2000,
          "Quest reference rejected: frame %zu violates pose, velocity or joint limits", i);
        return;
      }
    }
    next.received = std::chrono::steady_clock::now();
    next.session = message.session_id;
    next.sequence = message.sequence;
    next.enabled = message.enabled;
    next.valid = true;
    reference_ = next;
  }
  bool submit(const mimic::Sample & sample, int64_t input_time_ns)
  {
    // The controller owns reference scheduling; the worker only executes the
    // accepted immutable request. Do not advance reset/reference state on busy.
    mimic::PolicyRequest request;
    request.sample = sample;
    request.input_time_ns = input_time_ns;
    request.phase = phase_.load();
    request.generation = reference_generation_;
    request.reference_elapsed = phase_elapsed_;
    request.frame = phase_ == Phase::Tracking && !liveReference() ? std::min(
      policy_->frameCount() - 1, static_cast<size_t>(
        std::floor((tracking_elapsed_ + 1e-9) / policy_->metadata().control_dt))) : 0;
    request.reset = policy_reset_needed_;
    request.horizon = referenceHorizon(request.frame);
    if (!worker_->submit(request)) {return false;}
    current_reference_ = request.horizon.front();
    policy_reset_needed_ = false;
    job_pending_ = true;
    pending_elapsed_ = 0;
    return true;
  }

  mimic::Frame alignedReference(mimic::Frame frame) const
  {
    frame.orientation = Eigen::Quaterniond(reference_heading_ * frame.orientation.toRotationMatrix());
    frame.root_position = reference_heading_ * frame.root_position + reference_translation_;
    frame.linear_velocity = reference_heading_ * frame.linear_velocity;
    frame.angular_velocity = reference_heading_ * frame.angular_velocity;
    return frame;
  }

  mimic::Frame transitionReference(const mimic::Frame & destination, double elapsed, double duration) const
  {
    if (elapsed >= duration) {return destination;}
    const double t = std::clamp(elapsed / duration, 0., 1.);
    const double blend = t * t * (3. - 2. * t);
    const double rate = 6. * t * (1. - t) / duration;
    mimic::Frame frame;
    frame.position = transition_start_.position + blend * (destination.position - transition_start_.position);
    frame.velocity = rate * (destination.position - transition_start_.position);
    frame.root_position = transition_start_.root_position + blend * (destination.root_position - transition_start_.root_position);
    frame.linear_velocity = rate * (destination.root_position - transition_start_.root_position);
    frame.orientation = transition_start_.orientation.slerp(blend, destination.orientation).normalized();
    const Eigen::AngleAxisd rotation(destination.orientation * transition_start_.orientation.conjugate());
    frame.angular_velocity = rotation.axis() * rotation.angle() * rate;
    return frame;
  }

  std::array<mimic::Frame, mimic::kFuture> referenceHorizon(size_t frame_index) const
  {
    std::array<mimic::Frame, mimic::kFuture> horizon;
    for (size_t i = 0; i < mimic::kFuture; ++i) {
      const double elapsed = phase_elapsed_ + i * policy_->metadata().control_dt;
      if (phase_ == Phase::Preparing || phase_ == Phase::Tracking) {
        const auto reference = alignedReference(liveReference() ? live_reference_.frames[i] :
          policy_->frame(phase_ == Phase::Preparing ? 0 : std::min(frame_index + i, policy_->frameCount() - 1)));
        horizon[i] = phase_ == Phase::Preparing ?
          transitionReference(reference, elapsed, preparation_duration_) : reference;
      } else if (phase_ == Phase::Returning || phase_ == Phase::Paused) {
        horizon[i] = transitionReference(standing_reference_, elapsed, return_duration_);
      } else {
        horizon[i] = standing_reference_;
      }
    }
    return horizon;
  }

  void beginPreparation()
  {
    transition_start_ = current_reference_;
    const auto & destination = liveReference() ? live_reference_.frames.front() : policy_->frame(0);
    reference_heading_ = mimic::projectedYaw(transition_start_.orientation) *
      mimic::projectedYaw(destination.orientation).transpose();
    reference_translation_ = transition_start_.root_position - reference_heading_ * destination.root_position;
    reference_translation_.z() = 0.;
    phase_elapsed_ = 0;
    ++reference_generation_;
    phase_ = Phase::Preparing;
  }

  void beginReturn(Phase destination)
  {
    transition_start_ = current_reference_;
    standing_reference_.orientation = Eigen::Quaterniond(mimic::projectedYaw(current_reference_.orientation));
    // Return to standing at the current reference location, without generating
    // a horizontal velocity command back toward the motion's world origin.
    standing_reference_.root_position.head<2>() = current_reference_.root_position.head<2>();
    phase_elapsed_ = 0;
    ++reference_generation_;
    phase_ = destination;
  }
  void damping()
  {
    for (size_t j = 0; j < jointCount(); ++j) {
      auto & command = jointCommand(j);
      command.position = std::isfinite(jointState(j).position) ? jointState(j).position : 0.;
      command.kd = .5;
      command.torque_mode = isCoupledJoint(joint_names_[j]) ? 2. : 0.;
    }
  }
  std::atomic<Phase> phase_{Phase::StandingBefore};
  std::atomic<bool> active_{false}, stop_requested_{false};
  bool first_update_{true}, policy_reset_needed_{true}, policy_target_ready_{false};
  double preparation_duration_{3.0}, start_tolerance_{.35}, stand_height_{.68};
  double stand_before_duration_{1.0}, return_duration_{3.0};
  double activation_duration_{0.0}, activation_elapsed_{0};
  mimic::PdCommand initial_command_;
  double imu_pitch_offset_deg_{0.0};
  ImuReader imu_;
  double phase_elapsed_{0.}, tracking_elapsed_{0.}, inference_elapsed_{0.};
  mimic::Frame standing_reference_, transition_start_, current_reference_;
  Eigen::Matrix3d reference_heading_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d reference_translation_{Eigen::Vector3d::Zero()};
  uint64_t reference_generation_{0};
  mimic::Joints stand_position_{mimic::Joints::Zero()};
  mimic::Joints last_policy_target_{mimic::Joints::Zero()};
  std::array<size_t, 25> joint_to_policy_{}, position_indices_{}, velocity_indices_{};
  std::unique_ptr<mimic::Policy> policy_;
  std::unique_ptr<mimic::Worker> worker_;
  std::mutex reference_mutex_;
  LiveReference reference_, live_reference_;
  uint64_t live_session_{0};
  rclcpp::Subscription<ec_teleop_msgs::msg::QuestReference>::SharedPtr reference_subscription_;
  bool job_pending_{false};
  double pending_elapsed_{0};
  std::string log_dir_;
  std::string model_path_, controllers_file_;
  std::ofstream log_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::TimerBase::SharedPtr state_timer_;
};
class QuestController : public MimicController
{
protected:
  bool liveReference() const override {return true;}
};
}  // namespace ec

PLUGINLIB_EXPORT_CLASS(ec::MimicController, controller_interface::ControllerInterface)

PLUGINLIB_EXPORT_CLASS(ec::QuestController, controller_interface::ControllerInterface)
