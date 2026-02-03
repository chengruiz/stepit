#include <stepit/policy_neuro/subscriber_action.h>
#include <stepit/policy_neuro_ros2/cmd_posture_subscriber2.h>
#include <stepit/ros2/node.h>

namespace stepit::neuro_policy {
CmdRollSubscriber2::CmdRollSubscriber2(const PolicySpec &policy_spec, const std::string &home_dir)
    : CmdRollSource(policy_spec, home_dir) {
  YAML::Node subscriber_cfg = config_["cmd_roll_subscriber"];
  yml::setIf(subscriber_cfg, "timeout_threshold", timeout_threshold_);
  yml::setIf(subscriber_cfg, "default_enabled", default_subscriber_enabled_);
  auto [topic, topic_type, qos] = parseTopicInfo(subscriber_cfg, "cmd_vel", "geometry_msgs/msg/Twist");

  if (topic_type == "std_msgs/msg/Float32") {
    cmd_roll_sub_ = getNode()->create_subscription<std_msgs::msg::Float32>(
        topic, qos, std::bind(&CmdRollSubscriber2::float32Callback, this, std::placeholders::_1));
  } else if (topic_type == "geometry_msgs/msg/Twist") {
    cmd_roll_sub_ = getNode()->create_subscription<geometry_msgs::msg::Twist>(
        topic, qos, std::bind(&CmdRollSubscriber2::twistCallback, this, std::placeholders::_1));
  } else if (topic_type == "geometry_msgs/msg/TwistStamped") {
    cmd_roll_sub_ = getNode()->create_subscription<geometry_msgs::msg::TwistStamped>(
        topic, qos, std::bind(&CmdRollSubscriber2::twistStampedCallback, this, std::placeholders::_1));
  } else {
    STEPIT_ERROR(
        "Invalid topic_type: '{}'. Expected 'std_msgs/msg/Float32', 'geometry_msgs/msg/Twist' or  "
        "'geometry_msgs/msg/TwistStamped'.",
        topic_type);
  }
}

bool CmdRollSubscriber2::reset() {
  subscriber_enabled_.store(default_subscriber_enabled_, std::memory_order_relaxed);
  subscribing_status_ = publisher::StatusRegistration::make("Policy/CmdRoll/Subscribing");
  joystick_rules_.emplace_back([](const joystick::State &js) -> std::string {
    return js.LB().pressed and js.A().on_press ? "Policy/CmdRoll/SwitchSubscriber" : "";
  });
  return CmdRollSource::reset();
}

bool CmdRollSubscriber2::update(const LowState &low_state, ControlRequests &requests, FieldMap &result) {
  bool subscriber_enabled = subscriber_enabled_.load(std::memory_order_acquire);
  subscribing_status_->update(subscriber_enabled ? 1 : 0);
  if (subscriber_enabled) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (getElapsedTime(cmd_roll_stamp_) < timeout_threshold_) {
      cmd_roll_ = clamp(cmd_roll_msg_, -1.0F, 1.0F) * roll_scale_factor_;
    } else {
      cmd_roll_ = 0.0F;
    }
  }
  return CmdRollSource::update(low_state, requests, result);
}

void CmdRollSubscriber2::exit() {
  CmdRollSource::exit();
  subscribing_status_.reset();
}

void CmdRollSubscriber2::float32Callback(const std_msgs::msg::Float32::SharedPtr msg) {
  if (not subscriber_enabled_.load(std::memory_order_acquire)) return;
  std::lock_guard<std::mutex> lock(mutex_);
  cmd_roll_msg_   = msg->data;
  cmd_roll_stamp_ = getNode()->now();
}

void CmdRollSubscriber2::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  if (not subscriber_enabled_.load(std::memory_order_acquire)) return;
  std::lock_guard<std::mutex> lock(mutex_);
  cmd_roll_msg_   = static_cast<float>(msg->angular.x);
  cmd_roll_stamp_ = getNode()->now();
}

void CmdRollSubscriber2::twistStampedCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  if (not subscriber_enabled_.load(std::memory_order_acquire)) return;
  std::lock_guard<std::mutex> lock(mutex_);
  cmd_roll_msg_   = static_cast<float>(msg->twist.angular.x);
  cmd_roll_stamp_ = msg->header.stamp;
}

void CmdRollSubscriber2::handleControlRequest(ControlRequest request) {
  switch (lookupAction(request.action(), kSubscriberActionMap)) {
    case SubscriberAction::kEnableSubscriber:
      cmd_roll_msg_ = {};
      subscriber_enabled_.store(true, std::memory_order_release);
      request.response(kSuccess);
      STEPIT_LOG(kStartSubscribingTemplate, "command roll");
      break;
    case SubscriberAction::kDisableSubscriber:
      subscriber_enabled_.store(false, std::memory_order_relaxed);
      request.response(kSuccess);
      STEPIT_LOG(kStopSubscribingTemplate, "command roll");
      break;
    case SubscriberAction::kSwitchSubscriber: {
      bool subscriber_enabled = not subscriber_enabled_.load(std::memory_order_relaxed);
      subscriber_enabled_.store(subscriber_enabled, std::memory_order_relaxed);
      request.response(kSuccess);
      STEPIT_LOG(subscriber_enabled ? kStartSubscribingTemplate : kStopSubscribingTemplate, "command roll");
      break;
    }
    default:
      if (subscriber_enabled_.load(std::memory_order_relaxed)) {
        if (request.action() == "SetRoll" or request.action() == "SetRollUnscaled") {
          request.response(kNotInCorrectState, fmt::format(kActionBlockedTemplate, request.action()));
          break;
        }
      }
      CmdRollSource::handleControlRequest(std::move(request));
      break;
  }
}
CmdPitchSubscriber2::CmdPitchSubscriber2(const PolicySpec &policy_spec, const std::string &home_dir)
    : CmdPitchSource(policy_spec, home_dir) {
  YAML::Node subscriber_cfg = config_["cmd_pitch_subscriber"];
  yml::setIf(subscriber_cfg, "timeout_threshold", timeout_threshold_);
  yml::setIf(subscriber_cfg, "default_enabled", default_subscriber_enabled_);
  auto [topic, topic_type, qos] = parseTopicInfo(subscriber_cfg, "cmd_vel", "geometry_msgs/msg/Twist");

  if (topic_type == "std_msgs/msg/Float32") {
    cmd_pitch_sub_ = getNode()->create_subscription<std_msgs::msg::Float32>(
        topic, qos, std::bind(&CmdPitchSubscriber2::float32Callback, this, std::placeholders::_1));
  } else if (topic_type == "geometry_msgs/msg/Twist") {
    cmd_pitch_sub_ = getNode()->create_subscription<geometry_msgs::msg::Twist>(
        topic, qos, std::bind(&CmdPitchSubscriber2::twistCallback, this, std::placeholders::_1));
  } else if (topic_type == "geometry_msgs/msg/TwistStamped") {
    cmd_pitch_sub_ = getNode()->create_subscription<geometry_msgs::msg::TwistStamped>(
        topic, qos, std::bind(&CmdPitchSubscriber2::twistStampedCallback, this, std::placeholders::_1));
  } else {
    STEPIT_ERROR(
        "Invalid topic_type: '{}'. Expected 'std_msgs/msg/Float32', 'geometry_msgs/msg/Twist' or "
        "'geometry_msgs/msg/TwistStamped'.",
        topic_type);
  }
}

bool CmdPitchSubscriber2::reset() {
  subscriber_enabled_.store(default_subscriber_enabled_, std::memory_order_relaxed);
  subscribing_status_ = publisher::StatusRegistration::make("Policy/CmdPitch/Subscribing");
  joystick_rules_.emplace_back([](const joystick::State &js) -> std::string {
    return js.LB().pressed and js.A().on_press ? "Policy/CmdPitch/SwitchSubscriber" : "";
  });
  return CmdPitchSource::reset();
}

bool CmdPitchSubscriber2::update(const LowState &low_state, ControlRequests &requests, FieldMap &result) {
  bool subscriber_enabled = subscriber_enabled_.load(std::memory_order_acquire);
  subscribing_status_->update(subscriber_enabled ? 1 : 0);
  if (subscriber_enabled) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (getElapsedTime(cmd_pitch_stamp_) < timeout_threshold_) {
      cmd_pitch_ = clamp(cmd_pitch_msg_, -1.0F, 1.0F) * pitch_scale_factor_;
    } else {
      cmd_pitch_ = 0.0F;
    }
  }
  return CmdPitchSource::update(low_state, requests, result);
}

void CmdPitchSubscriber2::exit() {
  CmdPitchSource::exit();
  subscribing_status_.reset();
}

void CmdPitchSubscriber2::float32Callback(const std_msgs::msg::Float32::SharedPtr msg) {
  if (not subscriber_enabled_.load(std::memory_order_acquire)) return;
  std::lock_guard<std::mutex> lock(mutex_);
  cmd_pitch_msg_   = msg->data;
  cmd_pitch_stamp_ = getNode()->now();
}

void CmdPitchSubscriber2::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  if (not subscriber_enabled_.load(std::memory_order_acquire)) return;
  std::lock_guard<std::mutex> lock(mutex_);
  cmd_pitch_msg_   = static_cast<float>(msg->angular.y);
  cmd_pitch_stamp_ = getNode()->now();
}

void CmdPitchSubscriber2::twistStampedCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  if (not subscriber_enabled_.load(std::memory_order_acquire)) return;
  std::lock_guard<std::mutex> lock(mutex_);
  cmd_pitch_msg_   = static_cast<float>(msg->twist.angular.y);
  cmd_pitch_stamp_ = msg->header.stamp;
}

void CmdPitchSubscriber2::handleControlRequest(ControlRequest request) {
  switch (lookupAction(request.action(), kSubscriberActionMap)) {
    case SubscriberAction::kEnableSubscriber:
      cmd_pitch_msg_ = {};
      subscriber_enabled_.store(true, std::memory_order_release);
      request.response(kSuccess);
      STEPIT_LOG(kStartSubscribingTemplate, "command pitch");
      break;
    case SubscriberAction::kDisableSubscriber:
      subscriber_enabled_.store(false, std::memory_order_relaxed);
      request.response(kSuccess);
      STEPIT_LOG(kStopSubscribingTemplate, "command pitch");
      break;
    case SubscriberAction::kSwitchSubscriber: {
      bool subscriber_enabled = not subscriber_enabled_.load(std::memory_order_relaxed);
      subscriber_enabled_.store(subscriber_enabled, std::memory_order_relaxed);
      request.response(kSuccess);
      STEPIT_LOG(subscriber_enabled ? kStartSubscribingTemplate : kStopSubscribingTemplate, "command pitch");
      break;
    }
    default:
      if (subscriber_enabled_.load(std::memory_order_relaxed)) {
        if (request.action() == "SetPitch" or request.action() == "SetPitchUnscaled") {
          request.response(kNotInCorrectState, fmt::format(kActionBlockedTemplate, request.action()));
          break;
        }
      }
      CmdPitchSource::handleControlRequest(std::move(request));
      break;
  }
}

CmdHeightSubscriber2::CmdHeightSubscriber2(const PolicySpec &policy_spec, const std::string &home_dir)
    : CmdHeightSource(policy_spec, home_dir) {
  YAML::Node subscriber_cfg = config_["cmd_height_subscriber"];
  yml::setIf(subscriber_cfg, "timeout_threshold", timeout_threshold_);
  yml::setIf(subscriber_cfg, "default_enabled", default_subscriber_enabled_);
  auto [topic, topic_type, qos] = parseTopicInfo(subscriber_cfg, "cmd_vel", "geometry_msgs/msg/Twist");

  if (topic_type == "std_msgs/msg/Float32") {
    cmd_height_sub_ = getNode()->create_subscription<std_msgs::msg::Float32>(
        topic, qos, std::bind(&CmdHeightSubscriber2::float32Callback, this, std::placeholders::_1));
  } else if (topic_type == "geometry_msgs/msg/Twist") {
    cmd_height_sub_ = getNode()->create_subscription<geometry_msgs::msg::Twist>(
        topic, qos, std::bind(&CmdHeightSubscriber2::twistCallback, this, std::placeholders::_1));
  } else if (topic_type == "geometry_msgs/msg/TwistStamped") {
    cmd_height_sub_ = getNode()->create_subscription<geometry_msgs::msg::TwistStamped>(
        topic, qos, std::bind(&CmdHeightSubscriber2::twistStampedCallback, this, std::placeholders::_1));
  } else {
    STEPIT_ERROR(
        "Invalid topic_type: '{}'. Expected 'std_msgs/msg/Float32', 'geometry_msgs/msg/Twist' or "
        "'geometry_msgs/msg/TwistStamped'.",
        topic_type);
  }
}

bool CmdHeightSubscriber2::reset() {
  subscriber_enabled_.store(default_subscriber_enabled_, std::memory_order_relaxed);
  subscribing_status_ = publisher::StatusRegistration::make("Policy/CmdHeight/Subscribing");
  joystick_rules_.emplace_back([](const joystick::State &js) -> std::string {
    return js.LB().pressed and js.A().on_press ? "Policy/CmdHeight/SwitchSubscriber" : "";
  });
  return CmdHeightSource::reset();
}

bool CmdHeightSubscriber2::update(const LowState &low_state, ControlRequests &requests, FieldMap &result) {
  bool subscriber_enabled = subscriber_enabled_.load(std::memory_order_acquire);
  subscribing_status_->update(subscriber_enabled ? 1 : 0);
  if (subscriber_enabled) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (getElapsedTime(cmd_height_stamp_) < timeout_threshold_) {
      cmd_height_ = clamp(cmd_height_msg_ + default_cmd_height_, height_range_);
    } else {
      cmd_height_ = default_cmd_height_;
    }
  }
  return CmdHeightSource::update(low_state, requests, result);
}

void CmdHeightSubscriber2::exit() {
  CmdHeightSource::exit();
  subscribing_status_.reset();
}

void CmdHeightSubscriber2::float32Callback(const std_msgs::msg::Float32::SharedPtr msg) {
  if (not subscriber_enabled_.load(std::memory_order_acquire)) return;
  std::lock_guard<std::mutex> lock(mutex_);
  cmd_height_msg_   = msg->data;
  cmd_height_stamp_ = getNode()->now();
}

void CmdHeightSubscriber2::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  if (not subscriber_enabled_.load(std::memory_order_acquire)) return;
  std::lock_guard<std::mutex> lock(mutex_);
  cmd_height_msg_   = static_cast<float>(msg->linear.z);
  cmd_height_stamp_ = getNode()->now();
}

void CmdHeightSubscriber2::twistStampedCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  if (not subscriber_enabled_.load(std::memory_order_acquire)) return;
  std::lock_guard<std::mutex> lock(mutex_);
  cmd_height_msg_   = static_cast<float>(msg->twist.linear.z);
  cmd_height_stamp_ = msg->header.stamp;
}

void CmdHeightSubscriber2::handleControlRequest(ControlRequest request) {
  switch (lookupAction(request.action(), kSubscriberActionMap)) {
    case SubscriberAction::kEnableSubscriber:
      cmd_height_msg_ = {};
      subscriber_enabled_.store(true, std::memory_order_release);
      request.response(kSuccess);
      STEPIT_LOG(kStartSubscribingTemplate, "command height");
      break;
    case SubscriberAction::kDisableSubscriber:
      subscriber_enabled_.store(false, std::memory_order_relaxed);
      request.response(kSuccess);
      STEPIT_LOG(kStopSubscribingTemplate, "command height");
      break;
    case SubscriberAction::kSwitchSubscriber: {
      bool subscriber_enabled = not subscriber_enabled_.load(std::memory_order_relaxed);
      subscriber_enabled_.store(subscriber_enabled, std::memory_order_relaxed);
      request.response(kSuccess);
      STEPIT_LOG(subscriber_enabled ? kStartSubscribingTemplate : kStopSubscribingTemplate, "command height");
      break;
    }
    default:
      if (subscriber_enabled_.load(std::memory_order_relaxed)) {
        if (request.action() == "SetHeight" or request.action() == "IncreaseHeight" or
            request.action() == "DecreaseHeight") {
          request.response(kNotInCorrectState, fmt::format(kActionBlockedTemplate, request.action()));
          break;
        }
      }
      CmdHeightSource::handleControlRequest(std::move(request));
      break;
  }
}

STEPIT_REGISTER_MODULE(cmd_roll_subscriber, kDefPriority, Module::make<CmdRollSubscriber2>);
STEPIT_REGISTER_MODULE(cmd_pitch_subscriber, kDefPriority, Module::make<CmdPitchSubscriber2>);
STEPIT_REGISTER_MODULE(cmd_height_subscriber, kDefPriority, Module::make<CmdHeightSubscriber2>);
STEPIT_REGISTER_FIELD_SOURCE(cmd_roll, kDefPriority, Module::make<CmdRollSubscriber2>);
STEPIT_REGISTER_FIELD_SOURCE(cmd_pitch, kDefPriority, Module::make<CmdPitchSubscriber2>);
STEPIT_REGISTER_FIELD_SOURCE(cmd_height, kDefPriority, Module::make<CmdHeightSubscriber2>);
}  // namespace stepit::neuro_policy
