#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32.h>

#include <stepit/policy_neuro/subscriber_action.h>
#include <stepit/policy_neuro_ros/cmd_posture_subscriber.h>

namespace stepit {
namespace neuro_policy {
CmdRollSubscriber::CmdRollSubscriber(const PolicySpec &policy_spec, const std::string &home_dir)
    : CmdRollSource(policy_spec, home_dir) {
  YAML::Node subscriber_cfg = config_["cmd_roll_subscriber"];
  yml::setIf(subscriber_cfg, "timeout_threshold", timeout_threshold_);
  yml::setIf(subscriber_cfg, "default_enabled", default_subscriber_enabled_);
  cmd_roll_sub_ = makeSubscriber(subscriber_cfg, &CmdRollSubscriber::callback, this, "cmd_vel");
}

bool CmdRollSubscriber::reset() {
  subscriber_enabled_.store(default_subscriber_enabled_, std::memory_order_relaxed);
  subscribing_status_ = publisher::StatusRegistration::make("Policy/CmdRoll/Subscribing");
  joystick_rules_.emplace_back([](const joystick::State &js) -> std::string {
    return js.LB().pressed and js.A().on_press ? "Policy/CmdRoll/SwitchSubscriber" : "";
  });
  return CmdRollSource::reset();
}

bool CmdRollSubscriber::update(const LowState &low_state, ControlRequests &requests, FieldMap &result) {
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

void CmdRollSubscriber::exit() {
  CmdRollSource::exit();
  subscribing_status_.reset();
}

void CmdRollSubscriber::callback(const ros::MessageEvent<const topic_tools::ShapeShifter> &event) {
  if (not subscriber_enabled_.load(std::memory_order_acquire)) return;
  auto msg_data = event.getMessage();
  std::lock_guard<std::mutex> lock(mutex_);
  if (msg_data->getDataType() == "std_msgs/Float32") {
    auto msg        = msg_data->instantiate<std_msgs::Float32>();
    cmd_roll_msg_   = msg->data;
    cmd_roll_stamp_ = ros::Time::now();
  } else if (msg_data->getDataType() == "geometry_msgs/Twist") {
    auto msg        = msg_data->instantiate<geometry_msgs::Twist>();
    cmd_roll_msg_   = static_cast<float>(msg->angular.x);
    cmd_roll_stamp_ = ros::Time::now();
  } else if (msg_data->getDataType() == "geometry_msgs/TwistStamped") {
    auto msg        = msg_data->instantiate<geometry_msgs::TwistStamped>();
    cmd_roll_msg_   = static_cast<float>(msg->twist.angular.x);
    cmd_roll_stamp_ = msg->header.stamp;
  } else {
    subscriber_enabled_.store(false, std::memory_order_relaxed);
    STEPIT_WARN("CmdRollSubscriber received a message with unsupported type '{}'.", msg_data->getDataType());
  }
}

void CmdRollSubscriber::handleControlRequest(ControlRequest request) {
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

CmdPitchSubscriber::CmdPitchSubscriber(const PolicySpec &policy_spec, const std::string &home_dir)
    : CmdPitchSource(policy_spec, home_dir) {
  YAML::Node subscriber_cfg = config_["cmd_pitch_subscriber"];
  yml::setIf(subscriber_cfg, "timeout_threshold", timeout_threshold_);
  yml::setIf(subscriber_cfg, "default_enabled", default_subscriber_enabled_);
  cmd_pitch_sub_ = makeSubscriber(subscriber_cfg, &CmdPitchSubscriber::callback, this, "cmd_vel");
}

bool CmdPitchSubscriber::reset() {
  subscriber_enabled_.store(default_subscriber_enabled_, std::memory_order_relaxed);
  subscribing_status_ = publisher::StatusRegistration::make("Policy/CmdPitch/Subscribing");
  joystick_rules_.emplace_back([](const joystick::State &js) -> std::string {
    return js.LB().pressed and js.A().on_press ? "Policy/CmdPitch/SwitchSubscriber" : "";
  });
  return CmdPitchSource::reset();
}

bool CmdPitchSubscriber::update(const LowState &low_state, ControlRequests &requests, FieldMap &result) {
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

void CmdPitchSubscriber::exit() {
  CmdPitchSource::exit();
  subscribing_status_.reset();
}

void CmdPitchSubscriber::callback(const ros::MessageEvent<const topic_tools::ShapeShifter> &event) {
  if (not subscriber_enabled_.load(std::memory_order_acquire)) return;
  auto msg_data = event.getMessage();
  std::lock_guard<std::mutex> lock(mutex_);
  if (msg_data->getDataType() == "std_msgs/Float32") {
    auto msg         = msg_data->instantiate<std_msgs::Float32>();
    cmd_pitch_msg_   = msg->data;
    cmd_pitch_stamp_ = ros::Time::now();
  } else if (msg_data->getDataType() == "geometry_msgs/Twist") {
    auto msg         = msg_data->instantiate<geometry_msgs::Twist>();
    cmd_pitch_msg_   = static_cast<float>(msg->angular.y);
    cmd_pitch_stamp_ = ros::Time::now();
  } else if (msg_data->getDataType() == "geometry_msgs/TwistStamped") {
    auto msg         = msg_data->instantiate<geometry_msgs::TwistStamped>();
    cmd_pitch_msg_   = static_cast<float>(msg->twist.angular.y);
    cmd_pitch_stamp_ = msg->header.stamp;
  } else {
    subscriber_enabled_.store(false, std::memory_order_relaxed);
    STEPIT_WARN("CmdPitchSubscriber received a message with unsupported type '{}'.", msg_data->getDataType());
  }
}

void CmdPitchSubscriber::handleControlRequest(ControlRequest request) {
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

CmdHeightSubscriber::CmdHeightSubscriber(const PolicySpec &policy_spec, const std::string &home_dir)
    : CmdHeightSource(policy_spec, home_dir) {
  YAML::Node subscriber_cfg = config_["cmd_height_subscriber"];
  yml::setIf(subscriber_cfg, "timeout_threshold", timeout_threshold_);
  yml::setIf(subscriber_cfg, "default_enabled", default_subscriber_enabled_);
  cmd_height_sub_ = makeSubscriber(subscriber_cfg, &CmdHeightSubscriber::callback, this, "cmd_vel");
}

bool CmdHeightSubscriber::reset() {
  subscriber_enabled_.store(default_subscriber_enabled_, std::memory_order_relaxed);
  subscribing_status_ = publisher::StatusRegistration::make("Policy/CmdHeight/Subscribing");
  joystick_rules_.emplace_back([](const joystick::State &js) -> std::string {
    return js.LB().pressed and js.A().on_press ? "Policy/CmdHeight/SwitchSubscriber" : "";
  });
  return CmdHeightSource::reset();
}

bool CmdHeightSubscriber::update(const LowState &low_state, ControlRequests &requests, FieldMap &result) {
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

void CmdHeightSubscriber::exit() {
  CmdHeightSource::exit();
  subscribing_status_.reset();
}

void CmdHeightSubscriber::callback(const ros::MessageEvent<const topic_tools::ShapeShifter> &event) {
  if (not subscriber_enabled_.load(std::memory_order_acquire)) return;
  auto msg_data = event.getMessage();
  std::lock_guard<std::mutex> lock(mutex_);
  if (msg_data->getDataType() == "std_msgs/Float32") {
    auto msg          = msg_data->instantiate<std_msgs::Float32>();
    cmd_height_msg_   = msg->data;
    cmd_height_stamp_ = ros::Time::now();
  } else if (msg_data->getDataType() == "geometry_msgs/Twist") {
    auto msg          = msg_data->instantiate<geometry_msgs::Twist>();
    cmd_height_msg_   = static_cast<float>(msg->linear.z);
    cmd_height_stamp_ = ros::Time::now();
  } else if (msg_data->getDataType() == "geometry_msgs/TwistStamped") {
    auto msg          = msg_data->instantiate<geometry_msgs::TwistStamped>();
    cmd_height_msg_   = static_cast<float>(msg->twist.linear.z);
    cmd_height_stamp_ = msg->header.stamp;
  } else {
    subscriber_enabled_.store(false, std::memory_order_relaxed);
    STEPIT_WARN("CmdHeightSubscriber received a message with unsupported type '{}'.", msg_data->getDataType());
  }
}

void CmdHeightSubscriber::handleControlRequest(ControlRequest request) {
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

STEPIT_REGISTER_MODULE(cmd_pitch_subscriber, kDefPriority, Module::make<CmdPitchSubscriber>);
STEPIT_REGISTER_MODULE(cmd_height_subscriber, kDefPriority, Module::make<CmdHeightSubscriber>);
STEPIT_REGISTER_MODULE(cmd_roll_subscriber, kDefPriority, Module::make<CmdRollSubscriber>);
STEPIT_REGISTER_FIELD_SOURCE(cmd_pitch, kDefPriority, Module::make<CmdPitchSubscriber>);
STEPIT_REGISTER_FIELD_SOURCE(cmd_height, kDefPriority, Module::make<CmdHeightSubscriber>);
STEPIT_REGISTER_FIELD_SOURCE(cmd_roll, kDefPriority, Module::make<CmdRollSubscriber>);
}  // namespace neuro_policy
}  // namespace stepit
