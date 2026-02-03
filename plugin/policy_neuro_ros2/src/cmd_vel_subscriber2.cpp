#include <stepit/policy_neuro/subscriber_action.h>
#include <stepit/policy_neuro_ros2/cmd_vel_subscriber2.h>
#include <stepit/ros2/node.h>

namespace stepit::neuro_policy {
CmdVelSubscriber2::CmdVelSubscriber2(const PolicySpec &policy_spec, const std::string &home_dir)
    : CmdVelSource(policy_spec, home_dir) {
  YAML::Node subscriber_cfg     = config_["cmd_vel_subscriber"];
  auto [topic, topic_type, qos] = parseTopicInfo(subscriber_cfg, "cmd_vel", "geometry_msgs/msg/Twist");
  yml::setIf(subscriber_cfg, "timeout_threshold", timeout_threshold_);
  yml::setIf(subscriber_cfg, "default_enabled", default_subscriber_enabled_);

  if (topic_type == "geometry_msgs/msg/Twist") {
    cmd_vel_sub_ = getNode()->create_subscription<geometry_msgs::msg::Twist>(
        topic, qos, std::bind(&CmdVelSubscriber2::twistCallback, this, std::placeholders::_1));
  } else if (topic_type == "geometry_msgs/msg/TwistStamped") {
    cmd_vel_sub_ = getNode()->create_subscription<geometry_msgs::msg::TwistStamped>(
        topic, qos, std::bind(&CmdVelSubscriber2::twistStampedCallback, this, std::placeholders::_1));
  } else {
    STEPIT_ERROR("Invalid topic_type: '{}'. Expected 'geometry_msgs/msg/Twist' or 'geometry_msgs/msg/TwistStamped'.",
                 topic_type);
  }
}

bool CmdVelSubscriber2::reset() {
  subscriber_enabled_.store(default_subscriber_enabled_, std::memory_order_relaxed);
  subscribing_status_ = publisher::StatusRegistration::make("Policy/CmdVel/Subscribing");
  joystick_rules_.emplace_back([](const joystick::State &js) -> std::string {
    return js.LB().pressed and js.A().on_press ? "Policy/CmdVel/SwitchSubscriber" : "";
  });
  return CmdVelSource::reset();
}

void CmdVelSubscriber2::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  if (not subscriber_enabled_.load(std::memory_order_acquire)) return;
  cmd_vel_msg_   = *msg;
  cmd_vel_stamp_ = getNode()->now();
}

void CmdVelSubscriber2::twistStampedCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  if (not subscriber_enabled_.load(std::memory_order_acquire)) return;
  cmd_vel_msg_   = msg->twist;
  cmd_vel_stamp_ = msg->header.stamp;
}

bool CmdVelSubscriber2::update(const LowState &low_state, ControlRequests &requests, FieldMap &result) {
  bool subscriber_enabled = subscriber_enabled_.load(std::memory_order_acquire);
  subscribing_status_->update(subscriber_enabled ? 1 : 0);
  if (subscriber_enabled and getElapsedTime(cmd_vel_stamp_) < timeout_threshold_) {
    target_cmd_vel_.x() = static_cast<float>(cmd_vel_msg_.linear.x);
    target_cmd_vel_.y() = static_cast<float>(cmd_vel_msg_.linear.y);
    target_cmd_vel_.z() = static_cast<float>(cmd_vel_msg_.angular.z);
  } else {
    target_cmd_vel_.setZero();
  }
  return CmdVelSource::update(low_state, requests, result);
}

void CmdVelSubscriber2::exit() {
  CmdVelSource::exit();
  subscribing_status_.reset();
}

void CmdVelSubscriber2::handleControlRequest(ControlRequest request) {
  switch (lookupAction(request.action(), kSubscriberActionMap)) {
    case SubscriberAction::kEnableSubscriber:
      cmd_vel_msg_ = geometry_msgs::msg::Twist();
      subscriber_enabled_.store(true, std::memory_order_release);
      request.response(kSuccess);
      STEPIT_LOG(kStartSubscribingTemplate, "command velocity");
      break;
    case SubscriberAction::kDisableSubscriber:
      subscriber_enabled_.store(false, std::memory_order_relaxed);
      request.response(kSuccess);
      STEPIT_LOG(kStopSubscribingTemplate, "command velocity");
      break;
    case SubscriberAction::kSwitchSubscriber: {
      bool subscriber_enabled = not subscriber_enabled_.load(std::memory_order_relaxed);
      subscriber_enabled_.store(subscriber_enabled, std::memory_order_relaxed);
      request.response(kSuccess);
      STEPIT_LOG(subscriber_enabled ? kStartSubscribingTemplate : kStopSubscribingTemplate, "command velocity");
      break;
    }
    default:
      if (subscriber_enabled_.load(std::memory_order_relaxed)) {
        if (request.action() == "SetVelocity" or request.action() == "SetVelocityUnscaled" or
            request.action() == "SetTurboRatio") {
          request.response(kNotInCorrectState, fmt::format(kActionBlockedTemplate, request.action()));
          break;
        }
      }
      CmdVelSource::handleControlRequest(std::move(request));
      break;
  }
}

STEPIT_REGISTER_MODULE(cmd_vel_subscriber, kDefPriority, Module::make<CmdVelSubscriber2>);
STEPIT_REGISTER_FIELD_SOURCE(cmd_vel, kDefPriority, Module::make<CmdVelSubscriber2>);
STEPIT_REGISTER_FIELD_SOURCE(cmd_stall, kDefPriority, Module::make<CmdVelSubscriber2>);
}  // namespace stepit::neuro_policy
