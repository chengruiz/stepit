#include <geometry_msgs/TwistStamped.h>

#include <stepit/policy_neuro/subscriber_action.h>
#include <stepit/policy_neuro_ros/cmd_vel_subscriber.h>

namespace stepit {
namespace neuro_policy {
CmdVelSubscriber::CmdVelSubscriber(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec)
    : CmdVelSource(policy_spec, ModuleSpec(module_spec, "cmd_vel")) {
  yml::Node subscriber_cfg = config_["cmd_vel_subscriber"];
  subscriber_cfg["timeout_threshold"].to(timeout_threshold_, true);
  subscriber_cfg["default_enabled"].to(default_subscriber_enabled_, true);
  cmd_vel_sub_ = makeSubscriber(subscriber_cfg, &CmdVelSubscriber::callback, this, "cmd_vel");
}

bool CmdVelSubscriber::reset() {
  subscriber_enabled_.store(default_subscriber_enabled_, std::memory_order_relaxed);
  subscribing_status_ = publisher::StatusRegistration::make("Policy/CmdVel/Subscribing");
  if (not CmdVelSource::reset()) return false;
  joystick_rules_.emplace_back([](const joystick::State &js) -> std::string {
    return js.LB().pressed and js.A().on_press ? "Policy/CmdVel/SwitchSubscriber" : "";
  });
  return true;
}

bool CmdVelSubscriber::update(const LowState &low_state, ControlRequests &requests, FieldMap &context) {
  bool subscriber_enabled = subscriber_enabled_.load(std::memory_order_acquire);
  subscribing_status_->update(subscriber_enabled ? 1 : 0);
  if (subscriber_enabled) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (getElapsedTime(cmd_vel_stamp_) < timeout_threshold_) {
      target_cmd_vel_.x() = cmd_vel_msg_.linear.x;
      target_cmd_vel_.y() = cmd_vel_msg_.linear.y;
      target_cmd_vel_.z() = cmd_vel_msg_.angular.z;
    } else {
      target_cmd_vel_.setZero();
    }
  } else {
    target_cmd_vel_.setZero();
  }
  return CmdVelSource::update(low_state, requests, context);
}

void CmdVelSubscriber::exit() {
  CmdVelSource::exit();
  subscribing_status_.reset();
}

void CmdVelSubscriber::callback(const ros::MessageEvent<const topic_tools::ShapeShifter> &event) {
  if (not subscriber_enabled_.load(std::memory_order_acquire)) return;
  auto raw_msg = event.getMessage();
  std::lock_guard<std::mutex> lock(mutex_);
  if (raw_msg->getDataType() == "geometry_msgs/Twist") {
    auto msg       = raw_msg->instantiate<geometry_msgs::Twist>();
    cmd_vel_msg_   = *msg;
    cmd_vel_stamp_ = ros::Time::now();
  } else if (raw_msg->getDataType() == "geometry_msgs/TwistStamped") {
    auto msg       = raw_msg->instantiate<geometry_msgs::TwistStamped>();
    cmd_vel_msg_   = msg->twist;
    cmd_vel_stamp_ = msg->header.stamp;
  } else {
    subscriber_enabled_.store(false, std::memory_order_relaxed);
    STEPIT_WARN("CmdVelSubscriber received a message with unsupported type '{}'.", raw_msg->getDataType());
  }
}

void CmdVelSubscriber::handleControlRequest(ControlRequest request) {
  switch (lookupAction(request.action(), kSubscriberActionMap)) {
    case SubscriberAction::kEnableSubscriber:
      subscriber_enabled_.store(true, std::memory_order_release);
      request.response(kSuccess);
      STEPIT_LOG(kStartSubscribingTemplate, "command velocity");
      break;
    case SubscriberAction::kDisableSubscriber:
      subscriber_enabled_.store(false, std::memory_order_relaxed);
      target_cmd_vel_.setZero();
      request.response(kSuccess);
      STEPIT_LOG(kStopSubscribingTemplate, "command velocity");
      break;
    case SubscriberAction::kSwitchSubscriber: {
      bool subscriber_enabled = not subscriber_enabled_.load(std::memory_order_relaxed);
      subscriber_enabled_.store(subscriber_enabled, std::memory_order_relaxed);
      if (not subscriber_enabled) target_cmd_vel_.setZero();
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

STEPIT_REGISTER_MODULE(cmd_vel_subscriber, kDefPriority, Module::make<CmdVelSubscriber>);
STEPIT_REGISTER_FIELD_SOURCE(cmd_vel, kDefPriority, Module::make<CmdVelSubscriber>);
STEPIT_REGISTER_FIELD_SOURCE(cmd_stall, kDefPriority, Module::make<CmdVelSubscriber>);
}  // namespace neuro_policy
}  // namespace stepit
