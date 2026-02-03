#include <geometry_msgs/TwistStamped.h>

#include <stepit/policy_neuro/subscriber_action.h>
#include <stepit/policy_neuro_ros/cmd_vel_subscriber.h>

namespace stepit {
namespace neuro_policy {
CmdVelSubscriber::CmdVelSubscriber(const PolicySpec &policy_spec, const std::string &home_dir)
    : CmdVelSource(policy_spec, home_dir) {
  YAML::Node subscriber_cfg = config_["cmd_vel_subscriber"];
  yml::setIf(subscriber_cfg, "timeout_threshold", timeout_threshold_);
  yml::setIf(subscriber_cfg, "default_enabled", default_subscriber_enabled_);
  cmd_vel_sub_ = makeSubscriber(subscriber_cfg, &CmdVelSubscriber::callback, this, "cmd_vel");
}

bool CmdVelSubscriber::reset() {
  subscriber_enabled_.store(default_subscriber_enabled_, std::memory_order_relaxed);
  subscribing_status_ = publisher::StatusRegistration::make("Policy/CmdVel/Subscribing");
  joystick_rules_.emplace_back([](const joystick::State &js) -> std::string {
    return js.LB().pressed and js.A().on_press ? "Policy/CmdVel/SwitchSubscriber" : "";
  });
  return CmdVelSource::reset();
}

bool CmdVelSubscriber::update(const LowState &low_state, ControlRequests &requests, FieldMap &result) {
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
  }
  return CmdVelSource::update(low_state, requests, result);
}

void CmdVelSubscriber::exit() {
  CmdVelSource::exit();
  subscribing_status_.reset();
}

void CmdVelSubscriber::callback(const ros::MessageEvent<const topic_tools::ShapeShifter> &event) {
  if (not subscriber_enabled_.load(std::memory_order_acquire)) return;
  auto msg = event.getMessage();
  std::lock_guard<std::mutex> lock(mutex_);
  if (msg->getDataType() == "geometry_msgs/Twist") {
    auto t_msg     = msg->instantiate<geometry_msgs::Twist>();
    cmd_vel_msg_   = *t_msg;
    cmd_vel_stamp_ = ros::Time::now();
  } else if (msg->getDataType() == "geometry_msgs/TwistStamped") {
    auto ts_msg    = msg->instantiate<geometry_msgs::TwistStamped>();
    cmd_vel_msg_   = ts_msg->twist;
    cmd_vel_stamp_ = ts_msg->header.stamp;
  } else {
    subscriber_enabled_.store(false, std::memory_order_relaxed);
    STEPIT_WARN("CmdVelSubscriber received a message with unsupported type '{}'.", msg->getDataType());
  }
}

void CmdVelSubscriber::handleControlRequest(ControlRequest request) {
  switch (lookupAction(request.action(), kSubscriberActionMap)) {
    case SubscriberAction::kEnableSubscriber:
      cmd_vel_msg_ = {};
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

STEPIT_REGISTER_MODULE(cmd_vel_subscriber, kDefPriority, Module::make<CmdVelSubscriber>);
STEPIT_REGISTER_FIELD_SOURCE(cmd_vel, kDefPriority, Module::make<CmdVelSubscriber>);
STEPIT_REGISTER_FIELD_SOURCE(cmd_stall, kDefPriority, Module::make<CmdVelSubscriber>);
}  // namespace neuro_policy
}  // namespace stepit
