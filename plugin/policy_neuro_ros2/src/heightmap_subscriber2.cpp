#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <stepit/policy_neuro/subscriber_action.h>
#include <stepit/policy_neuro_ros2/heightmap_subscriber2.h>
#include <stepit/ros2/node.h>

namespace stepit::neuro_policy {
grid_map::InterpolationMethods parseInterpolationMethod(const std::string &method) {
  if (method == "nearest") return grid_map::InterpolationMethods::INTER_NEAREST;
  if (method == "linear") return grid_map::InterpolationMethods::INTER_LINEAR;
  if (method == "cubic_convolution") return grid_map::InterpolationMethods::INTER_CUBIC_CONVOLUTION;
  if (method == "cubic") return grid_map::InterpolationMethods::INTER_CUBIC;
  STEPIT_ERROR("Unsupported interpolation method '{}'. ", method);
}

HeightmapSubscriber2::HeightmapSubscriber2(const PolicySpec &policy_spec, const std::string &home_dir)
    : DummyHeightmapSource(policy_spec, home_dir) {
  YAML::Node map_sub_cfg = config_["grid_map_subscriber"];
  yml::setIf(map_sub_cfg, "timeout_threshold", map_timeout_threshold_);
  yml::setIf(map_sub_cfg, "default_enabled", default_subscriber_enabled_);
  auto [map_topic, _, map_qos] = parseTopicInfo(map_sub_cfg, "/elevation_mapping/elevation_map");

  YAML::Node loc_sub_cfg = config_["localization_subscriber"];
  yml::setIf(loc_sub_cfg, "timeout_threshold", loc_timeout_threshold_);
  yml::setIf(loc_sub_cfg, "robot_frame_id", robot_frame_id_);
  use_tf_ = not robot_frame_id_.empty();

  yml::setIf(config_, "elevation_layer", elevation_layer_);
  yml::setIf(config_, "uncertainty_layer", uncertainty_layer_);
  if (uncertainty_layer_ == "variance") {
    uncertainty_squared_ = true;
    uncertainty_scaling_ = 1.0F;
  } else {  // Default values for uncertainty_range layer
    uncertainty_squared_ = false;
    uncertainty_scaling_ = 0.25F;
  }
  yml::setIf(config_, "elevation_zero_mean", elevation_zero_mean_);
  yml::setIf(config_, "uncertainty_squared", uncertainty_squared_);
  yml::setIf(config_, "uncertainty_scaling", uncertainty_scaling_);
  std::string elevation_interp_method, uncertainty_interp_method;
  yml::setIf(config_, "elevation_interpolation_method", elevation_interp_method);
  yml::setIf(config_, "uncertainty_interpolation_method", uncertainty_interp_method);
  if (not elevation_interp_method.empty()) {
    elevation_interp_method_ = parseInterpolationMethod(elevation_interp_method);
  }
  if (not uncertainty_interp_method.empty()) {
    uncertainty_interp_method_ = parseInterpolationMethod(uncertainty_interp_method);
  }

  map_sub_ = getNode()->create_subscription<grid_map_msgs::msg::GridMap>(
      map_topic, map_qos, std::bind(&HeightmapSubscriber2::gridMapCallback, this, std::placeholders::_1));
  if (use_tf_) {
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(getNode()->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  } else {
    auto [loc_topic, loc_topic_type, loc_qos] = parseTopicInfo(loc_sub_cfg, "/odometry", "nav_msgs/msg/Odometry");
    if (loc_topic_type == "geometry_msgs/msg/PoseStamped") {
      loc_sub_ = getNode()->create_subscription<geometry_msgs::msg::PoseStamped>(
          loc_topic, loc_qos, std::bind(&HeightmapSubscriber2::poseStampedCallback, this, std::placeholders::_1));
    } else if (loc_topic_type == "geometry_msgs/msg/PoseWithCovarianceStamped") {
      loc_sub_ = getNode()->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          loc_topic, loc_qos,
          std::bind(&HeightmapSubscriber2::poseWithCovarianceStampedCallback, this, std::placeholders::_1));
    } else if (loc_topic_type == "nav_msgs/msg/Odometry") {
      loc_sub_ = getNode()->create_subscription<nav_msgs::msg::Odometry>(
          loc_topic, loc_qos, std::bind(&HeightmapSubscriber2::odometryCallback, this, std::placeholders::_1));
    } else {
      STEPIT_ERROR(
          "Unsupported localization message type '{}'. Expected 'geometry_msgs/msg/Pose', "
          "'geometry_msgs/msg/PoseStamped', 'geometry_msgs/msg/PoseWithCovariance', "
          "'geometry_msgs/msg/PoseWithCovarianceStamped', or 'nav_msgs/msg/Odometry'.",
          loc_topic_type);
    }
  }

  YAML::Node sample_pub_cfg = config_["height_sample_publisher"];
  publish_samples_          = STEPIT_VERBOSITY <= kDbug;
  yml::setIf(sample_pub_cfg, "enabled", publish_samples_);
  if (publish_samples_) {
    auto [sample_topic, _, sample_qos] = parseTopicInfo(sample_pub_cfg, "heightmap_samples");
    sample_pub_              = getNode()->create_publisher<sensor_msgs::msg::PointCloud2>(sample_topic, sample_qos);
    sample_msg_.width        = sample_coords_.size();
    sample_msg_.height       = 1;
    sample_msg_.is_dense     = true;
    sample_msg_.is_bigendian = false;
    sensor_msgs::PointCloud2Modifier modifier(sample_msg_);
    using sensor_msgs::msg::PointField;
    modifier.setPointCloud2Fields(4, "x", 1, PointField::FLOAT32, "y", 1, PointField::FLOAT32, "z", 1,
                                  PointField::FLOAT32, "rgb", 1, PointField::UINT32);
    modifier.resize(sample_coords_.size());
  }
  global_sample_coords_.resize(sample_coords_.size());
}

bool HeightmapSubscriber2::reset() {
  subscriber_enabled_ = default_subscriber_enabled_;
  map_timeout_        = false;
  loc_timeout_        = false;
  error_msg_.clear();
  subscribing_status_ = publisher::StatusRegistration::make("Policy/Heightmap/Subscribing");
  error_msg_status_   = publisher::StatusRegistration::make("Policy/Heightmap/ErrorMessage");
  joystick_rules_.emplace_back([](const joystick::State &js) -> std::string {
    return js.LB().pressed and js.B().on_press ? "Policy/Heightmap/SwitchSubscriber" : "";
  });
  return true;
}

bool HeightmapSubscriber2::update(const LowState &low_state, ControlRequests &requests, FieldMap &result) {
  for (auto &&request : requests.filterByChannel("Policy/Heightmap")) {
    handleControlRequest(std::move(request));
  }

  bool status = checkAllReady();
  subscribing_status_->update(subscriber_enabled_);
  error_msg_status_->update(error_msg_);

  if (not status) {
    elevation_.setZero();
    uncertainty_.setConstant(max_uncertainty_);
    return DummyHeightmapSource::update(low_state, requests, result);
  }
  {
    std::lock_guard<std::mutex> lock(msg_mtx_);
    Vec2f pos{loc_msg_.pose.position.x, loc_msg_.pose.position.y};
    const auto &orn = loc_msg_.pose.orientation;
    Eigen::Rotation2Df rot(Quatf(orn.w, orn.x, orn.y, orn.z).eulerAngles().z());

    for (std::size_t i{}; i < numHeightSamples(); ++i) {
      const auto &local_coord  = sample_coords_[i];
      Vec2f global_coord       = pos + rot * local_coord;
      global_sample_coords_[i] = global_coord;
      samplefromMap(global_coord.x(), global_coord.y(), elevation_[static_cast<Eigen::Index>(i)],
                    uncertainty_[static_cast<Eigen::Index>(i)]);
    }
  }
  int num_finite = 0;
  double sum     = 0.0;
  for (std::size_t i{}; i < numHeightSamples(); ++i) {
    float elevation = elevation_[static_cast<Eigen::Index>(i)];
    if (std::isfinite(elevation)) {
      num_finite++;
      sum += elevation;
    }
  }
  float mean = num_finite ? static_cast<float>(sum / num_finite) : 0.0F;
  for (std::size_t i{}; i < numHeightSamples(); ++i) {
    // Fill in NaN values with mean and set uncertainty to max_uncertainty_
    float &elevation = elevation_[static_cast<Eigen::Index>(i)];
    if (not std::isfinite(elevation)) elevation = mean;
  }
  if (publish_samples_) {
    sensor_msgs::PointCloud2Iterator<float> iter_x(sample_msg_, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(sample_msg_, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(sample_msg_, "z");
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_rgb(sample_msg_, "rgb");
    const auto &map_pos = map_info_.pose.position;

    for (std::size_t i{}; i < numHeightSamples(); ++i) {
      const auto &coord = global_sample_coords_[i];

      *iter_x    = coord.x();
      *iter_y    = coord.y();
      *iter_z    = elevation_[static_cast<Eigen::Index>(i)] + static_cast<float>(map_pos.z);
      uint32_t r = std::min<uint32_t>(uncertainty_[static_cast<Eigen::Index>(i)] * 20 * 255, 255);
      uint32_t g = 255 - r;
      uint32_t b = 0;
      *iter_rgb  = r << 16 | g << 8 | b;

      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_rgb;
    }
    sample_msg_.header = loc_msg_.header;
    sample_pub_->publish(sample_msg_);
  }
  if (elevation_zero_mean_) elevation_ -= mean;
  return DummyHeightmapSource::update(low_state, requests, result);
}

void HeightmapSubscriber2::exit() {
  DummyHeightmapSource::exit();
  joystick_rules_.clear();
  subscribing_status_.reset();
  error_msg_status_.reset();
}

void HeightmapSubscriber2::gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(msg_mtx_);
  grid_map::GridMapRosConverter::fromMessage(*msg, map_msg_);
  map_info_     = msg->info;
  map_received_ = true;
}

void HeightmapSubscriber2::poseStampedCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(msg_mtx_);
  loc_msg_.header = msg->header;
  loc_msg_.pose   = msg->pose;
  loc_received_   = true;
}

void HeightmapSubscriber2::poseWithCovarianceStampedCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(msg_mtx_);
  loc_msg_.header = msg->header;
  loc_msg_.pose   = msg->pose.pose;
  loc_received_   = true;
}

void HeightmapSubscriber2::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(msg_mtx_);
  loc_msg_.header = msg->header;
  loc_msg_.pose   = msg->pose.pose;
  loc_received_   = true;
}

void HeightmapSubscriber2::handleControlRequest(ControlRequest request) {
  switch (lookupAction(request.action(), kSubscriberActionMap)) {
    case SubscriberAction::kEnableSubscriber:
      subscriber_enabled_.store(true, std::memory_order_release);
      request.response(kSuccess);
      STEPIT_LOG(kStartSubscribingTemplate, "heightmap");
      break;
    case SubscriberAction::kDisableSubscriber:
      subscriber_enabled_ = false;
      request.response(kSuccess);
      STEPIT_LOG(kStopSubscribingTemplate, "heightmap");
      break;
    case SubscriberAction::kSwitchSubscriber:
      subscriber_enabled_.store(not subscriber_enabled_, std::memory_order_release);
      request.response(kSuccess);
      STEPIT_LOG(subscriber_enabled_ ? kStartSubscribingTemplate : kStopSubscribingTemplate, "heightmap");
      break;
    default:
      request.response(kUnrecognizedRequest);
      break;
  }
}

bool HeightmapSubscriber2::checkAllReady() {
  std::lock_guard<std::mutex> lock(msg_mtx_);
  if (not subscriber_enabled_) return false;
  if (not map_received_) {
    error_msg_ = "Heightmap subscriber was disabled because the heightmap is not received.";
    STEPIT_WARN(error_msg_);
    return subscriber_enabled_ = false;
  }

  if (use_tf_) {
    const auto &map_frame_id = map_msg_.getFrameId();
    try {
      const auto transform      = tf_buffer_->lookupTransform(map_frame_id, robot_frame_id_, tf2::TimePointZero);
      loc_msg_.header           = transform.header;
      loc_msg_.pose.position.x  = transform.transform.translation.x;
      loc_msg_.pose.position.y  = transform.transform.translation.y;
      loc_msg_.pose.position.z  = transform.transform.translation.z;
      loc_msg_.pose.orientation = transform.transform.rotation;
      loc_received_             = true;
    } catch (const tf2::TransformException &error) {
      error_msg_ = fmt::format(
          "Heightmap subscriber was disabled because TF transform from '{}' to '{}' is unavailable: {}",
          robot_frame_id_, map_frame_id, error.what());
      STEPIT_WARN(error_msg_);
      return subscriber_enabled_ = false;
    }
  }

  if (not loc_received_) {
    error_msg_ = "Heightmap subscriber was disabled because the localization is not received.";
    STEPIT_WARN(error_msg_);
    return subscriber_enabled_ = false;
  }

  double map_lag   = getElapsedTime(map_stamp_);
  bool map_timeout = map_lag > map_timeout_threshold_;
  if (map_timeout) {
    if (not map_timeout_) {
      map_timeout_ = true;
      error_msg_   = fmt::format(
          "Heightmap subscriber was interrupted because the heightmap is outdated (received {:.2f}s ago).", map_lag);
      STEPIT_WARN(error_msg_);
    }
    return false;
  } else if (map_timeout_) {
    map_timeout_ = false;
    error_msg_   = fmt::format("Heightmap subscriber recovered as the heightmap is up-to-date (received {:.2f}s ago).",
                               map_lag);
    STEPIT_INFO(error_msg_);
  }

  double loc_lag   = getElapsedTime(loc_msg_.header.stamp);
  bool loc_timeout = loc_lag > loc_timeout_threshold_;
  if (loc_timeout) {
    if (not loc_timeout_) {
      loc_timeout_ = true;
      error_msg_   = fmt::format(
          "Heightmap subscriber was interrupted because the localization is outdated (received {:.2f}s ago).", loc_lag);
      STEPIT_WARN(error_msg_);
    }
    return false;
  } else if (loc_timeout_) {
    loc_timeout_ = false;
    error_msg_ = fmt::format("Heightmap subscriber recovered as the localization is up-to-date (received {:.2f}s ago).",
                             loc_lag);
    STEPIT_INFO(error_msg_);
  }

  const auto &layers = map_msg_.getLayers();
  if (std::find(layers.begin(), layers.end(), elevation_layer_) == layers.end()) {
    error_msg_ = fmt::format("Heightmap subscriber was disabled because the elevation layer '{}' is missing.",
                             elevation_layer_);
    STEPIT_WARN(error_msg_);
    return subscriber_enabled_ = false;
  }
  if (not uncertainty_layer_.empty() and std::find(layers.begin(), layers.end(), uncertainty_layer_) == layers.end()) {
    error_msg_ = fmt::format("Heightmap subscriber was disabled because the uncertainty layer '{}' is missing.",
                             uncertainty_layer_);
    STEPIT_WARN(error_msg_);
    return subscriber_enabled_ = false;
  }
  return true;
}

void HeightmapSubscriber2::samplefromMap(float x, float y, float &z, float &u) const {
  try {
    z = map_msg_.atPosition(elevation_layer_, {x, y}, elevation_interp_method_);
  } catch (const std::out_of_range &) {
    z = std::numeric_limits<float>::quiet_NaN();
  }
  if (not std::isfinite(z)) {
    u = max_uncertainty_;
    return;
  }
  if (uncertainty_layer_.empty()) {
    u = default_uncertainty_;
    return;
  }

  u = map_msg_.atPosition(uncertainty_layer_, {x, y}, uncertainty_interp_method_);
  if (u < 0 or not std::isfinite(u)) {
    u = max_uncertainty_;
    return;
  }
  if (uncertainty_squared_) u = std::sqrt(u);
  u *= uncertainty_scaling_;
}

STEPIT_REGISTER_MODULE(heightmap_subscriber, kDefPriority, Module::make<HeightmapSubscriber2>);
STEPIT_REGISTER_FIELD_SOURCE(heightmap, kDefPriority, Module::make<HeightmapSubscriber2>);
STEPIT_REGISTER_FIELD_SOURCE(heightmap_uncertainty, kDefPriority, Module::make<HeightmapSubscriber2>);
}  // namespace stepit::neuro_policy
