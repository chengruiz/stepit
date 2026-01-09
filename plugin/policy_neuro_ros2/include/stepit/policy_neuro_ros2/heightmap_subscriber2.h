#ifndef STEPIT_NEURO_POLICY_ROS2_HEIGHTMAP_SUBSCRIBER2_H_
#define STEPIT_NEURO_POLICY_ROS2_HEIGHTMAP_SUBSCRIBER2_H_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <stepit/joystick/joystick.h>
#include <stepit/policy_neuro/heightmap_source.h>
#include <stepit/ros2/node.h>

namespace stepit::neuro_policy {
class HeightmapSubscriber2 : public DummyHeightmapSource {
 public:
  HeightmapSubscriber2(const PolicySpec &policy_spec, const std::string &home_dir);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &result) override;
  void exit() override;

 private:
  using InterpolationMethods = grid_map::InterpolationMethods;
  void gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg);
  void poseStampedCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void poseWithCovarianceStampedCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void handleControlRequest(ControlRequest request);
  bool checkAllReady();
  void samplefromMap(float x, float y, float &z, float &u) const;

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr map_sub_{nullptr};
  rclcpp::SubscriptionBase::SharedPtr loc_sub_{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sample_pub_{nullptr};

  std::string map_topic_{"/elevation_mapping/elevation_map"};
  std::string loc_topic_{"/odometry"};
  float map_timeout_threshold_{0.5F};
  float loc_timeout_threshold_{0.1F};
  std::string elevation_layer_{"elevation"};
  std::string uncertainty_layer_{"uncertainty_range"};
  bool elevation_zero_mean_{true};
  bool uncertainty_squared_{false};
  float uncertainty_scaling_{0.25F};
  InterpolationMethods elevation_interp_method_{InterpolationMethods::INTER_NEAREST};
  InterpolationMethods uncertainty_interp_method_{InterpolationMethods::INTER_NEAREST};
  bool publish_samples_{false};
  bool default_subscriber_enabled_{false};
  std::mutex msg_mtx_;
  std::vector<JoystickControl::Registration> js_rules_;
  publisher::StatusRegistration::Ptr subscribing_status_;
  publisher::StatusRegistration::Ptr error_msg_status_;

  std::atomic<bool> subscriber_enabled_{false};
  std::atomic<bool> map_received_{false};
  std::atomic<bool> loc_received_{false};
  bool map_timeout_{false};
  bool loc_timeout_{false};
  std::string error_msg_;
  grid_map::GridMap map_msg_;
  grid_map_msgs::msg::GridMapInfo map_info_;
  rclcpp::Time map_stamp_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::PoseStamped loc_msg_;
  sensor_msgs::msg::PointCloud2 sample_msg_;
  std::vector<Vec2f> global_sample_coords_;
};
}  // namespace stepit::neuro_policy

#endif  // STEPIT_NEURO_POLICY_ROS2_HEIGHTMAP_SUBSCRIBER2_H_
