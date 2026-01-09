#ifndef STEPIT_NEURO_POLICY_ROS_HEIGHTMAP_SUBSCRIBER_H_
#define STEPIT_NEURO_POLICY_ROS_HEIGHTMAP_SUBSCRIBER_H_

#include <geometry_msgs/PoseStamped.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <topic_tools/shape_shifter.h>

#include <stepit/joystick/joystick.h>
#include <stepit/policy_neuro/heightmap_source.h>
#include <stepit/ros/node_handle.h>

namespace stepit {
namespace neuro_policy {
class HeightmapSubscriber : public DummyHeightmapSource {
 public:
  HeightmapSubscriber(const PolicySpec &policy_spec, const std::string &home_dir);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &result) override;
  void exit() override;

 private:
  using InterpolationMethods = grid_map::InterpolationMethods;
  void gridMapCallback(const grid_map_msgs::GridMap::ConstPtr &msg);
  void localizationCallback(const ros::MessageEvent<const topic_tools::ShapeShifter> &event);
  void handleControlRequest(ControlRequest request);
  bool checkAllReady();
  void samplefromMap(float x, float y, float &z, float &u) const;

  ros::Subscriber map_sub_;
  ros::Subscriber loc_sub_;
  ros::Publisher sample_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  float map_timeout_threshold_{0.5F};
  float loc_timeout_threshold_{0.1F};
  bool use_tf_{false};
  std::string robot_frame_id_;
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
  grid_map_msgs::GridMapInfo map_info_;
  geometry_msgs::PoseStamped loc_msg_;
  sensor_msgs::PointCloud2 sample_msg_;
  std::vector<Vec2f> global_sample_coords_;
};
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_ROS_HEIGHTMAP_SUBSCRIBER_H_
