#ifndef TOD_PERCEPTION_TOOLS__TOD_DUMMY_PERCEPTION_PUB_NODE_HPP_
#define TOD_PERCEPTION_TOOLS__TOD_DUMMY_PERCEPTION_PUB_NODE_HPP_

#include <random>
#include <string>

#include "autoware_perception_msgs/msg/detected_objects.hpp"
#include "autoware_perception_msgs/msg/predicted_objects.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/create_timer.hpp>

namespace tod_perception_tools
{

class TodDummyPerceptionPubNode : public rclcpp::Node
{
public:
  TodDummyPerceptionPubNode();

private:
  autoware_perception_msgs::msg::DetectedObjects load_detected_objects(const std::string & path);
  autoware_perception_msgs::msg::PredictedObjects load_predicted_objects(const std::string & path);
  double sample_noise(double std_dev);
  void apply_pose_noise(geometry_msgs::msg::PoseWithCovariance & pose_with_covariance);
  void apply_position_noise(autoware_perception_msgs::msg::DetectedObjects & msg);
  void apply_position_noise(autoware_perception_msgs::msg::PredictedObjects & msg);
  void timer_callback();

  double publish_frequency_{10.0};
  std::string output_topic_name_{"~/output/detected_objects"};
  // Per-axis standard deviations (matching the tier4 dummy object rviz plugin).
  double std_dev_x_{0.03};       // [m]
  double std_dev_y_{0.03};       // [m]
  double std_dev_z_{0.03};       // [m]
  double std_dev_theta_{0.0};    // [rad] yaw
  bool publish_predicted_objects_{false};
  std::string detected_objects_path_;

  rclcpp::Publisher<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr detected_pub_;
  rclcpp::Publisher<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr predicted_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  autoware_perception_msgs::msg::DetectedObjects detected_base_message_;
  autoware_perception_msgs::msg::PredictedObjects predicted_base_message_;

  std::mt19937 rng_;
};

}  // namespace tod_perception_tools

#endif  // TOD_PERCEPTION_TOOLS__TOD_DUMMY_PERCEPTION_PUB_NODE_HPP_
