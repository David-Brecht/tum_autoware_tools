#ifndef TOD_PERCEPTION_TOOLS__TOD_DUMMY_PERCEPTION_PUB_NODE_HPP_
#define TOD_PERCEPTION_TOOLS__TOD_DUMMY_PERCEPTION_PUB_NODE_HPP_

#include <random>
#include <string>

#include "autoware_perception_msgs/msg/detected_objects.hpp"
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
  void apply_position_noise(autoware_perception_msgs::msg::DetectedObjects & msg);
  void timer_callback();

  double publish_frequency_{10.0};
  std::string output_topic_name_{"~/output/detected_objects"};
  double position_covariance_{0.04};
  std::string detected_objects_path_;

  rclcpp::Publisher<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  autoware_perception_msgs::msg::DetectedObjects base_message_;

  std::mt19937 rng_;
  std::normal_distribution<double> position_noise_{0.0, 0.0};
};

}  // namespace tod_perception_tools

#endif  // TOD_PERCEPTION_TOOLS__TOD_DUMMY_PERCEPTION_PUB_NODE_HPP_
