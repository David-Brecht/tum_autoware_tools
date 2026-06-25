#ifndef TOD_PERCEPTION_TOOLS__TOD_DETECTED_OBJECT_MERGER_NODE_HPP_
#define TOD_PERCEPTION_TOOLS__TOD_DETECTED_OBJECT_MERGER_NODE_HPP_

#include <array>
#include <memory>
#include <string>

#include "autoware_perception_msgs/msg/detected_objects.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace tod_perception_tools
{

// Merges two DetectedObjects streams into one. Unlike autoware_simple_object_merger,
// the inputs may live in different frames: each input is transformed independently
// into new_frame_id before being concatenated, and merging is driven by a timer
// (no ApproximateTime stamp synchronization that breaks across unsynced sources).
class TodDetectedObjectMergerNode : public rclcpp::Node
{
public:
  TodDetectedObjectMergerNode();

private:
  using DetectedObjects = autoware_perception_msgs::msg::DetectedObjects;

  void on_data(const DetectedObjects::ConstSharedPtr msg, size_t input_index);
  void on_timer();

  DetectedObjects::SharedPtr transform_objects(
    const DetectedObjects::ConstSharedPtr & objects, const std::string & target_frame_id);

  static constexpr size_t kNumInputs = 2;

  std::array<std::string, kNumInputs> input_topics_{"~/input/objects0", "~/input/objects1"};
  std::string output_topic_name_{"~/output/objects"};
  std::string new_frame_id_{"base_link"};
  double update_rate_hz_{10.0};      // merging frequency
  double timeout_threshold_{1.0};    // drop inputs older than this many seconds

  std::array<rclcpp::Subscription<DetectedObjects>::SharedPtr, kNumInputs> sub_objects_;
  std::array<DetectedObjects::ConstSharedPtr, kNumInputs> objects_data_;

  rclcpp::Publisher<DetectedObjects>::SharedPtr pub_objects_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace tod_perception_tools

#endif  // TOD_PERCEPTION_TOOLS__TOD_DETECTED_OBJECT_MERGER_NODE_HPP_
