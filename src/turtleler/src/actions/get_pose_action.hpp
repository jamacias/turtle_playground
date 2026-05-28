#ifndef TURTLELER_ACTIONS_GETPOSEACTION_H
#define TURTLELER_ACTIONS_GETPOSEACTION_H

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

using namespace BT;

class GetPoseAction : public RosTopicSubNode<tf2_msgs::msg::TFMessage>
{
public:
    using TFMessage = tf2_msgs::msg::TFMessage;
    using Pose      = geometry_msgs::msg::TransformStamped;
    GetPoseAction(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);

    static PortsList providedPorts();

    virtual NodeStatus onTick(const std::shared_ptr<TFMessage>& lastMsg) override;
};

#endif // TURTLELER_ACTIONS_GETPOSEACTION_H
