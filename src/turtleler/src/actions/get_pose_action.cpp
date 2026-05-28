#include "get_pose_action.hpp"

#include <algorithm>
#include <behaviortree_cpp/basic_types.h>
#include <rclcpp/logging.hpp>

GetPoseAction::GetPoseAction(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
: RosTopicSubNode<TFMessage>(name, conf, params)
{
}

PortsList GetPoseAction::providedPorts()
{
    return providedBasicPorts({OutputPort<Pose>("current_pose", "The current pose.")});
}

NodeStatus GetPoseAction::onTick(const std::shared_ptr<TFMessage>& lastMsg)
{
    RCLCPP_DEBUG(logger(), "%s", __func__);

    if (!lastMsg)
        return NodeStatus::FAILURE;

    const auto tf = std::find_if(lastMsg->transforms.cbegin(), lastMsg->transforms.cend(),
                                 [](const geometry_msgs::msg::TransformStamped& tf)
                                 { return tf.child_frame_id == "turtle1" && tf.header.frame_id == "world"; });

    if (tf == lastMsg->transforms.cend())
        return NodeStatus::FAILURE;

    RCLCPP_INFO(logger(), "%s -- %s", __func__, geometry_msgs::msg::to_yaml(*tf, true).c_str());

    setOutput<Pose>("current_pose", *tf);

    return NodeStatus::SUCCESS;
}
