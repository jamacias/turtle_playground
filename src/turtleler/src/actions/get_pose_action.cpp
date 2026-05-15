#include "get_pose_action.hpp"

#include "behaviortree_ros2/plugins.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <behaviortree_cpp/basic_types.h>

NodeStatus GetPoseAction::onTick(const std::shared_ptr<Pose>& lastMsg)
{
    RCLCPP_INFO(logger(), "%s", __func__);

    if (!lastMsg)
        return NodeStatus::FAILURE;

    RCLCPP_INFO(logger(), "%s -- %s", __func__, geometry_msgs::msg::to_yaml(*lastMsg, true).c_str());

    return NodeStatus::SUCCESS;
}

CreateRosNodePlugin(GetPoseAction, "GetPoseAction");