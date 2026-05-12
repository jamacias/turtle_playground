#include "go_to_pose_action.hpp"

#include "behaviortree_ros2/plugins.hpp"

bool GoToPoseAction::setGoal(RosActionNode::Goal& goal)
{
    const auto target_pose = getInput<std::string>("target_pose").value();
    goal = convertFromString<RosActionNode::Goal>(target_pose);

    RCLCPP_INFO(logger(), "%s -- Received request: '%s'", __func__, turtleler_msgs::action::to_yaml(goal, true).c_str());

    return true;
}

NodeStatus GoToPoseAction::onResultReceived(const WrappedResult& wr)
{
    RCLCPP_INFO(logger(), "%s -- Arrived to point with deviation: %.3f [m]", __func__, wr.result->remaining_distance);

    return NodeStatus::SUCCESS;
}

NodeStatus GoToPoseAction::onFailure(ActionNodeErrorCode error)
{
    RCLCPP_ERROR(logger(), "Error: %d", error);
    return NodeStatus::FAILURE;
}

NodeStatus GoToPoseAction::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
    rclcpp::Clock clock(RCL_ROS_TIME);
    if (auto node = node_.lock())
    {
        clock = *node->get_clock();
    }
    RCLCPP_INFO_THROTTLE(logger(), clock, 1000, "%s -- Remaining distance: %.3f [m] (%.2f)%%", __func__,
                         feedback->remaining_distance, feedback->progress * 100.0f);

    return NodeStatus::RUNNING;
}

CreateRosNodePlugin(GoToPoseAction, "GoToPoseAction");