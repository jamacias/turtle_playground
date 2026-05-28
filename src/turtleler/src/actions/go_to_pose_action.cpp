#include "bt_conversions.hpp"
#include "go_to_pose_action.hpp"
#include "utils.h"

using namespace turtleler;

GoToPoseAction::GoToPoseAction(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
: RosActionNode<NavigationGoal>(name, conf, params)
{
}

PortsList GoToPoseAction::providedPorts()
{
    return providedBasicPorts({InputPort<RosActionNode::Goal>("target_pose", "", "The target pose to which to navigate.")});
}

bool GoToPoseAction::setGoal(RosActionNode::Goal& goal)
{
    RCLCPP_DEBUG(logger(), "%s", __func__);

    if (!getInput<RosActionNode::Goal>("target_pose", goal))
    {
        RCLCPP_ERROR(logger(), "%s -- Could not get 'target_pose'", __func__);
        return false;
    }

    RCLCPP_INFO(logger(), "%s -- Goal set to '%s'", __func__,
                turtleler_msgs::action::to_yaml(goal, true).c_str());

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
    RCLCPP_INFO_THROTTLE(logger(), *getClock(node_), 1000, "%s -- Remaining distance: %.3f [m] (%.2f)%%", __func__,
                         feedback->remaining_distance, feedback->progress * 100.0f);

    return NodeStatus::RUNNING;
}
