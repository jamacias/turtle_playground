#include "generate_dance_action.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <behaviortree_cpp/actions/set_blackboard_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/decorators/loop_node.h>
#include <cstddef>
#include <rcl/time.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2/convert.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

NodeStatus GenerateDanceAction::tick()
{
    const auto logger = rclcpp::get_logger("GenerateDanceAction");

    RCLCPP_DEBUG(logger, "%s", __func__);
    Pose currentPose;
    if (!getInput("current_pose", currentPose))
        return NodeStatus::FAILURE;
    
    RCLCPP_INFO(logger, "%s -- %s", __func__, geometry_msgs::msg::to_yaml(currentPose, true).c_str());

    Goals goals;
    turtleler_msgs::action::NavigationGoal::Goal goal;
    auto& pose = goal.goal_pose;
    pose.header.frame_id = "world";
    pose.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    pose.child_frame_id = currentPose.child_frame_id;
    pose.transform = currentPose.transform;
    tf2::convert(tf2::Quaternion(tf2::Vector3(0, 0, 1), M_PI / 2.0), pose.transform.rotation);
    goals.emplace_back(turtleler_msgs::action::to_yaml(goal, true));

    tf2::convert(tf2::Quaternion(tf2::Vector3(0, 0, 1), -M_PI / 2.0), pose.transform.rotation);
    goals.emplace_back(turtleler_msgs::action::to_yaml(goal, true));

    pose.transform = currentPose.transform;
    goals.emplace_back(turtleler_msgs::action::to_yaml(goal, true));

    for (size_t i = 0; i < goals.size(); ++i)
    {
        RCLCPP_INFO(logger, "%s -- Computed goal[%lu]: %s", __func__, i, goals.at(i).c_str());
    }
    setOutput<Goals>("goals", goals);

    return NodeStatus::SUCCESS;
}

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<GenerateDanceAction>("GenerateDanceAction");

    // TODO: move to its own plugin
    factory.registerNodeType<LoopNode<Goals::value_type>>("LoopPose");
}
