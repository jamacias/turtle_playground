#ifndef TURTLELER_ACTIONS_GENERATEDANCEACTION_H
#define TURTLELER_ACTIONS_GENERATEDANCEACTION_H

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

using namespace BT;

class GenerateDanceAction : public SyncActionNode
{
public:
    using Pose = geometry_msgs::msg::TransformStamped;
    using Goal  = std::string; // TODO: for some reason using turtle_msgs::action::NavigationGoal::Goal does not work.
    using Goals = std::vector<Goal>;

    GenerateDanceAction(const std::string& name, const NodeConfig& conf);

    static PortsList providedPorts();

    virtual NodeStatus tick() override;
};

#endif // TURTLELER_ACTIONS_GENERATEDANCEACTION_H
