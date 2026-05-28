#ifndef TURTLELER_ACTIONS_GENERATEDANCEACTION_H
#define TURTLELER_ACTIONS_GENERATEDANCEACTION_H

#include "geometry_msgs/msg/transform_stamped.hpp"

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

using Pose = geometry_msgs::msg::TransformStamped;
// using Goals = nav_msgs::msg::Goals;
using Goals = std::vector<std::string>; // FIXME: use Pose when GoToPoseAction accepts Pose

using namespace BT;

class GenerateDanceAction : public SyncActionNode
{
public:
    GenerateDanceAction(const std::string& name, const NodeConfig& conf);

    static PortsList providedPorts();

    virtual NodeStatus tick() override;
};

#endif // TURTLELER_ACTIONS_GENERATEDANCEACTION_H
