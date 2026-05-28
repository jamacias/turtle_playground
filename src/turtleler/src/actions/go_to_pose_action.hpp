#ifndef TURTLELER_ACTIONS_GOTOPOSEACTION_H
#define TURTLELER_ACTIONS_GOTOPOSEACTION_H

#include "turtleler_msgs/action/navigation_goal.hpp"

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_ros2/bt_action_node.hpp>

using NavigationGoal = turtleler_msgs::action::NavigationGoal;
using namespace BT;

class GoToPoseAction : public RosActionNode<NavigationGoal>
{
public:
    GoToPoseAction(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);

    static PortsList providedPorts();

    bool setGoal(RosActionNode::Goal& goal) override;

    NodeStatus onResultReceived(const WrappedResult& wr) override;

    virtual NodeStatus onFailure(ActionNodeErrorCode error) override;

    NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;
};

#endif // TURTLELER_ACTIONS_GOTOPOSEACTION_H
