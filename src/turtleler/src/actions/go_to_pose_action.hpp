#include "turtleler_msgs/action/navigation_goal.hpp"
#include "yaml-cpp/yaml.h"

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_ros2/bt_action_node.hpp>

using NavigationGoal = turtleler_msgs::action::NavigationGoal;

namespace BT
{
template <>
inline NavigationGoal::Goal convertFromString(StringView str)
{
    NavigationGoal::Goal output;
    try
    {
        const YAML::Node msg = YAML::Load(std::string(str));
        output.goal_pose.child_frame_id       = msg["goal_pose"]["child_frame_id"].as<std::string>();
        output.goal_pose.header.frame_id      = msg["goal_pose"]["header"]["frame_id"].as<std::string>();
        output.goal_pose.header.stamp.sec     = msg["goal_pose"]["header"]["stamp"]["sec"].as<int32_t>();
        output.goal_pose.header.stamp.nanosec = msg["goal_pose"]["header"]["stamp"]["sec"].as<uint32_t>();
        output.goal_pose.transform.translation.x = msg["goal_pose"]["transform"]["translation"]["x"].as<double>();
        output.goal_pose.transform.translation.y = msg["goal_pose"]["transform"]["translation"]["y"].as<double>();
        output.goal_pose.transform.translation.z = msg["goal_pose"]["transform"]["translation"]["z"].as<double>();
        output.goal_pose.transform.rotation.x = msg["goal_pose"]["transform"]["rotation"]["x"].as<double>();
        output.goal_pose.transform.rotation.y = msg["goal_pose"]["transform"]["rotation"]["y"].as<double>();
        output.goal_pose.transform.rotation.z = msg["goal_pose"]["transform"]["rotation"]["z"].as<double>();
        output.goal_pose.transform.rotation.w = msg["goal_pose"]["transform"]["rotation"]["w"].as<double>();
    }
    catch (const YAML::Exception& e)
    {
        std::cerr << e.what() << "\n";
    }
    return output;
}
} // namespace BT

using namespace BT;

class GoToPoseAction : public RosActionNode<NavigationGoal>
{
public:
    GoToPoseAction(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
    : RosActionNode<NavigationGoal>(name, conf, params)
    {
    }

    // The specific ports of this Derived class
    // should be merged with the ports of the base class,
    // using RosActionNode::providedBasicPorts()
    static PortsList providedPorts()
    {
        return providedBasicPorts({InputPort<std::string>("target_pose", "", "The target pose to which to navigate.")});
    }

    // This is called when the TreeNode is ticked and it should
    // send the request to the action server
    bool setGoal(RosActionNode::Goal& goal) override;

    // Callback executed when the reply is received.
    // Based on the reply you may decide to return SUCCESS or FAILURE.
    NodeStatus onResultReceived(const WrappedResult& wr) override;

    // Callback invoked when there was an error at the level
    // of the communication between client and server.
    // This will set the status of the TreeNode to either SUCCESS or FAILURE,
    // based on the return value.
    // If not overridden, it will return FAILURE by default.
    virtual NodeStatus onFailure(ActionNodeErrorCode error) override;

    // we also support a callback for the feedback, as in
    // the original tutorial.
    // Usually, this callback should return RUNNING, but you
    // might decide, based on the value of the feedback, to abort
    // the action, and consider the TreeNode completed.
    // In that case, return SUCCESS or FAILURE.
    // The Cancel request will be send automatically to the server.
    NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;
};
