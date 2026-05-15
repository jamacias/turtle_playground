#include <geometry_msgs/msg/transform_stamped.hpp>
#include "bt_conversions.hpp"

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>

using Pose = geometry_msgs::msg::TransformStamped;

using namespace BT;

class GetPoseAction : public RosTopicSubNode<Pose>
{
public:
    GetPoseAction(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
    : RosTopicSubNode<Pose>(name, conf, params)
    {
    }

    static PortsList providedPorts()
    {
        // return providedBasicPorts({OutputPort<Pose>("current_pose", "", "The current pose.")});
        return providedBasicPorts({});
    }


    virtual NodeStatus onTick(const std::shared_ptr<Pose>& last_msg) override;
};
