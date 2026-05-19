#include "bt_conversions.hpp"

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

using TFMessage = tf2_msgs::msg::TFMessage;
using Pose = geometry_msgs::msg::TransformStamped;

using namespace BT;

class GetPoseAction : public RosTopicSubNode<TFMessage>
{
public:
    GetPoseAction(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
    : RosTopicSubNode<TFMessage>(name, conf, params)
    {
    }

    static PortsList providedPorts()
    {
        return providedBasicPorts({OutputPort<Pose>("current_pose", "The current pose.")});
    }

    virtual NodeStatus onTick(const std::shared_ptr<TFMessage>& lastMsg) override;
};
