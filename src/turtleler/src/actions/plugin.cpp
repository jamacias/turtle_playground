#include "generate_dance_action.hpp"
#include "get_pose_action.hpp"
#include "go_to_pose_action.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/decorators/loop_node.h>
#include <behaviortree_ros2/plugins.hpp>

// Not sure if I like this approach. On one hand it is the only way
// to have a single plugin with all the necessary nodes, which is easier to
// maintain, while at the same time it is kinda confusing that non-ROS nodes
// have to be inside a function called *_ROS_NODES. Also, I am not sure what
// happens if the "params" have to be different for each node? Hmm.
// The alternative of having a plugin for each node is also ugly IMHO: you
// have more boilerplate to maintain the different shared libraries and make
// sure they end up in the right folder...
BT_REGISTER_ROS_NODES(factory, params)
{
    // Keep alphabetical order(!)

    // ROS nodes
    factory.registerNodeType<GetPoseAction>("GetPoseAction", params);
    factory.registerNodeType<GoToPoseAction>("GoToPoseAction", params);

    // Pure BT nodes
    factory.registerNodeType<GenerateDanceAction>("GenerateDanceAction");
    factory.registerNodeType<LoopNode<Goals::value_type>>("LoopPose");
}
