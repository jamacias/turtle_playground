#ifndef TURTLELER_ACTIONS_BTCONVERSIONS_H
#define TURTLELER_ACTIONS_BTCONVERSIONS_H

#include "turtleler_msgs/action/navigation_goal.hpp"
#include "yaml-cpp/yaml.h"

#include <behaviortree_cpp/behavior_tree.h>


namespace BT
{
using NavigationGoal = turtleler_msgs::action::NavigationGoal;
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
        output.goal_pose.header.stamp.nanosec = msg["goal_pose"]["header"]["stamp"]["nanosec"].as<uint32_t>();
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

#endif // TURTLELER_ACTIONS_BTCONVERSIONS_H
