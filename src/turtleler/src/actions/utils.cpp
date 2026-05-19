#include "utils.h"

namespace turtleler
{

std::shared_ptr<rclcpp::Clock> getClock(const std::weak_ptr<rclcpp::Node> node)
{
    if (const auto n = node.lock())
    {
        return n->get_clock();
    }

    return std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
}

} // namespace turtleler
