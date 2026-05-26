#ifndef TURTLELER_ACTIONS_UTILS_H
#define TURTLELER_ACTIONS_UTILS_H

#include <memory>
#include <rclcpp/node.hpp>

namespace turtleler
{
std::shared_ptr<rclcpp::Clock> getClock(const std::weak_ptr<rclcpp::Node> node);
}

#endif // TURTLELER_ACTIONS_UTILS_H
