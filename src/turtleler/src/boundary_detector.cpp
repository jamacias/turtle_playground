#include "rcl_interfaces/msg/log.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cstdlib>
#include <functional>
#include <memory>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>

class BoundaryDetector : public rclcpp::Node
{
public:
    BoundaryDetector()
    : Node("boundary_detector")
    {
        // const auto onCallback = [this](const rcl_interfaces::msg::Log::SharedPtr msg)
        // {
        //     if (!msg || msg->msg.empty())
        //         return;
        //     RCLCPP_WARN(get_logger(), "Got message: %s", msg->msg.c_str());
        //     // if (const auto firstNumber = msg.msg.find_last_of("Oh no! I hit the wall! (Clamping from [x=");
        //     //     firstNumber != std::string::npos)
        //     // {
        //     //     RCLCPP_INFO(get_logger(), "First number is: %lu", firstNumber);
        //     //     // const auto firstNumberString = std::string(firstNumber, msg.msg.find(",", firstNumber) - 1);
        //     //     // RCLCPP_INFO(get_logger(), "First number is: %s", firstNumberString.c_str());
        //     //     // std::strtod(firstNumber)
        //     // }
        // };
        subscriber_ = create_subscription<rcl_interfaces::msg::Log>("/rosout", 10, std::bind(&BoundaryDetector::onCallback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr subscriber_;
    std::size_t count_ {0};

    void onCallback(const rcl_interfaces::msg::Log::UniquePtr msg)
    {
        if (!msg || msg->msg.empty() )//|| msg->level != rcl_interfaces::msg::Log::WARN)
            return;
        ++count_;

        // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "onCallback counter: %lu", count_);
        RCLCPP_WARN(get_logger(), "onCallback counter: %lu", count_);

        // RCLCPP_WARN(get_logger(), "Got message: %s", msg->msg.c_str());
    };
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BoundaryDetector>());
    rclcpp::shutdown();

    return 0;
}
