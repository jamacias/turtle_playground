#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "turtleler_msgs/action/navigation_goal.hpp"

#include <functional>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/server.hpp>
#include <string>
#include <tf2/transform_datatypes.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;
class TurtleController : public rclcpp::Node
{
public:
    TurtleController()
    : Node("turtle_controller")
    {
        turtlename_ = declare_parameter<std::string>("turtlename", "turtle");

        std::ostringstream stream;
        stream << "/" << turtlename_.c_str() << "/cmd_vel";
        std::string topicName = stream.str();
        cmdPublisher_         = create_publisher<geometry_msgs::msg::Twist>(topicName, 0);

        navigationActionServer_ = rclcpp_action::create_server<NavigationGoal>(
            this, "navigation_goal",
            std::bind(&TurtleController::goalCallback, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&TurtleController::cancelCallback, this, std::placeholders::_1),
            std::bind(&TurtleController::acceptCallback, this, std::placeholders::_1));
        tfBuffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
        tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
    }

private:
    using NavigationGoal           = turtleler_msgs::action::NavigationGoal;
    using GoalHandleNavigationGoal = rclcpp_action::ServerGoalHandle<NavigationGoal>;

    std::string                                             turtlename_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdPublisher_;
    rclcpp_action::Server<NavigationGoal>::SharedPtr        navigationActionServer_;
    std::shared_ptr<tf2_ros::TransformListener>             tfListener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer>                        tfBuffer_;

    rclcpp_action::GoalResponse goalCallback(const rclcpp_action::GoalUUID&              uuid,
                                             std::shared_ptr<const NavigationGoal::Goal> goal) const
    {
        RCLCPP_INFO(get_logger(), "%s -- Received goal request: %s", __func__,
                    turtleler_msgs::action::to_yaml(*goal, true).c_str());
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    rclcpp_action::CancelResponse cancelCallback(const std::shared_ptr<GoalHandleNavigationGoal> goalHandle) const
    {
        RCLCPP_INFO(this->get_logger(), "%s -- Received request to cancel goal", __func__);
        (void)goalHandle;
        return rclcpp_action::CancelResponse::ACCEPT;
    };

    void acceptCallback(const std::shared_ptr<GoalHandleNavigationGoal> goalHandle) const
    {
        // this needs to return quickly to avoid blocking the executor,
        // so we declare a lambda function to be called inside a new thread
        const auto execute_in_thread = [this, goalHandle]() { return controlThread(goalHandle); };
        std::thread{execute_in_thread}.detach();
    };

    void controlThread(const std::shared_ptr<GoalHandleNavigationGoal> goalHandle) const
    {
        RCLCPP_INFO(get_logger(), "%s -- Start navigating to the goal", __func__);

        auto         feedback = std::make_shared<NavigationGoal::Feedback>();
        rclcpp::Rate controlRate(100);

        tf2::Stamped<tf2::Transform> worldTgoal;
        tf2::fromMsg(goalHandle->get_goal()->goal_pose, worldTgoal);

        auto worldTturtle = getPose("world", turtlename_);

        float       remainingDistance = calculateDistance(worldTgoal.getOrigin(), worldTturtle.getOrigin());
        const float startingDistance  = remainingDistance;
        while (rclcpp::ok() && remainingDistance > 0.1)
        {
            geometry_msgs::msg::Twist command;
            // TODO: cancel action

            worldTturtle             = getPose("world", turtlename_);
            remainingDistance        = calculateDistance(worldTgoal.getOrigin(), worldTturtle.getOrigin());
            const auto pointInTurtle = worldTturtle.inverse() * worldTgoal.getOrigin();
            if (tf2Fabs(pointInTurtle.y()) > 0.1)
            {
                // Not aligned, rotate
                command.angular.z = 0.6;
            }
            else
            {
                // Aligned enough, move forward
                command.linear.x = 0.6;
            }

            if (pointInTurtle.y() < 0)
            {
                command.angular.z *= -1;
            }

            sendCommand(command);

            feedback->progress           = (startingDistance - remainingDistance) / startingDistance;
            feedback->remaining_distance = remainingDistance;
            goalHandle->publish_feedback(feedback);

            controlRate.sleep();
        }

        if (rclcpp::ok())
        {
            sendCommand(geometry_msgs::msg::Twist());

            auto result                = std::make_shared<NavigationGoal::Result>();
            result->success            = true;
            result->remaining_distance = remainingDistance;
            goalHandle->succeed(result);
            RCLCPP_INFO(get_logger(), "Goal succeeded");
        }
    }

    tf2::Stamped<tf2::Transform> getPose(const std::string& targetFrame, const std::string& sourceFrame) const
    {
        tf2::Stamped<tf2::Transform> pose;
        try
        {
            const auto t = tfBuffer_->lookupTransform(targetFrame, sourceFrame, tf2::TimePointZero);
            // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1, "TF of %s in world: %s", turtlename_.c_str(),
            //                      geometry_msgs::msg::to_yaml(t, true).c_str());
            tf2::convert(t, pose);
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_INFO(get_logger(), "Could not transform %s to %s: %s", turtlename_.c_str(), "world", ex.what());
            // TODO: better handle this
        }

        return pose;
    }

    float calculateDistance(const tf2::Vector3& pointA, const tf2::Vector3& pointB) const
    {
        return tf2::tf2Distance(pointA, pointB);
    }

    void sendCommand(const geometry_msgs::msg::Twist& command) const
    {
        // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1, "Publishing: %s - Throttled log",
        //                      geometry_msgs::msg::to_yaml(command, true).c_str());
        cmdPublisher_->publish(command);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto controller = std::make_shared<TurtleController>();
    rclcpp::spin(controller);
    rclcpp::shutdown();

    return 0;
}
