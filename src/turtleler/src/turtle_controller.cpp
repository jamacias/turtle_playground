#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "turtleler_msgs/action/navigation_goal.hpp"

#include <cmath>
#include <control_toolbox/pid.hpp>
#include <functional>
#include <limits>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/server.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>
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

        {
            steeringController_ = std::make_unique<control_toolbox::Pid>();
            control_toolbox::AntiWindupStrategy strategy;
            strategy.type = control_toolbox::AntiWindupStrategy::BACK_CALCULATION;
            steeringController_->initialize(0.8, 0.2, 0.0, 3.0, -3.0, strategy);
        }

        {
            speedController_ = std::make_unique<control_toolbox::Pid>();
            control_toolbox::AntiWindupStrategy strategy;
            strategy.type = control_toolbox::AntiWindupStrategy::BACK_CALCULATION;
            speedController_->initialize(1.0, 0.1, 0.0, 2.0, 0.05, strategy);
        }
    }

private:
    using NavigationGoal           = turtleler_msgs::action::NavigationGoal;
    using GoalHandleNavigationGoal = rclcpp_action::ServerGoalHandle<NavigationGoal>;

    std::string                                             turtlename_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdPublisher_;
    rclcpp_action::Server<NavigationGoal>::SharedPtr        navigationActionServer_;
    std::shared_ptr<tf2_ros::TransformListener>             tfListener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer>                        tfBuffer_;
    std::unique_ptr<control_toolbox::Pid>                   steeringController_;
    std::unique_ptr<control_toolbox::Pid>                   speedController_;

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

        float        remainingDistance = calculateDistance(worldTgoal.getOrigin(), worldTturtle.getOrigin());
        auto         goalInTurtle      = worldTturtle.inverse() * worldTgoal.getOrigin();
        float        remainingAngle    = std::atan2(goalInTurtle.y(), goalInTurtle.x());
        const float  startingDistance  = calculateDistance(worldTgoal.getOrigin(), worldTturtle.getOrigin());
        rclcpp::Time lastTime          = get_clock()->now();
        while (rclcpp::ok())
        {
            // TODO: cancel action

            worldTturtle      = getPose("world", turtlename_);
            remainingDistance = calculateDistance(worldTgoal.getOrigin(), worldTturtle.getOrigin());
            goalInTurtle      = worldTturtle.inverse() * worldTgoal.getOrigin();

            if (remainingDistance > 0.05)
            {
                // Far away from target, head to that point
                remainingAngle = std::atan2(goalInTurtle.y(), goalInTurtle.x());
            }
            else
            {
                // Already at position, orientate to the right orientation
                const auto& vector1 = worldTturtle.getBasis().getColumn(0);
                const auto& vector2 = worldTgoal.getBasis().getColumn(0);
                remainingAngle      = std::atan2(vector2.y(), vector2.x()) - std::atan2(vector1.y(), vector1.x());
                if (remainingAngle > M_PI)
                {
                    remainingAngle -= 2 * M_PI;
                }
                else if (remainingAngle <= -M_PI)
                {
                    remainingAngle += 2 * M_PI;
                }
            }

            if (remainingDistance < 0.05 && std::abs(remainingAngle) < 0.05)
            {
                RCLCPP_INFO(get_logger(),
                            "Arrived to target (angle error: %.3f [rad] / %.3f [deg]; distance error: %.3f [m])",
                            remainingAngle, remainingAngle * 180.0f / M_PI, remainingDistance);
                break;
            }

            geometry_msgs::msg::Twist command;
            rclcpp::Time time = get_clock()->now();
            const auto   dt   = get_clock()->now() - lastTime;
            if (std::abs(remainingAngle) > 0.05)
            {
                // Not aligned, rotate
                command.angular.z = steeringController_->compute_command(remainingAngle, dt);
            }
            else
            {
                // Aligned enough, move forward
                command.linear.x = speedController_->compute_command(remainingDistance, dt);
            }

            sendCommand(command);

            feedback->progress           = (startingDistance - remainingDistance) / startingDistance;
            feedback->remaining_distance = remainingDistance;
            feedback->remaining_angle    = remainingAngle;
            goalHandle->publish_feedback(feedback);

            controlRate.sleep();
            lastTime = time;
        }

        if (rclcpp::ok())
        {
            sendCommand(geometry_msgs::msg::Twist());

            auto result                = std::make_shared<NavigationGoal::Result>();
            result->success            = true;
            result->remaining_distance = remainingDistance;
            result->remaining_angle    = remainingAngle;
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
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Publishing: %s - Throttled log",
                             geometry_msgs::msg::to_yaml(command, true).c_str());
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
