#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "turtleler_msgs/action/navigation_goal.hpp"
#include "actions/go_to_pose_action.hpp"

#include <filesystem>
#include <functional>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/tree_execution_server.hpp>
#include <behaviortree_ros2/plugins.hpp>


class TurtleBrain : public rclcpp::Node
{
public:
    TurtleBrain()
    : Node("turtle_brain")
    {
        subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&TurtleBrain::newGoalCallback, this, std::placeholders::_1));
        tfPublisher_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        treeRequester_ = rclcpp_action::create_client<ExecuteTree>(this, "bt_execution");
    }

private:
    using ExecuteTree              = btcpp_ros2_interfaces::action::ExecuteTree;
    using ExecuteTreeGoalHandle    = rclcpp_action::ClientGoalHandle<ExecuteTree>;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>                   tfPublisher_;
    geometry_msgs::msg::TransformStamped                             currentGoal_{};
    rclcpp_action::Client<ExecuteTree>::SharedPtr                    treeRequester_;

    void newGoalCallback(const geometry_msgs::msg::PoseStamped::UniquePtr msg)
    {
        if (!msg)
            return;

        RCLCPP_INFO(get_logger(), "%s -- Received message: %s", __func__, geometry_msgs::msg::to_yaml(*msg, true).c_str());

        geometry_msgs::msg::TransformStamped goalTF;
        goalTF.header                  = msg->header;
        goalTF.transform.translation.x = msg->pose.position.x;
        goalTF.transform.translation.y = msg->pose.position.y;
        goalTF.transform.translation.z = msg->pose.position.z;
        goalTF.transform.rotation.w    = msg->pose.orientation.w;
        goalTF.transform.rotation.x    = msg->pose.orientation.x;
        goalTF.transform.rotation.y    = msg->pose.orientation.y;
        goalTF.transform.rotation.z    = msg->pose.orientation.z;
        goalTF.child_frame_id          = "goal";

        currentGoal_ = goalTF;

        tfPublisher_->sendTransform(goalTF);

        NavigationGoal::Goal goal;
        goal.goal_pose = goalTF;
        const auto payload = turtleler_msgs::action::to_yaml(goal, true);
        RCLCPP_INFO(get_logger(), "%s -- Payload: %s", __func__, payload.c_str());

        requestTree("Navigation", payload);
    };

    void requestTree(const std::string& treeName, const std::string& payload) const
    {
        if (!treeRequester_->wait_for_action_server())
        {
            RCLCPP_ERROR(get_logger(), "%s -- Action server not available after waiting", __func__);
            return;
        }

        auto goalMsg        = ExecuteTree::Goal();
        goalMsg.target_tree = treeName;
        goalMsg.payload     = payload;

        RCLCPP_INFO(get_logger(), "%s -- Sending goal", __func__);

        auto sendGoalOptions                   = rclcpp_action::Client<ExecuteTree>::SendGoalOptions();
        sendGoalOptions.goal_response_callback = [this](const ExecuteTreeGoalHandle::SharedPtr& goalHandle)
        {
            if (!goalHandle)
            {
                RCLCPP_ERROR(get_logger(), "%s -- Goal was rejected by server", __func__);
            }
            else
            {
                RCLCPP_INFO(get_logger(), "%s -- Goal accepted by server, waiting for result", __func__);
            }
        };

        sendGoalOptions.feedback_callback =
            [this](ExecuteTreeGoalHandle::SharedPtr, const std::shared_ptr<const ExecuteTree::Feedback> feedback)
        {
            RCLCPP_INFO(get_logger(), "%s -- Feedback received: %s", __func__, feedback->message.c_str());
        };

        sendGoalOptions.result_callback = [this](const ExecuteTreeGoalHandle::WrappedResult& result)
        {
            switch (result.code)
            {
                case rclcpp_action::ResultCode::SUCCEEDED: break;
                case rclcpp_action::ResultCode::ABORTED:   RCLCPP_ERROR(get_logger(), "Goal was aborted"); return;
                case rclcpp_action::ResultCode::CANCELED:  RCLCPP_ERROR(get_logger(), "Goal was canceled"); return;
                default:                                   RCLCPP_ERROR(get_logger(), "Unknown result code"); return;
            }
        };

        treeRequester_->async_send_goal(goalMsg, sendGoalOptions);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<TurtleBrain>());
    rclcpp::shutdown();

    return 0;
}
