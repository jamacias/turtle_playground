#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/tree_node.h>
#include <behaviortree_ros2/tree_execution_server.hpp>
#include <optional>
#include <rclcpp/logging.hpp>

using namespace BT;

class ActionServer : public BT::TreeExecutionServer
{
public:
    ActionServer(const rclcpp::NodeOptions& options)
    : TreeExecutionServer(options)
    {
    }

    void onTreeCreated(BT::Tree& tree) override
    {
        loggerCout_ = std::make_shared<BT::StdCoutLogger>(tree);


        RCLCPP_INFO(node()->get_logger(), "Payload: '%s'", goalPayload().c_str());
        tree.rootBlackboard()->set("payload", goalPayload());

        std::string dummy{"????"};
        const auto rc = tree.rootBlackboard()->get("payload", dummy);
        RCLCPP_INFO(node()->get_logger(), "Payload in the blackboard: '%s' (%i)", dummy.c_str(), rc);
    }

    std::optional<std::string> onTreeExecutionCompleted([[maybe_unused]] BT::NodeStatus status,
                                                        [[maybe_unused]] bool           was_cancelled) override
    {
        // NOT really needed, even if loggerCout_ may contain a dangling pointer of the tree
        // at this point
        loggerCout_.reset();
        return std::nullopt;
    }

private:
    std::shared_ptr<BT::StdCoutLogger> loggerCout_;
    std::string                        payload_{};
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    auto                action_server = std::make_shared<ActionServer>(options);

    // TODO: This workaround is for a bug in MultiThreadedExecutor where it can deadlock when spinning without a timeout.
    // Deadlock is caused when Publishers or Subscribers are dynamically removed as the node is spinning.
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 0, false, std::chrono::milliseconds(250));
    exec.add_node(action_server->node());
    exec.spin();
    exec.remove_node(action_server->node());

    rclcpp::shutdown();
}
