// bt_nodes.hpp - custom BehaviorTree.CPP v4 nodes for the search & rescue task.
//
// Nodes:
//   NavigateTo    (StatefulAction)  - drive to (x,y) via Nav2 navigate_to_pose
//   IsAlive       (Condition)       - SUCCESS if status port == "ALIVE"
//   DeployKit     (StatefulAction)  - print + hold 5 s (rescue-kit deployment)
//   RetrieveBody  (SyncAction)      - print body-retrieval message
//   SaySomething  (SyncAction)      - print a message
//
// NavigateTo and DeployKit need the ROS node (action client / sim clock), so
// they are registered with a builder lambda that injects it.

#ifndef PIO_MAZE_BT_NODES_HPP
#define PIO_MAZE_BT_NODES_HPP

#include <atomic>
#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace pio_maze
{

// ----------------------------------------------------------------------------
class NavigateTo : public BT::StatefulActionNode
{
public:
  using Action = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;

  NavigateTo(const std::string & name, const BT::NodeConfig & cfg,
             rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, cfg), node_(node)
  {
    client_ = rclcpp_action::create_client<Action>(node_, "navigate_to_pose");
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("x"), BT::InputPort<double>("y"),
            BT::InputPort<std::string>("loc")};
  }

  BT::NodeStatus onStart() override
  {
    double x = 0.0, y = 0.0;
    getInput("x", x);
    getInput("y", y);
    std::string loc = "?";
    getInput("loc", loc);
    loc_ = loc;

    if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(node_->get_logger(), "navigate_to_pose server not available");
      return BT::NodeStatus::FAILURE;
    }

    Action::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = node_->now();
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.orientation.w = 1.0;

    done_ = false;
    succeeded_ = false;
    goal_handle_.reset();

    auto opts = rclcpp_action::Client<Action>::SendGoalOptions();
    opts.goal_response_callback = [this](GoalHandle::SharedPtr h) {
      if (!h) {done_ = true; succeeded_ = false;} else {goal_handle_ = h;}
    };
    opts.result_callback = [this](const GoalHandle::WrappedResult & r) {
      done_ = true;
      succeeded_ = (r.code == rclcpp_action::ResultCode::SUCCEEDED);
    };
    client_->async_send_goal(goal, opts);
    RCLCPP_INFO(node_->get_logger(), "[NavigateTo] -> %s (%.2f, %.2f)",
                loc_.c_str(), x, y);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (!done_) {return BT::NodeStatus::RUNNING;}
    if (succeeded_) {
      RCLCPP_INFO(node_->get_logger(), "[NavigateTo] reached %s", loc_.c_str());
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_WARN(node_->get_logger(), "[NavigateTo] failed to reach %s",
                loc_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  void onHalted() override
  {
    if (goal_handle_) {client_->async_cancel_goal(goal_handle_);}
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<Action>::SharedPtr client_;
  GoalHandle::SharedPtr goal_handle_;
  std::atomic<bool> done_{false};
  std::atomic<bool> succeeded_{false};
  std::string loc_;
};

// ----------------------------------------------------------------------------
class IsAlive : public BT::ConditionNode
{
public:
  IsAlive(const std::string & name, const BT::NodeConfig & cfg)
  : BT::ConditionNode(name, cfg) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("status")};
  }

  BT::NodeStatus tick() override
  {
    std::string status = "";
    getInput("status", status);
    return (status == "ALIVE") ? BT::NodeStatus::SUCCESS
                               : BT::NodeStatus::FAILURE;
  }
};

// ----------------------------------------------------------------------------
class DeployKit : public BT::StatefulActionNode
{
public:
  DeployKit(const std::string & name, const BT::NodeConfig & cfg,
            rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, cfg), node_(node) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("loc"),
            BT::InputPort<double>("seconds")};
  }

  BT::NodeStatus onStart() override
  {
    std::string loc = "?";
    getInput("loc", loc);
    secs_ = 5.0;
    getInput("seconds", secs_);
    t0_ = node_->now();
    RCLCPP_INFO(node_->get_logger(),
                "ALIVE VICTIM FOUND AT %s. DEPLOYING RESCUE KIT...",
                loc.c_str());
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if ((node_->now() - t0_).seconds() >= secs_) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time t0_;
  double secs_{5.0};
};

// ----------------------------------------------------------------------------
class RetrieveBody : public BT::SyncActionNode
{
public:
  RetrieveBody(const std::string & name, const BT::NodeConfig & cfg,
               rclcpp::Node::SharedPtr node)
  : BT::SyncActionNode(name, cfg), node_(node) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("loc")};
  }

  BT::NodeStatus tick() override
  {
    std::string loc = "?";
    getInput("loc", loc);
    RCLCPP_INFO(node_->get_logger(),
                "DEAD VICTIM FOUND AT %s. RETRIEVING THE BODY...", loc.c_str());
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
};

// ----------------------------------------------------------------------------
class SaySomething : public BT::SyncActionNode
{
public:
  SaySomething(const std::string & name, const BT::NodeConfig & cfg,
               rclcpp::Node::SharedPtr node)
  : BT::SyncActionNode(name, cfg), node_(node) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("message")};
  }

  BT::NodeStatus tick() override
  {
    std::string msg = "";
    getInput("message", msg);
    RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
};

// ----------------------------------------------------------------------------
inline void RegisterRescueNodes(BT::BehaviorTreeFactory & factory,
                                rclcpp::Node::SharedPtr node)
{
  factory.registerBuilder<NavigateTo>(
    "NavigateTo",
    [node](const std::string & n, const BT::NodeConfig & c) {
      return std::make_unique<NavigateTo>(n, c, node);
    });
  factory.registerBuilder<DeployKit>(
    "DeployKit",
    [node](const std::string & n, const BT::NodeConfig & c) {
      return std::make_unique<DeployKit>(n, c, node);
    });
  factory.registerBuilder<RetrieveBody>(
    "RetrieveBody",
    [node](const std::string & n, const BT::NodeConfig & c) {
      return std::make_unique<RetrieveBody>(n, c, node);
    });
  factory.registerBuilder<SaySomething>(
    "SaySomething",
    [node](const std::string & n, const BT::NodeConfig & c) {
      return std::make_unique<SaySomething>(n, c, node);
    });
  factory.registerNodeType<IsAlive>("IsAlive");
}

}  // namespace pio_maze

#endif  // PIO_MAZE_BT_NODES_HPP
