
#include <chrono>
#include <memory>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

#include "pio_maze/bt_nodes.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("rescue_bt");

  // resolve tree path: ROS param 'bt_xml' overrides the packaged default
  std::string default_xml =
    ament_index_cpp::get_package_share_directory("pio_maze") +
    "/behavior_trees/rescue.xml";
  std::string xml_path = node->declare_parameter<std::string>("bt_xml", "");
  if (xml_path.empty()) {
    xml_path = default_xml;
  }
  RCLCPP_INFO(node->get_logger(), "Loading BT: %s", xml_path.c_str());

  BT::BehaviorTreeFactory factory;
  pio_maze::RegisterRescueNodes(factory, node);

  BT::Tree tree;
  try {
    tree = factory.createTreeFromFile(xml_path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to load BT: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Starting search & rescue behavior tree.");
  rclcpp::Rate rate(10);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    status = tree.tickOnce();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "Behavior tree finished with status: %s",
              status == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE");
  rclcpp::shutdown();
  return 0;
}
