// Copyright (C) 2026 Cedric <cedric@mowgli.dev>

#include <memory>

#include "mowgli_coverage/coverage_server.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mowgli_coverage::CoverageServer>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
