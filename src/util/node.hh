#pragma once

#include "util/parameter.hh"

#include <rclcpp/node.hpp>

namespace rmcs::util {

inline auto make_simple_node(
    const std::string& name, const rclcpp::NodeOptions& options = util::NodeOptions {})
    -> std::shared_ptr<rclcpp::Node> {
    return std::make_shared<rclcpp::Node>(name, options);
}

} // namespace rmcs::util
