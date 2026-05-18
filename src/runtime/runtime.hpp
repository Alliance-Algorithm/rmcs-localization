#pragma once

#include "util/pimpl.hpp"

#include <rclcpp/node.hpp>

namespace rmcs {

class Runtime {
    RMCS_PIMPL_DEFINTION(Runtime)

public:
    void initialize(rclcpp::Node& node);
};

} // namespace rmcs
