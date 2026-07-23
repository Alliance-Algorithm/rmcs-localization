#pragma once

#include <rclcpp/node.hpp>

namespace rmcs::util {

struct NodeOptions : rclcpp::NodeOptions {
    explicit NodeOptions() { automatically_declare_parameters_from_overrides(true); }
};

struct quick_paramtetr_reader {
public:
    explicit quick_paramtetr_reader(rclcpp::Node& node)
        : node(node) {
        if (!node.get_node_options().automatically_declare_parameters_from_overrides()) {
            throw std::runtime_error{"NodeOptions must auto-declare overridden parameters"};
        }
    }

    template <typename T>
    auto operator()(const std::string& name, T = {}) const -> T {
        return get_parameter<T>(name);
    }

    template <typename T>
    auto get_parameter(const std::string& name) const -> T {
        auto value = T{};
        node.get_parameter(name, value);
        return value;
    }

private:
    rclcpp::Node& node;
};

} // namespace rmcs::util
