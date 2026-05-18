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
            throw std::runtime_error { "NodeOptions must auto-declare overridden parameters" };
        }
    }

    template <typename T>
    T operator()(const std::string& name, T just_send_a_type_using_default_constructor = {}) const {
        (void)just_send_a_type_using_default_constructor;
        return get_parameter<T>(name);
    }

    template <typename T>
    T get_parameter(const std::string& name) const {
        auto value = T {};
        node.get_parameter(name, value);
        return value;
    }

private:
    rclcpp::Node& node;
};

} // namespace rmcs::util
