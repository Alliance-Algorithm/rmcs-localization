#include "runtime/runtime.hpp"
#include "util/parameter.hpp"

#include <rclcpp/rclcpp.hpp>

struct LocalizationNode : rclcpp::Node {
    rmcs::Runtime runtime {};

    LocalizationNode()
        : rclcpp::Node("rmcs_localization", rmcs::util::NodeOptions {}) {
        runtime.initialize(*this);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
