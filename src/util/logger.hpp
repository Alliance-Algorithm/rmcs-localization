#pragma once

#include <Eigen/Geometry>
#include <rclcpp/logging.hpp>
#include <stdexcept>
#include <format>

#ifdef __clang__
#pragma clang diagnostic ignored "-Wformat-security"
#endif

#define RMCS_INITIALIZE_LOGGER(NAME)                                                               \
    rclcpp::Logger _internal_logger { rclcpp::get_logger(std::string(NAME)) };                     \
    inline void rclcpp_info(auto&&... args) const { RCLCPP_INFO(_internal_logger, args...); }      \
    inline void rclcpp_warn(auto&&... args) const { RCLCPP_WARN(_internal_logger, args...); }      \
    inline void rclcpp_error(auto&&... args) const { RCLCPP_ERROR(_internal_logger, args...); }

namespace rmcs::util {

template <auto name_getter>
struct Log {
    static inline rclcpp::Logger log { rclcpp::get_logger(name_getter()) };
    static inline void info(auto&&... args) { RCLCPP_INFO(log, args...); }
    static inline void warn(auto&&... args) { RCLCPP_WARN(log, args...); }
    static inline void error(auto&&... args) { RCLCPP_ERROR(log, args...); }
};

inline std::string to_string(const Eigen::Isometry3f& transform) {
    const auto t = Eigen::Vector3f { transform.translation() };
    const auto q = Eigen::Quaternionf { transform.rotation() };
    return std::format("t({:.2f} {:.2f} {:.2f}) q({:.2f} {:.2f} {:.2f} {:.2f})", t.x(), t.y(),
        t.z(), q.w(), q.x(), q.y(), q.z());
}

inline std::runtime_error runtime_error(const std::string& text) {
    return std::runtime_error { text };
}

} // namespace rmcs::util
