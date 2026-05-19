#pragma once

#include "util/pimpl.hh"

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/node.hpp>

namespace rmcs {

class Registration final {
    RMCS_PIMPL_DEFINTION(Registration)

public:
    using PointT = pcl::PointXYZ;
    using PointCloudT = pcl::PointCloud<PointT>;

    auto initialize(rclcpp::Node& node) -> void;

    auto register_map(const std::shared_ptr<PointCloudT>& map) -> void;
    auto register_scan(const std::shared_ptr<PointCloudT>& scan) -> void;

    auto full_match(const std::shared_ptr<PointCloudT>& align) -> void;
    auto full_match(
        const std::shared_ptr<PointCloudT>& align, const Eigen::Isometry3f& transformation) -> void;

    auto fitness_score() const -> double;
    auto transformation() const -> Eigen::Isometry3f;
};

} // namespace rmcs
