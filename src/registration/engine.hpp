#pragma once

#include "util/pimpl.hpp"

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

    void initialize(rclcpp::Node& node);

    void register_map(const std::shared_ptr<PointCloudT>& map);
    void register_scan(const std::shared_ptr<PointCloudT>& scan);

    void full_match(const std::shared_ptr<PointCloudT>& align);
    void full_match(const std::shared_ptr<PointCloudT>& align, const Eigen::Isometry3f& transformation);

    double fitness_score() const;
    Eigen::Isometry3f transformation() const;
};

} // namespace rmcs
