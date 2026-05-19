#pragma once

#include "util/pimpl.hh"

#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace rmcs {

class Segmentation {
    RMCS_PIMPL_DEFINTION(Segmentation)

public:
    using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

    auto set_input_source(const std::shared_ptr<PointCloud>& source) -> void;
    auto execute() -> std::shared_ptr<PointCloud>;

    auto set_limit_distance(double v) -> void;
    auto set_limit_max_height(double v) -> void;
    auto set_distance_threshold(double v) -> void;
    auto set_ground_max_height(double v) -> void;
};

} // namespace rmcs
