#include "util/segmentation.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <numbers>

namespace {

constexpr auto kSegmentSizeLimit = std::size_t { 1'000 };

}

struct Segmentation::Impl {
    std::shared_ptr<PointCloud> source;
    double limit_distance = 10.0;
    double limit_max_height = 2.0;
    double segmentation_point_distance = 0.01;
    double ground_height = 0.2;
    pcl::SACSegmentation<PointCloud::PointType> segmentation;
};

Segmentation::Segmentation()
    : pimpl(std::make_unique<Impl>()) {
    auto& segmentation = pimpl->segmentation;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setAxis(Eigen::Vector3f::UnitZ());
    segmentation.setEpsAngle(std::numbers::pi / 180.0 * 10.0);
    segmentation.setDistanceThreshold(pimpl->segmentation_point_distance);
    segmentation.setMaxIterations(1000);
}

Segmentation::~Segmentation() = default;

void Segmentation::set_input_source(const std::shared_ptr<PointCloud>& source) { pimpl->source = source; }

std::shared_ptr<Segmentation::PointCloud> Segmentation::execute() {
    auto outside_condition = std::make_shared<pcl::ConditionAnd<PointCloud::PointType>>();
    outside_condition->addComparison(std::make_shared<const pcl::FieldComparison<PointCloud::PointType>>(
        "x", pcl::ComparisonOps::GT, -pimpl->limit_distance / 2.0));
    outside_condition->addComparison(std::make_shared<const pcl::FieldComparison<PointCloud::PointType>>(
        "x", pcl::ComparisonOps::LT, pimpl->limit_distance / 2.0));
    outside_condition->addComparison(std::make_shared<const pcl::FieldComparison<PointCloud::PointType>>(
        "y", pcl::ComparisonOps::GT, -pimpl->limit_distance / 2.0));
    outside_condition->addComparison(std::make_shared<const pcl::FieldComparison<PointCloud::PointType>>(
        "y", pcl::ComparisonOps::LT, pimpl->limit_distance / 2.0));
    outside_condition->addComparison(std::make_shared<const pcl::FieldComparison<PointCloud::PointType>>(
        "z", pcl::ComparisonOps::LT, pimpl->limit_max_height));

    pcl::ConditionalRemoval<PointCloud::PointType> removal;
    removal.setCondition(outside_condition);
    removal.setInputCloud(pimpl->source);

    auto pointcloud_removed_xy = std::make_shared<PointCloud>();
    removal.filter(*pointcloud_removed_xy);

    auto pass_through = pcl::PassThrough<PointCloud::PointType> {};
    pass_through.setInputCloud(pointcloud_removed_xy);
    pass_through.setFilterFieldName("z");
    pass_through.setFilterLimits(-1.0f, static_cast<float>(pimpl->ground_height));

    auto pointcloud_removed_xyz = std::make_shared<PointCloud>();
    auto indices_removed_xyz = std::make_shared<pcl::PointIndices>();
    pass_through.filter(*pointcloud_removed_xyz);
    pass_through.filter(indices_removed_xyz->indices);

    if (pointcloud_removed_xyz->size() < kSegmentSizeLimit) {
        return pointcloud_removed_xyz;
    }

    auto coefficients = std::make_shared<pcl::ModelCoefficients>();
    auto plane_points = std::make_shared<pcl::PointIndices>();
    pimpl->segmentation.setInputCloud(pointcloud_removed_xyz);
    pimpl->segmentation.segment(*plane_points, *coefficients);

    auto indices_original = std::make_shared<pcl::PointIndices>();
    for (const auto index : plane_points->indices) {
        if (index >= 0 && index < static_cast<int>(indices_removed_xyz->indices.size())) {
            indices_original->indices.push_back(indices_removed_xyz->indices[static_cast<std::size_t>(index)]);
        }
    }

    auto extract = pcl::ExtractIndices<PointCloud::PointType> {};
    extract.setInputCloud(pointcloud_removed_xy);
    extract.setIndices(indices_original);
    extract.setNegative(true);

    auto output = std::make_shared<PointCloud>();
    extract.filter(*output);
    return output;
}

void Segmentation::set_limit_distance(double v) { pimpl->limit_distance = v; }
void Segmentation::set_limit_max_height(double v) { pimpl->limit_max_height = v; }
void Segmentation::set_distance_threshold(double v) { pimpl->segmentation_point_distance = v; }
void Segmentation::set_ground_max_height(double v) { pimpl->ground_height = v; }
