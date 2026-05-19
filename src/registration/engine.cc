#include "registration/engine.hh"
#include "util/parameter.hh"

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <small_gicp/pcl/pcl_registration.hpp>

namespace rmcs {

using namespace rmcs;

struct Registration::Impl {
public:
    using PointCloudT = pcl::PointCloud<PointT>;

    auto preprocess(const std::shared_ptr<PointCloudT>& cloud) -> void {
        voxel_grid_filter->setInputCloud(cloud);
        voxel_grid_filter->filter(*cloud);
        outlier_removal_filter->setInputCloud(cloud);
        outlier_removal_filter->filter(*cloud);
    }

    auto initialize(rclcpp::Node& node) -> void {
        auto p = util::quick_paramtetr_reader{node};

        registration = std::make_unique<small_gicp::RegistrationPCL<PointT, PointT>>();
        registration->setRegistrationType("GICP");
        registration->setNumThreads(p("registration.threads", int{}));
        registration->setMaxCorrespondenceDistance(p("registration.distance_threshold", double{}));
        registration->setCorrespondenceRandomness(20);
        registration->setMaximumIterations(
            coarse_iterations = p("registration.coarse_iterations", int{}));

        outlier_removal_filter = std::make_unique<pcl::StatisticalOutlierRemoval<PointT>>();
        outlier_removal_filter->setMeanK(p("registration.outlier_removal.mean_k", int{}));
        outlier_removal_filter->setStddevMulThresh(
            p("registration.outlier_removal.stddev_mul_thresh", double{}));

        voxel_grid_filter = std::make_unique<pcl::VoxelGrid<PointT>>();
        voxel_grid_filter->setLeafSize(
            p("registration.voxel_grid.lx", float{}), p("registration.voxel_grid.ly", float{}),
            p("registration.voxel_grid.lz", float{}));

        scan_angle = p("registration.scan_angle", int{});
        precise_iterations = p("registration.precise_iterations", int{});
        score_threshold = p("registration.score_threshold", double{});
    }

    auto register_map(const std::shared_ptr<PointCloudT>& map) -> void {
        preprocess(map);
        registration->setInputTarget(map);
        last_score = 1.0;
    }

    auto register_scan(const std::shared_ptr<PointCloudT>& scan) -> void {
        preprocess(scan);
        registration->setInputSource(scan);
        last_score = 1.0;
    }

    auto full_match(
        const std::shared_ptr<PointCloudT>& align, const Eigen::Isometry3f& transformation)
        -> void {
        auto score_min = 1.0;
        auto angle_best = 0;

        registration->setMaximumIterations(coarse_iterations);

        for (auto n = 1; (scan_angle * n / 2) < 181; ++n) {
            auto angle = static_cast<int>(scan_angle * static_cast<int>(n / 2) * std::pow(-1, n));
            auto radian =
                static_cast<float>(static_cast<float>(angle) / 180.0f * std::numbers::pi_v<float>);
            auto rotation = Eigen::AngleAxisf{radian, Eigen::Vector3f::UnitZ()};
            auto guess = Eigen::Matrix4f{(rotation * transformation).matrix()};

            registration->align(*align, guess);
            auto score = double{registration->getFitnessScore()};
            if (score < score_min) {
                score_min = score;
                angle_best = angle;
            }
            if (score < score_threshold) {
                break;
            }
        }

        registration->setMaximumIterations(precise_iterations);

        auto radian =
            static_cast<float>(static_cast<float>(angle_best) / 180.0f * std::numbers::pi_v<float>);
        auto rotation = Eigen::AngleAxisf{radian, Eigen::Vector3f::UnitZ()};
        auto guess = Eigen::Matrix4f{(rotation * transformation).matrix()};

        registration->align(*align, guess);
        last_score = registration->getFitnessScore();
    }

    auto fitness_score() const -> double { return last_score; }

    auto transformation() const -> Eigen::Isometry3f {
        return Eigen::Isometry3f{registration->getFinalTransformation()};
    }

private:
    std::unique_ptr<small_gicp::RegistrationPCL<PointT, PointT>> registration;
    std::unique_ptr<pcl::StatisticalOutlierRemoval<PointT>> outlier_removal_filter;
    std::unique_ptr<pcl::VoxelGrid<PointT>> voxel_grid_filter;
    int scan_angle{};
    int coarse_iterations{};
    int precise_iterations{};
    double score_threshold{};
    double last_score{1.0};
};

Registration::Registration()
    : pimpl(std::make_unique<Impl>()) {}

Registration::~Registration() = default;

auto Registration::initialize(rclcpp::Node& node) -> void { pimpl->initialize(node); }
auto Registration::register_map(const std::shared_ptr<PointCloudT>& map) -> void {
    pimpl->register_map(map);
}
auto Registration::register_scan(const std::shared_ptr<PointCloudT>& scan) -> void {
    pimpl->register_scan(scan);
}
auto Registration::full_match(const std::shared_ptr<PointCloudT>& align) -> void {
    pimpl->full_match(align, Eigen::Isometry3f::Identity());
}
auto Registration::full_match(
    const std::shared_ptr<PointCloudT>& align, const Eigen::Isometry3f& transformation) -> void {
    pimpl->full_match(align, transformation);
}
auto Registration::fitness_score() const -> double { return pimpl->fitness_score(); }
auto Registration::transformation() const -> Eigen::Isometry3f { return pimpl->transformation(); }

} // namespace rmcs
