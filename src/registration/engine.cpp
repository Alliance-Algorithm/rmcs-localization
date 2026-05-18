#include "registration/engine.hpp"

#include "util/logger.hpp"
#include "util/parameter.hpp"

#include <small_gicp/pcl/pcl_registration.hpp>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <numbers>

namespace rmcs {

class Registration::Impl {
public:
    using PointCloudT = pcl::PointCloud<PointT>;

    void initialize(rclcpp::Node& node) {
        const auto p = util::quick_paramtetr_reader { node };

        registration = std::make_unique<small_gicp::RegistrationPCL<PointT, PointT>>();
        registration->setRegistrationType("GICP");
        registration->setNumThreads(p("registration.threads", int {}));
        registration->setMaxCorrespondenceDistance(p("registration.distance_threshold", double {}));
        registration->setCorrespondenceRandomness(20);
        registration->setMaximumIterations(coarse_iterations = p("registration.coarse_iterations", int {}));

        outlier_removal_filter = std::make_unique<pcl::StatisticalOutlierRemoval<PointT>>();
        outlier_removal_filter->setMeanK(p("registration.outlier_removal.mean_k", int {}));
        outlier_removal_filter->setStddevMulThresh(
            p("registration.outlier_removal.stddev_mul_thresh", double {}));

        voxel_grid_filter = std::make_unique<pcl::VoxelGrid<PointT>>();
        voxel_grid_filter->setLeafSize(p("registration.voxel_grid.lx", float {}),
            p("registration.voxel_grid.ly", float {}), p("registration.voxel_grid.lz", float {}));

        scan_angle = p("registration.scan_angle", int {});
        precise_iterations = p("registration.precise_iterations", int {});
        score_threshold = p("registration.score_threshold", double {});
    }

    void register_map(const std::shared_ptr<PointCloudT>& map) {
        voxel_grid_filter->setInputCloud(map);
        voxel_grid_filter->filter(*map);
        outlier_removal_filter->setInputCloud(map);
        outlier_removal_filter->filter(*map);
        registration->setInputTarget(map);
        last_score = 1.0;
    }

    void register_scan(const std::shared_ptr<PointCloudT>& scan) {
        voxel_grid_filter->setInputCloud(scan);
        voxel_grid_filter->filter(*scan);
        registration->setInputSource(scan);
        last_score = 1.0;
    }

    void full_match(const std::shared_ptr<PointCloudT>& align, const Eigen::Isometry3f& transformation) {
        double score_min = 1.0;
        int angle_best = 0;

        registration->setMaximumIterations(coarse_iterations);

        for (auto n = 1; (scan_angle * n / 2) < 181; ++n) {
            const auto angle = static_cast<int>(scan_angle * static_cast<int>(n / 2) * std::pow(-1, n));
            const auto radian = static_cast<float>(static_cast<float>(angle) / 180.0f * std::numbers::pi_v<float>);
            const auto rotation = Eigen::AngleAxisf(radian, Eigen::Vector3f::UnitZ());
            const auto guess = (rotation * transformation).matrix();

            registration->align(*align, guess);
            const auto score = registration->getFitnessScore();
            if (score < score_min) {
                score_min = score;
                angle_best = angle;
            }
            if (score < score_threshold) {
                break;
            }
        }

        registration->setMaximumIterations(precise_iterations);

        const auto radian = static_cast<float>(static_cast<float>(angle_best) / 180.0f * std::numbers::pi_v<float>);
        const auto rotation = Eigen::AngleAxisf(radian, Eigen::Vector3f::UnitZ());
        const auto guess = (rotation * transformation).matrix();

        registration->align(*align, guess);
        last_score = registration->getFitnessScore();
    }

    double fitness_score() const { return last_score; }

    Eigen::Isometry3f transformation() const {
        return Eigen::Isometry3f { registration->getFinalTransformation() };
    }

private:
    std::unique_ptr<small_gicp::RegistrationPCL<PointT, PointT>> registration;
    std::unique_ptr<pcl::StatisticalOutlierRemoval<PointT>> outlier_removal_filter;
    std::unique_ptr<pcl::VoxelGrid<PointT>> voxel_grid_filter;
    int scan_angle {};
    int coarse_iterations {};
    int precise_iterations {};
    double score_threshold {};
    double last_score { 1.0 };
};

Registration::Registration()
    : pimpl(std::make_unique<Impl>()) { }

Registration::~Registration() = default;

void Registration::initialize(rclcpp::Node& node) { pimpl->initialize(node); }
void Registration::register_map(const std::shared_ptr<PointCloudT>& map) { pimpl->register_map(map); }
void Registration::register_scan(const std::shared_ptr<PointCloudT>& scan) { pimpl->register_scan(scan); }
void Registration::full_match(const std::shared_ptr<PointCloudT>& align) {
    pimpl->full_match(align, Eigen::Isometry3f::Identity());
}
void Registration::full_match(
    const std::shared_ptr<PointCloudT>& align, const Eigen::Isometry3f& transformation) {
    pimpl->full_match(align, transformation);
}
double Registration::fitness_score() const { return pimpl->fitness_score(); }
Eigen::Isometry3f Registration::transformation() const { return pimpl->transformation(); }

} // namespace rmcs
