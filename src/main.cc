#include "util/convert.hh"
#include "util/logger.hh"
#include "util/parameter.hh"

#include <cmath>
#include <numbers>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/factors/icp_factor.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

namespace rmcs {

// 配准引擎：预处理（体素 + 离群点）+ ICP/GICP 配准 + 绕初值的 yaw 粗搜
// ICP 使用 small_gicp 核心实现（多线程），GICP/VGICP 使用其 PCL 封装
class Registration {
public:
    using PointT = pcl::PointXYZ;
    using PointCloudT = pcl::PointCloud<PointT>;
    using Icp = small_gicp::Registration<small_gicp::ICPFactor, small_gicp::ParallelReductionOMP>;

    auto initialize(rclcpp::Node& node) -> void {
        auto p = util::quick_paramtetr_reader{node};

        type = p("registration.type", std::string{"ICP"});
        distance_threshold = p("registration.distance_threshold", double{});
        threads = p("registration.threads", int{});
        coarse_iterations = p("registration.coarse_iterations", int{});
        precise_iterations = p("registration.precise_iterations", int{});
        scan_angle = p("registration.scan_angle", int{});
        score_threshold = p("registration.score_threshold", double{});

        if (type == "ICP") {
            // point-to-point ICP：不利用平面结构，平行墙/地面对其无退化
            icp = std::make_unique<Icp>();
            icp->reduction.num_threads = threads;
            icp->rejector.max_dist_sq = distance_threshold * distance_threshold;
            icp->optimizer.max_iterations = coarse_iterations;
        } else {
            auto gicp = std::make_unique<small_gicp::RegistrationPCL<PointT, PointT>>();
            gicp->setRegistrationType(type);
            gicp->setNumThreads(threads);
            gicp->setCorrespondenceRandomness(20);
            gicp->setMaxCorrespondenceDistance(distance_threshold);
            gicp->setMaximumIterations(coarse_iterations);
            registration = std::move(gicp);
        }

        outlier_removal_filter = std::make_unique<pcl::StatisticalOutlierRemoval<PointT>>();
        outlier_removal_filter->setMeanK(p("registration.outlier_removal.mean_k", int{}));
        outlier_removal_filter->setStddevMulThresh(
            p("registration.outlier_removal.stddev_mul_thresh", double{}));

        voxel_grid_filter = std::make_unique<pcl::VoxelGrid<PointT>>();
        voxel_grid_filter->setLeafSize(
            p("registration.voxel_grid.lx", float{}), p("registration.voxel_grid.ly", float{}),
            p("registration.voxel_grid.lz", float{}));
    }

    auto register_map(const std::shared_ptr<PointCloudT>& map) -> void {
        preprocess(map);
        if (icp) {
            icp_target = map;
            icp_target_tree = std::make_shared<small_gicp::KdTree<PointCloudT>>(
                map, small_gicp::KdTreeBuilderOMP(threads));
        } else {
            registration->setInputTarget(map);
        }
    }

    auto register_scan(const std::shared_ptr<PointCloudT>& scan) -> void {
        preprocess(scan);
        if (icp) {
            icp_source = scan;
        } else {
            registration->setInputSource(scan);
        }
    }

    // 绕初值位置做 yaw 粗搜，再用最优角度精配准
    auto full_match(
        const std::shared_ptr<PointCloudT>& align, const Eigen::Isometry3f& transformation)
        -> void {
        auto score_min = std::numeric_limits<double>::max();
        auto angle_best = 0;

        set_iterations(coarse_iterations);

        for (auto n = 1; (scan_angle * n / 2) < 181; ++n) {
            auto angle = static_cast<int>(scan_angle * static_cast<int>(n / 2) * std::pow(-1, n));
            auto radian =
                static_cast<float>(static_cast<float>(angle) / 180.0f * std::numbers::pi_v<float>);
            auto rotation = Eigen::AngleAxisf{radian, Eigen::Vector3f::UnitZ()};
            auto guess = Eigen::Isometry3d{(transformation * rotation).matrix().cast<double>()};

            const auto [score, result] = align_and_score(align, guess);
            if (score < score_min) {
                score_min = score;
                angle_best = angle;
                last_transformation = Eigen::Isometry3f{result.matrix().cast<float>()};
            }
            if (score < score_threshold) {
                break;
            }
        }

        set_iterations(precise_iterations);

        auto radian =
            static_cast<float>(static_cast<float>(angle_best) / 180.0f * std::numbers::pi_v<float>);
        auto rotation = Eigen::AngleAxisf{radian, Eigen::Vector3f::UnitZ()};
        auto guess = Eigen::Isometry3d{(transformation * rotation).matrix().cast<double>()};

        const auto [score, result] = align_and_score(align, guess);
        last_score = score;
        last_transformation = Eigen::Isometry3f{result.matrix().cast<float>()};
    }

    auto fitness_score() const -> double { return last_score; }

    auto transformation() const -> Eigen::Isometry3f { return last_transformation; }

private:
    // 返回 {fitness(与 PCL 同语义：内点均方距离), 位姿}
    auto align_and_score(const std::shared_ptr<PointCloudT>& align, const Eigen::Isometry3d& guess)
        -> std::pair<double, Eigen::Isometry3d> {
        if (icp) {
            const auto result = icp->align(*icp_target, *icp_source, *icp_target_tree, guess);
            const auto score = result.num_inliers > 0
                ? 2.0 * result.error / static_cast<double>(result.num_inliers)
                : std::numeric_limits<double>::max();
            return {score, result.T_target_source};
        }
        registration->align(*align, Eigen::Matrix4f{guess.matrix().cast<float>()});
        return {
            registration->getFitnessScore(),
            Eigen::Isometry3d{registration->getFinalTransformation().cast<double>()}};
    }

    auto set_iterations(int iterations) -> void {
        if (icp) {
            icp->optimizer.max_iterations = iterations;
        } else {
            registration->setMaximumIterations(iterations);
        }
    }

    auto preprocess(const std::shared_ptr<PointCloudT>& cloud) -> void {
        voxel_grid_filter->setInputCloud(cloud);
        voxel_grid_filter->filter(*cloud);
        outlier_removal_filter->setInputCloud(cloud);
        outlier_removal_filter->filter(*cloud);
    }

    std::string type;
    std::unique_ptr<pcl::Registration<PointT, PointT, float>> registration;
    std::unique_ptr<Icp> icp;
    std::shared_ptr<PointCloudT> icp_target;
    std::shared_ptr<PointCloudT> icp_source;
    std::shared_ptr<small_gicp::KdTree<PointCloudT>> icp_target_tree;
    std::unique_ptr<pcl::StatisticalOutlierRemoval<PointT>> outlier_removal_filter;
    std::unique_ptr<pcl::VoxelGrid<PointT>> voxel_grid_filter;
    double distance_threshold{};
    int threads{};
    int scan_angle{};
    int coarse_iterations{};
    int precise_iterations{};
    double score_threshold{};
    double last_score{1.0};
    Eigen::Isometry3f last_transformation{Eigen::Isometry3f::Identity()};
};

struct LocalizationNode : rclcpp::Node {
    using Point = pcl::PointXYZ;
    using PointCloud = pcl::PointCloud<Point>;
    using log = util::Log<[] { return "rmcs_localization"; }>;

    struct Config {
        std::string world_frame;
        std::string odom_frame;

        std::string pointcloud_topic;
        std::string map_path;

        double registration_radius;
        double accept_score_threshold;

        Eigen::Isometry3f initial_world_from_odom_red;
        Eigen::Isometry3f initial_world_from_odom_blue;

        auto initial_world_to_odom(std::string_view mode) const -> const Eigen::Isometry3f& {
            return mode == "red" ? initial_world_from_odom_red : initial_world_from_odom_blue;
        }

        explicit Config(rclcpp::Node& node) {
            auto p = util::quick_paramtetr_reader{node};

            world_frame = p("frames.world", std::string{});
            odom_frame = p("frames.odom", std::string{});

            pointcloud_topic = p("subscription.pointcloud", std::string{});
            map_path = p("map_path", std::string{});

            registration_radius = p("registration.initial_map_radius", double{});
            accept_score_threshold = p("registration.accept_score_threshold", double{});

            initial_world_from_odom_red =
                read_transform(node, "initial_world_to_odom.red.t", "initial_world_to_odom.red.q");
            initial_world_from_odom_blue = read_transform(
                node, "initial_world_to_odom.blue.t", "initial_world_to_odom.blue.q");
        }

    private:
        static auto read_transform(
            rclcpp::Node& node, const char* translation_key, const char* orientation_key)
            -> Eigen::Isometry3f {
            auto p = util::quick_paramtetr_reader{node};
            auto translation_values = p(translation_key, std::vector<double>{});
            auto orientation_values = p(orientation_key, std::vector<double>{});
            auto translation = Eigen::Translation3f{
                static_cast<float>(translation_values[0]),
                static_cast<float>(translation_values[1]),
                static_cast<float>(translation_values[2]),
            };
            auto orientation =
                Eigen::Quaternionf{
                    static_cast<float>(orientation_values[0]),
                    static_cast<float>(orientation_values[1]),
                    static_cast<float>(orientation_values[2]),
                    static_cast<float>(orientation_values[3]),
                }
                    .normalized();
            return translation * orientation;
        }
    };

    static constexpr auto kCollectDuration = std::chrono::duration<double>{2.0};

    Registration registration{};
    Config config;

    rclcpp::TimerBase::SharedPtr runtime_timer;
    rclcpp::TimerBase::SharedPtr debug_pointcloud_timer;

    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr origin_pointcloud_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mapped_pointcloud_publisher;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr relocalize_red_service;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr relocalize_blue_service;

    std::shared_ptr<PointCloud> standard_map = std::make_shared<PointCloud>();
    std::shared_ptr<PointCloud> latest_scan = std::make_shared<PointCloud>();
    std::shared_ptr<PointCloud> latest_origin_pointcloud = std::make_shared<PointCloud>();
    std::shared_ptr<PointCloud> latest_mapped_pointcloud = std::make_shared<PointCloud>();
    std::size_t collected_scan_frames = 0;

    // 收集状态：collecting 期间订阅回调积累点云，首帧到达才开始计时
    bool collecting = false;
    std::optional<std::chrono::steady_clock::time_point> collect_start_time;
    std::optional<Eigen::Isometry3f> initial_transform;

    LocalizationNode()
        : rclcpp::Node("rmcs_localization", rmcs::util::NodeOptions{})
        , config(*this) {
        registration.initialize(*this);

        if (pcl::io::loadPCDFile(config.map_path, *standard_map) == -1) {
            throw util::runtime_error("failed to load map pcd file");
        }

        tf_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

        origin_pointcloud_publisher = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/rmcs_localization/origin_pointcloud", 10);
        mapped_pointcloud_publisher = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/rmcs_localization/mapped_pointcloud", 10);

        pointcloud_subscription = create_subscription<sensor_msgs::msg::PointCloud2>(
            config.pointcloud_topic, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
                if (!collecting) {
                    return;
                }
                if (!collect_start_time.has_value()) {
                    collect_start_time = std::chrono::steady_clock::now();
                }

                auto cloud = std::make_shared<PointCloud>();
                pcl::fromROSMsg(*msg, *cloud);
                *latest_scan += *cloud;
                ++collected_scan_frames;
                log::info(
                    "collected scan frame #%zu: frame_points=%zu, accumulated_points=%zu",
                    collected_scan_frames, cloud->size(), latest_scan->size());
            });

        relocalize_red_service = create_service<std_srvs::srv::Trigger>(
            "/rmcs_localization/relocalize/red",
            [this](
                const std::shared_ptr<std_srvs::srv::Trigger::Request>& request,
                const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
                std::ignore = request;
                handle_relocalize_service("red", *response);
            });

        relocalize_blue_service = create_service<std_srvs::srv::Trigger>(
            "/rmcs_localization/relocalize/blue",
            [this](
                const std::shared_ptr<std_srvs::srv::Trigger::Request>& request,
                const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
                std::ignore = request;
                handle_relocalize_service("blue", *response);
            });

        log::info(
            "initialized with map=%s, pointcloud_topic=%s, frames=(%s -> %s)",
            config.map_path.c_str(), config.pointcloud_topic.c_str(), config.world_frame.c_str(),
            config.odom_frame.c_str());

        using namespace std::chrono_literals;
        runtime_timer = create_wall_timer(50ms, [this] { spin_once_runtime(); });
        debug_pointcloud_timer = create_wall_timer(1s, [this] { publish_debug_pointclouds(); });
    }

    auto
        handle_relocalize_service(std::string_view mode, std_srvs::srv::Trigger::Response& response)
            -> void {
        if (collecting) {
            log::warn(
                "ignored %.*s relocalization request because collection is in progress",
                static_cast<int>(mode.size()), mode.data());
            response.success = false;
            response.message = "relocalization already in progress";
            return;
        }

        log::info(
            "received %.*s relocalization request", static_cast<int>(mode.size()), mode.data());
        initial_transform = config.initial_world_to_odom(mode);
        latest_scan = std::make_shared<PointCloud>();
        collected_scan_frames = 0;
        collect_start_time.reset();
        collecting = true;

        response.success = true;
        response.message = std::string{mode} + " relocalization started";
    }

    auto spin_once_runtime() -> void {
        if (!collecting || !collect_start_time.has_value()) {
            return;
        }
        if (std::chrono::steady_clock::now() - *collect_start_time < kCollectDuration) {
            return;
        }
        collecting = false;
        log::info(
            "finished collecting relocalization pointclouds: frames=%zu, points=%zu",
            collected_scan_frames, latest_scan->size());
        relocalize_once();
    }

    auto relocalize_once() -> void {
        // 服务入口已保证 initial_transform 有值
        const auto initial_guess = *initial_transform;
        initial_transform.reset();

        auto center = Eigen::Vector3f{initial_guess.translation()};
        auto local_map =
            extract_pointcloud(standard_map, Point{center.x(), center.y(), center.z()});
        registration.register_map(local_map);
        latest_origin_pointcloud = local_map;

        registration.register_scan(latest_scan);

        auto aligned = std::make_shared<PointCloud>();
        registration.full_match(aligned, initial_guess);

        auto mapped_scan = std::make_shared<PointCloud>();
        pcl::transformPointCloud(
            *latest_scan, *mapped_scan, registration.transformation().matrix());
        latest_mapped_pointcloud = mapped_scan;

        // 配准质量验收：fitness 超阈值视为失败，不污染 TF
        const auto score = registration.fitness_score();
        log::info(
            "relocalization fitness score: %.4f (accept threshold: %.4f)", score,
            config.accept_score_threshold);
        if (score > config.accept_score_threshold) {
            log::error(
                "relocalization rejected: fitness score %.4f exceeds accept threshold %.4f", score,
                config.accept_score_threshold);
            return;
        }

        // scan 在 odom 系、地图在 world 系，配准结果即 world->odom，广播一次即可
        const auto world_from_odom = registration.transformation();

        auto msg = geometry_msgs::msg::TransformStamped{};
        msg.header.stamp = now();
        msg.header.frame_id = config.world_frame;
        msg.child_frame_id = config.odom_frame;
        util::convert_orientation(
            Eigen::Quaternionf{world_from_odom.rotation()}, msg.transform.rotation);
        util::convert_translation(
            Eigen::Translation3f{world_from_odom.translation()}, msg.transform.translation);
        tf_broadcaster->sendTransform(msg);

        const auto& R = world_from_odom.rotation();
        const auto t = world_from_odom.translation();
        constexpr auto deg = 180.0f / std::numbers::pi_v<float>;
        const auto roll = std::atan2(R(2, 1), R(2, 2)) * deg;
        const auto pitch = std::atan2(-R(2, 0), std::hypot(R(2, 1), R(2, 2))) * deg;
        const auto yaw = std::atan2(R(1, 0), R(0, 0)) * deg;
        log::info(
            "relocalization succeeded: world_from_odom t=(%.3f, %.3f, %.3f) m, "
            "rpy=(%.2f, %.2f, %.2f) deg",
            t.x(), t.y(), t.z(), roll, pitch, yaw);
    }

    auto publish_debug_pointclouds() -> void {
        auto stamp = now();

        if (latest_origin_pointcloud && !latest_origin_pointcloud->empty()) {
            auto msg = sensor_msgs::msg::PointCloud2{};
            pcl::toROSMsg(*latest_origin_pointcloud, msg);
            msg.header.stamp = stamp;
            msg.header.frame_id = config.world_frame;
            origin_pointcloud_publisher->publish(msg);
        }

        if (latest_mapped_pointcloud && !latest_mapped_pointcloud->empty()) {
            auto msg = sensor_msgs::msg::PointCloud2{};
            pcl::toROSMsg(*latest_mapped_pointcloud, msg);
            msg.header.stamp = stamp;
            msg.header.frame_id = config.world_frame;
            mapped_pointcloud_publisher->publish(msg);
        }
    }

    auto extract_pointcloud(const std::shared_ptr<PointCloud>& pointcloud, Point center) const
        -> std::shared_ptr<PointCloud> {
        auto flann_kd_tree = pcl::KdTreeFLANN<Point>{};
        flann_kd_tree.setInputCloud(pointcloud);

        auto indices = pcl::Indices{};
        auto distances = std::vector<float>{};
        flann_kd_tree.radiusSearch(center, config.registration_radius, indices, distances);

        auto search_result = std::make_shared<PointCloud>();
        search_result->reserve(indices.size());
        for (const auto index : indices) {
            search_result->push_back(pointcloud->at(static_cast<std::size_t>(index)));
        }
        return search_result;
    }
};

} // namespace rmcs

auto main(int argc, char* argv[]) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rmcs::LocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
