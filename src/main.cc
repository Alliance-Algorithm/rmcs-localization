#include "registration/engine.hh"
#include "util/convert.hh"
#include "util/fsm.hh"
#include "util/logger.hh"
#include "util/parameter.hh"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

namespace rmcs {

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

    enum class Status {
        IDLE,
        COLLECTING,
        LOCALIZING,
        LOCALIZED,
        FAILED,
        END,
    };
    util::Fsm<Status> fsm{Status::IDLE};

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
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr relocalize_lost_service;

    std::shared_ptr<PointCloud> standard_map = std::make_shared<PointCloud>();
    std::shared_ptr<PointCloud> latest_scan = std::make_shared<PointCloud>();
    std::shared_ptr<PointCloud> latest_origin_pointcloud = std::make_shared<PointCloud>();
    std::shared_ptr<PointCloud> latest_mapped_pointcloud = std::make_shared<PointCloud>();
    std::size_t collected_scan_frames = 0;
    std::optional<std::chrono::steady_clock::time_point> collect_start_time;

    std::atomic<bool> collecting_enabled = false;
    std::atomic<bool> is_localizing = false;
    std::optional<Eigen::Isometry3f> current_world_from_odom;
    std::optional<Eigen::Isometry3f> initial_transform;
    std::optional<Eigen::Isometry3f> pending_world_from_odom;

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
                if (!collecting_enabled.load(std::memory_order_relaxed)) {
                    return;
                }

                auto cloud = std::make_shared<PointCloud>();
                pcl::fromROSMsg(*msg, *cloud);
                if (!latest_scan) {
                    latest_scan = std::make_shared<PointCloud>();
                }
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

        relocalize_lost_service = create_service<std_srvs::srv::Trigger>(
            "/rmcs_localization/relocalize/lost",
            [this](
                const std::shared_ptr<std_srvs::srv::Trigger::Request>& request,
                const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
                std::ignore = request;
                handle_relocalize_service("lost", *response);
            });

        install_fsm();

        log::info(
            "initialized with map=%s, pointcloud_topic=%s, frames=(%s -> %s)",
            config.map_path.c_str(), config.pointcloud_topic.c_str(), config.world_frame.c_str(),
            config.odom_frame.c_str());

        using namespace std::chrono_literals;
        runtime_timer = create_wall_timer(50ms, [this] { fsm.spin_once(); });
        debug_pointcloud_timer = create_wall_timer(1s, [this] { publish_debug_pointclouds(); });
    }

    auto install_fsm() -> void {
        fsm.use<Status::IDLE>(
            [this] { collecting_enabled.store(false, std::memory_order_relaxed); },
            [] { return Status::IDLE; });

        fsm.use<Status::COLLECTING>(
            [this] {
                collecting_enabled.store(true, std::memory_order_relaxed);
                latest_scan = std::make_shared<PointCloud>();
                collected_scan_frames = 0;
                collect_start_time = std::chrono::steady_clock::now();
                log::info(
                    "started collecting relocalization pointclouds for %.1f seconds",
                    kCollectDuration.count());
            },
            [this] {
                if (collect_start_time.has_value()
                    && (std::chrono::steady_clock::now() - *collect_start_time) >= kCollectDuration
                    && latest_scan && !latest_scan->empty()) {
                    log::info(
                        "finished collecting relocalization pointclouds: frames=%zu, points=%zu",
                        collected_scan_frames, latest_scan->size());
                    return Status::LOCALIZING;
                }
                return Status::COLLECTING;
            });

        fsm.use<Status::LOCALIZING>(
            [this] {
                collecting_enabled.store(false, std::memory_order_relaxed);
                is_localizing.store(true, std::memory_order_relaxed);
                log::info(
                    "starting relocalization with scan_points=%zu and map_radius=%.3f",
                    latest_scan ? latest_scan->size() : 0, config.registration_radius);
                relocalize_once();
                is_localizing.store(false, std::memory_order_relaxed);
            },
            [this] {
                if (pending_world_from_odom.has_value()) {
                    current_world_from_odom = pending_world_from_odom;
                    const auto translation = current_world_from_odom->translation();
                    log::info(
                        "relocalization succeeded: world_from_odom translation=(%.3f, %.3f, %.3f)",
                        translation.x(), translation.y(), translation.z());
                    pending_world_from_odom.reset();
                    return Status::LOCALIZED;
                }
                return Status::FAILED;
            });

        fsm.use<Status::LOCALIZED>([this] { publish_outputs(); }, [] { return Status::LOCALIZED; });

        fsm.use<Status::FAILED>(
            [] { log::warn("relocalization failed !!!"); }, [] { return Status::FAILED; });

        if (!fsm.fully_registered()) {
            throw std::runtime_error{"runtime fsm is not fully registered"};
        }
    }

    auto
        handle_relocalize_service(std::string_view mode, std_srvs::srv::Trigger::Response& response)
            -> void {
        if (mode == "lost") {
            log::warn("received lost relocalization request, but lost mode is not implemented");
            response.success = false;
            response.message = "lost relocalization placeholder";
            return;
        }

        if (is_localizing.load(std::memory_order_relaxed)) {
            log::warn(
                "ignored %.*s relocalization request because localization is already running",
                static_cast<int>(mode.size()), mode.data());
            response.success = false;
            response.message = "relocalization already in progress";
            return;
        }

        log::info(
            "received %.*s relocalization request", static_cast<int>(mode.size()), mode.data());
        initial_transform = config.initial_world_to_odom(mode);
        latest_scan = std::make_shared<PointCloud>();
        collect_start_time.reset();
        pending_world_from_odom.reset();
        fsm.start_on(Status::COLLECTING);

        response.success = true;
        response.message = std::string{mode} + " relocalization started";
    }

    auto relocalize_once() -> void {
        try {
            auto initial_world_from_odom =
                current_world_from_odom.has_value()
                    ? *current_world_from_odom
                    : initial_transform.value_or(config.initial_world_to_odom("red"));
            auto initial_guess = initial_world_from_odom;

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

            pending_world_from_odom = Eigen::Isometry3f{registration.transformation()};
            initial_transform.reset();
        } catch (const std::exception& e) {
            log::error("relocalization failed: %s", e.what());
            pending_world_from_odom.reset();
        }
    }

    auto publish_outputs() -> void {
        if (!current_world_from_odom.has_value()) {
            return;
        }

        auto msg = geometry_msgs::msg::TransformStamped{};
        msg.header.stamp = now();
        msg.header.frame_id = config.world_frame;
        msg.child_frame_id = config.odom_frame;
        util::convert_orientation(
            Eigen::Quaternionf{current_world_from_odom->rotation()}, msg.transform.rotation);
        util::convert_translation(
            Eigen::Translation3f{current_world_from_odom->translation()},
            msg.transform.translation);
        tf_broadcaster->sendTransform(msg);
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
