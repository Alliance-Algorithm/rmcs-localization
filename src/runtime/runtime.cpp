#include "runtime/runtime.hpp"

#include "registration/engine.hpp"
#include "util/convert.hpp"
#include "util/logger.hpp"
#include "util/parameter.hpp"
#include "util/segmentation.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <array>
#include <cmath>
#include <deque>
#include <optional>
#include <ranges>
#include <thread>

namespace rmcs {

struct Runtime::Impl {
    using Point = pcl::PointXYZ;
    using PointCloud = pcl::PointCloud<Point>;
    using log = util::Log<[] { return "rmcs_localization"; }>;

    template <typename state_type>
    class Fsm {
        static_assert(std::is_enum_v<state_type>);
        static_assert(requires { state_type::END; });

        struct IState {
            virtual auto on_begin() -> void = 0;
            virtual auto on_event() -> state_type = 0;
            virtual ~IState() = default;
        };

        using StateUnique = std::unique_ptr<IState>;

        std::array<StateUnique, std::to_underlying(state_type::END)> states_map;
        state_type current_state;
        IState* current_event = nullptr;

    public:
        explicit Fsm(state_type start_state) noexcept
            : current_state(start_state) { }

        template <typename on_begin_type, typename on_event_type>
        auto use(state_type state, on_begin_type&& on_begin, on_event_type&& on_event) {
            struct State final : IState {
                std::decay_t<on_begin_type> impl_on_begin;
                std::decay_t<on_event_type> impl_on_event;

                State(on_begin_type&& on_begin, on_event_type&& on_event)
                    : impl_on_begin(std::forward<on_begin_type>(on_begin))
                    , impl_on_event(std::forward<on_event_type>(on_event)) { }

                auto on_begin() -> void override { impl_on_begin(); }
                auto on_event() -> state_type override { return impl_on_event(); }
            };

            states_map.at(std::to_underlying(state)) = std::make_unique<State>(
                std::forward<on_begin_type>(on_begin), std::forward<on_event_type>(on_event));
        }

        template <state_type state>
        auto use(auto&& on_begin, auto&& on_event) {
            use(
                state, std::forward<decltype(on_begin)>(on_begin), std::forward<decltype(on_event)>(on_event));
        }

        [[nodiscard]] auto fully_registered() const noexcept -> bool {
            return std::ranges::all_of(states_map, [](const auto& state) {
                return static_cast<bool>(state);
            });
        }

        auto start_on(state_type state) -> void {
            current_state = state;
            current_event = nullptr;
        }

        auto spin_once() -> bool {
            if (current_event == nullptr) {
                current_event = states_map.at(std::to_underlying(current_state)).get();
                current_event->on_begin();
            }

            const auto next = current_event->on_event();
            if (next != current_state) {
                current_state = next;
                current_event = nullptr;
            }
            return next == state_type::END;
        }
    };

    enum class Status {
        IDLE,
        COLLECTING,
        LOCALIZING,
        LOCALIZED,
        FAILED,
        END,
    };

    struct PoseSample {
        rclcpp::Time stamp;
        Eigen::Isometry3f odom_from_base = Eigen::Isometry3f::Identity();
        bool trusted = true;
    };

    void initialize(rclcpp::Node& node) {
        this->node = &node;
        const auto p = util::quick_paramtetr_reader { node };

        registration.initialize(node);

        world_frame = p("frames.world", std::string {});
        odom_frame = p("frames.odom", std::string {});
        base_frame = p("frames.base", std::string {});
        pointcloud_topic = p("subscription.pointcloud", std::string {});
        pose_topic = p("publish.robot_pose", std::string {});
        relocalize_service_name = p("service.relocalize", std::string {});
        publish_tf = p("publish.tf", bool {});
        receive_pointcloud_size = p("registration.receive_size", std::size_t {});
        registration_radius = p("registration.initial_map_radius", double {});
        history_enabled = p("history.enable", bool {});
        history_duration = rclcpp::Duration::from_seconds(p("history.duration_sec", double {}));
        history_sample_period = std::chrono::duration<double>(p("history.sample_period_sec", double {}));

        {
            const auto translation = Eigen::Translation3f {
                p("default_pose.translation.x", float {}),
                p("default_pose.translation.y", float {}),
                p("default_pose.translation.z", float {}),
            };
            const auto orientation = Eigen::Quaternionf {
                p("default_pose.orientation.w", float {}),
                p("default_pose.orientation.x", float {}),
                p("default_pose.orientation.y", float {}),
                p("default_pose.orientation.z", float {}),
            }.normalized();
            default_pose = translation * orientation;
        }
        current_world_from_odom = std::nullopt;

        if (pcl::io::loadPCDFile(p("map_path", std::string {}), *standard_map) == -1) {
            throw util::runtime_error("failed to load map pcd file");
        }

        segmentation.set_distance_threshold(p("segmentation.distance_threshold", double {}));
        segmentation.set_ground_max_height(p("segmentation.ground_max_height", double {}));
        segmentation.set_limit_distance(p("segmentation.limit_distance", double {}));
        segmentation.set_limit_max_height(p("segmentation.limit_max_height", double {}));

        tf_buffer = std::make_unique<tf2_ros::Buffer>(node.get_clock());
        tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);

        pose_publisher = node.create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, 10);
        pointcloud_subscription = node.create_subscription<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
                if (!collecting_enabled.load(std::memory_order_relaxed)) {
                    return;
                }

                auto cloud = std::make_shared<PointCloud>();
                pcl::fromROSMsg(*msg, *cloud);
                latest_scan = cloud;
            });

        relocalize_service = node.create_service<std_srvs::srv::Trigger>(
            relocalize_service_name,
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>& request,
                const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
                std::ignore = request;
                if (is_localizing.load(std::memory_order_relaxed)) {
                    response->success = false;
                    response->message = "relocalization already in progress";
                    return;
                }

                latest_scan = std::make_shared<PointCloud>();
                frozen_odom_from_base.reset();
                pending_world_from_odom.reset();
                fsm.start_on(Status::COLLECTING);
                response->success = true;
                response->message = "relocalization started";
            });

        install_fsm();
        if (!fsm.fully_registered()) {
            throw std::runtime_error { "runtime fsm is not fully registered" };
        }

        using namespace std::chrono_literals;
        runtime_timer = node.create_wall_timer(50ms, [this] { update(); });
    }

private:
    rclcpp::Node* node {};
    Fsm<Status> fsm { Status::IDLE };

    Registration registration {};
    Segmentation segmentation {};

    std::string world_frame;
    std::string odom_frame;
    std::string base_frame;
    std::string pointcloud_topic;
    std::string pose_topic;
    std::string relocalize_service_name;

    bool publish_tf = true;
    bool history_enabled = true;
    std::size_t receive_pointcloud_size = 2500;
    double registration_radius = 25.0;
    rclcpp::Duration history_duration = rclcpp::Duration::from_seconds(10.0);
    std::chrono::duration<double> history_sample_period = std::chrono::duration<double>(0.05);

    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr relocalize_service;
    rclcpp::TimerBase::SharedPtr runtime_timer;

    std::shared_ptr<PointCloud> standard_map = std::make_shared<PointCloud>();
    std::shared_ptr<PointCloud> latest_scan = std::make_shared<PointCloud>();

    std::atomic<bool> collecting_enabled = false;
    std::atomic<bool> is_localizing = false;
    std::optional<Eigen::Isometry3f> current_world_from_odom;
    std::optional<Eigen::Isometry3f> frozen_odom_from_base;
    std::optional<Eigen::Isometry3f> pending_world_from_odom;
    Eigen::Isometry3f default_pose = Eigen::Isometry3f::Identity();

    std::deque<PoseSample> pose_history;
    rclcpp::Time last_history_sample_time { 0, 0, RCL_ROS_TIME };

    void install_fsm() {
        fsm.use<Status::IDLE>([this] { collecting_enabled.store(false, std::memory_order_relaxed); },
            [this] {
                record_pose_sample_if_needed();
                publish_outputs();
                return Status::IDLE;
            });

        fsm.use<Status::COLLECTING>(
            [this] {
                collecting_enabled.store(true, std::memory_order_relaxed);
                frozen_odom_from_base = latest_trusted_pose_transform();
                latest_scan = std::make_shared<PointCloud>();
            },
            [this] {
                record_pose_sample_if_needed();
                if (latest_scan && latest_scan->size() > receive_pointcloud_size) {
                    return Status::LOCALIZING;
                }
                return Status::COLLECTING;
            });

        fsm.use<Status::LOCALIZING>(
            [this] {
                collecting_enabled.store(false, std::memory_order_relaxed);
                is_localizing.store(true, std::memory_order_relaxed);
                relocalize_once();
                is_localizing.store(false, std::memory_order_relaxed);
            },
            [this] {
                if (pending_world_from_odom.has_value()) {
                    current_world_from_odom = pending_world_from_odom;
                    pending_world_from_odom.reset();
                    return Status::LOCALIZED;
                }
                return Status::FAILED;
            });

        fsm.use<Status::LOCALIZED>([this] { publish_outputs(); }, [this] {
            record_pose_sample_if_needed();
            publish_outputs();
            return Status::LOCALIZED;
        });

        fsm.use<Status::FAILED>([this] { publish_outputs(); }, [this] {
            record_pose_sample_if_needed();
            publish_outputs();
            return Status::FAILED;
        });
    }

    void update() { fsm.spin_once(); }

    void relocalize_once() {
        try {
            auto odom_from_base = frozen_odom_from_base.value_or(default_pose);
            auto initial_guess = current_world_from_odom.has_value()
                ? (*current_world_from_odom) * odom_from_base
                : odom_from_base;

            const auto center = initial_guess.translation();
            auto local_map = extract_pointcloud(standard_map, Point { center.x(), center.y(), center.z() });
            registration.register_map(local_map);

            auto scan = std::make_shared<PointCloud>(*latest_scan);
            segmentation.set_input_source(scan);
            scan = segmentation.execute();
            registration.register_scan(scan);

            auto aligned = std::make_shared<PointCloud>();
            registration.full_match(aligned, initial_guess);

            const auto world_from_base = registration.transformation();
            const auto world_from_odom = world_from_base * odom_from_base.inverse();
            pending_world_from_odom = world_from_odom;
        } catch (const std::exception& e) {
            log::error("relocalization failed: %s", e.what());
            pending_world_from_odom.reset();
        }
    }

    void publish_outputs() {
        if (!current_world_from_odom.has_value()) {
            return;
        }

        const auto odom_from_base = lookup_odom_from_base();
        if (!odom_from_base.has_value()) {
            return;
        }

        const auto world_from_base = (*current_world_from_odom) * (*odom_from_base);

        if (publish_tf) {
            auto msg = geometry_msgs::msg::TransformStamped {};
            msg.header.stamp = node->now();
            msg.header.frame_id = world_frame;
            msg.child_frame_id = odom_frame;
            util::convert_orientation(Eigen::Quaternionf { current_world_from_odom->rotation() }, msg.transform.rotation);
            util::convert_translation(Eigen::Translation3f { current_world_from_odom->translation() }, msg.transform.translation);
            tf_broadcaster->sendTransform(msg);
        }

        auto pose = geometry_msgs::msg::PoseStamped {};
        pose.header.stamp = node->now();
        pose.header.frame_id = world_frame;
        util::convert_orientation(Eigen::Quaternionf { world_from_base.rotation() }, pose.pose.orientation);
        util::convert_translation(Eigen::Translation3f { world_from_base.translation() }, pose.pose.position);
        pose_publisher->publish(pose);
    }

    std::optional<Eigen::Isometry3f> lookup_odom_from_base() const {
        try {
            const auto transform = tf_buffer->lookupTransform(odom_frame, base_frame, tf2::TimePointZero);
            return Eigen::Isometry3f { tf2::transformToEigen(transform.transform).cast<float>() };
        } catch (const tf2::TransformException&) {
            return std::nullopt;
        }
    }

    void record_pose_sample_if_needed() {
        if (!history_enabled) {
            return;
        }

        const auto now = node->now();
        if (last_history_sample_time.nanoseconds() != 0) {
            const auto delta = (now - last_history_sample_time).seconds();
            if (delta < history_sample_period.count()) {
                return;
            }
        }

        const auto odom_from_base = lookup_odom_from_base();
        if (!odom_from_base.has_value()) {
            return;
        }

        pose_history.push_back(PoseSample { now, *odom_from_base, true });
        last_history_sample_time = now;

        while (!pose_history.empty() && (now - pose_history.front().stamp) > history_duration) {
            pose_history.pop_front();
        }
    }

    std::optional<Eigen::Isometry3f> latest_trusted_pose_transform() const {
        for (auto it = pose_history.rbegin(); it != pose_history.rend(); ++it) {
            if (it->trusted) {
                return it->odom_from_base;
            }
        }
        return lookup_odom_from_base();
    }

    std::shared_ptr<PointCloud> extract_pointcloud(const std::shared_ptr<PointCloud>& pointcloud, Point center) const {
        auto flann_kd_tree = pcl::KdTreeFLANN<Point> {};
        flann_kd_tree.setInputCloud(pointcloud);

        auto indices = pcl::Indices {};
        auto distances = std::vector<float> {};
        flann_kd_tree.radiusSearch(center, registration_radius, indices, distances);

        auto search_result = std::make_shared<PointCloud>();
        search_result->reserve(indices.size());
        for (const auto index : indices) {
            search_result->push_back(pointcloud->at(static_cast<std::size_t>(index)));
        }
        return search_result;
    }
};

Runtime::Runtime()
    : pimpl(std::make_unique<Impl>()) { }

Runtime::~Runtime() = default;

void Runtime::initialize(rclcpp::Node& node) { pimpl->initialize(node); }

} // namespace rmcs
