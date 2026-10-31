// 项目接口适配：算法保持上游原样，位姿参考点为 IMU body。
#include "small_point_lio/small_point_lio.h"
#include "sensor_input_audit.hpp"
#include "livox_pointcloud2.hpp"
#include "world_alignment.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <deque>

class Backend : public rclcpp::Node {
 public:
  Backend() : Node("team43_small_point_lio") {
    const auto lidar = declare_parameter<std::string>("lidar_topic", "/red_standard_robot1/livox/lidar");
    const auto imu = declare_parameter<std::string>("imu_topic", "/red_standard_robot1/livox/imu");
    world_ = declare_parameter<std::string>("world_frame", "lio_world");
    body_ = declare_parameter<std::string>("body_frame", "body");
    output_rotation_ = rm_task1::world_alignment(declare_parameter<std::vector<double>>(
      "output_world_up", std::vector<double>{0., 0., 1.}));
    // 高度仅供配套建图入口计算近似地面，声明以便现场采集实际运行依据。
    declare_parameter<bool>("approximate_floor_from_height", false);
    declare_parameter<double>("lidar_height_m", 0.231);
    const auto depth = declare_parameter<int>("sensor_qos_depth", 200);
    if (depth < 1 || depth > 8192) throw std::invalid_argument("接收队列须为1至8192");
    imu_startup_capacity_ = static_cast<std::size_t>(depth);
    audit_.configure(declare_parameter<std::string>("input_audit_path", ""));
    const auto translation = declare_parameter<std::vector<double>>("extrinsic_T");
    const auto rotation = declare_parameter<std::vector<double>>("extrinsic_R");
    if (translation.size() != 3 || rotation.size() != 9)
      throw std::invalid_argument("雷达到 IMU 外参须为3维平移和9维旋转矩阵");
    if (declare_parameter<bool>("extrinsic_est_en", false))
      throw std::invalid_argument("项目适配要求已标定的固定外参，不支持在线外参估计");
    for (int i = 0; i < 3; ++i) {
      lidar_translation_[i] = translation[i];
      for (int j = 0; j < 3; ++j) lidar_rotation_(i, j) = rotation[3*i+j];
    }
    if (!lidar_translation_.allFinite() || !lidar_rotation_.allFinite() ||
        !(lidar_rotation_.transpose()*lidar_rotation_).isApprox(Eigen::Matrix3f::Identity(), 1e-4f) ||
        std::abs(lidar_rotation_.determinant()-1.f) > 1e-4f)
      throw std::invalid_argument("雷达外参须有限且旋转矩阵须正交、行列式为1");
    // 上游初始化地图直接加入输入点；先将输入统一到 IMU frame，确保初始地图
    // 与后续状态处于相同坐标约定。核心使用单位外参，避免重复变换。
    std::vector<rclcpp::Parameter> core_parameters;
    for (const auto& [name, value] : get_node_parameters_interface()->get_parameter_overrides()) {
      if (name != "extrinsic_T" && name != "extrinsic_R") core_parameters.emplace_back(name, value);
    }
    core_parameters.emplace_back("extrinsic_T", std::vector<double>{0., 0., 0.});
    core_parameters.emplace_back("extrinsic_R", std::vector<double>{1., 0., 0., 0., 1., 0., 0., 0., 1.});
    rclcpp::NodeOptions options;
    options.use_global_arguments(false).parameter_overrides(core_parameters)
      .start_parameter_services(false).start_parameter_event_publisher(false).enable_rosout(false);
    core_parameters_ = std::make_shared<rclcpp::Node>("small_lio_core_parameters", get_namespace(), options);
    core_ = std::make_unique<small_point_lio::SmallPointLio>(*core_parameters_);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("aft_mapped_to_init", 100);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered", 10);
    core_->set_odometry_callback([this](const common::Odometry& state) {
      stamp_ = rclcpp::Time(static_cast<int64_t>(std::llround(state.timestamp * 1e9)));
      nav_msgs::msg::Odometry msg;
      msg.header.stamp = stamp_;
      msg.header.frame_id = world_;
      msg.child_frame_id = body_;
      auto& p = msg.pose.pose;
      const Eigen::Vector3d position = output_rotation_ * state.position;
      const Eigen::Quaterniond orientation = (output_rotation_ * state.orientation).normalized();
      p.position.x = position.x(); p.position.y = position.y(); p.position.z = position.z();
      p.orientation.x = orientation.x(); p.orientation.y = orientation.y();
      p.orientation.z = orientation.z(); p.orientation.w = orientation.w();
      // 与现有下游一致，由位姿差分推导底盘速度。
      odom_pub_->publish(msg);
    });
    core_->set_pointcloud_callback([this](const std::vector<Eigen::Vector3f>& points) {
      sensor_msgs::msg::PointCloud2 msg;
      msg.header.stamp = stamp_;
      msg.header.frame_id = world_;
      sensor_msgs::PointCloud2Modifier modifier(msg);
      modifier.setPointCloud2FieldsByString(1, "xyz");
      modifier.resize(points.size());
      sensor_msgs::PointCloud2Iterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z");
      for (const auto& p : points) {
        const Eigen::Vector3f aligned = output_rotation_.cast<float>() * p;
        *x = aligned.x(); *y = aligned.y(); *z = aligned.z(); ++x; ++y; ++z;
      }
      cloud_pub_->publish(msg);
    });
    auto qos = rclcpp::SensorDataQoS().keep_last(depth);
    // 固定接收Livox PointCloud2；ROS负责晚启动发布者发现和话题remap。
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(lidar, qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        std::vector<common::Point> points;
        try {
          // 由有界data决定预留量，避免坏width/height导致巨量分配。
          if (msg->point_step) points.reserve(msg->data.size() / msg->point_step);
          rm_task1::visit_livox_pointcloud2(*msg, [this, &points](double time, float x, float y, float z) {
            points.push_back({time, lidar_rotation_*Eigen::Vector3f(x, y, z)+lidar_translation_});
          });
        } catch (const std::invalid_argument& error) {
          RCLCPP_ERROR_THROTTLE(get_logger(), diagnostic_clock_, 2000,
                               "拒收 Livox PointCloud2：%s", error.what());
          return;
        }
        if (points.empty()) return;
        lidar_received_ = true;
        audit_.record(false, rclcpp::Time(msg->header.stamp).seconds());
        core_->on_point_cloud_callback(points);
        while (!startup_imu_.empty()) {
          consume_imu(startup_imu_.front());
          startup_imu_.pop_front();
        }
        core_->handle_once();
      });
    RCLCPP_INFO(get_logger(), "点云输入：%s，固定格式：PointCloud2", cloud_sub_->get_topic_name());
    // 实机默认仍使用传感器 QoS；离线仿真可与可靠桥接发布端协商重传，
    // 避免共享 CPU 调度抖动造成不可重传的 IMU 丢样。
    auto imu_qos = qos;
    if (declare_parameter<bool>("imu_qos_reliable", false)) imu_qos.reliable();
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(imu, imu_qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        const double time = rclcpp::Time(msg->header.stamp).seconds();
        common::ImuMsg sample;
        sample.timestamp = time;
        sample.angular_velocity = {msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
        sample.linear_acceleration = {msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z};
        if (!lidar_received_) {
          // 保留早于首帧点云到达的IMU；驱动长期无点云时缓存仍有界。
          if (startup_imu_.size() >= imu_startup_capacity_) startup_imu_.pop_front();
          startup_imu_.push_back(sample);
          return;
        }
        consume_imu(sample);
      });
  }
 private:
  void consume_imu(const common::ImuMsg& sample) {
    audit_.record(true, sample.timestamp);
    core_->on_imu_callback(sample);
    core_->handle_once();
  }
  std::string world_, body_;
  bool lidar_received_ = false;
  std::size_t imu_startup_capacity_ = 200;
  std::deque<common::ImuMsg> startup_imu_;
  rclcpp::Clock diagnostic_clock_{RCL_STEADY_TIME};
  rclcpp::Time stamp_{0};
  rm_task1::InputAudit audit_;
  Eigen::Quaterniond output_rotation_;
  Eigen::Matrix3f lidar_rotation_;
  Eigen::Vector3f lidar_translation_;
  rclcpp::Node::SharedPtr core_parameters_;
  std::unique_ptr<small_point_lio::SmallPointLio> core_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  { auto node = std::make_shared<Backend>(); rclcpp::spin(node); }
  rclcpp::shutdown();
}
