// 项目接口适配：算法保持上游原样，位姿参考点为 IMU body。
#include "small_point_lio/small_point_lio.h"
#include "sensor_input_audit.hpp"
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class Backend : public rclcpp::Node {
 public:
  Backend() : Node("team43_small_point_lio") {
    const auto lidar = declare_parameter<std::string>("lidar_topic", "/livox/lidar");
    const auto imu = declare_parameter<std::string>("imu_topic", "/livox/imu");
    world_ = declare_parameter<std::string>("world_frame", "camera_init");
    body_ = declare_parameter<std::string>("body_frame", "body");
    const auto depth = declare_parameter<int>("sensor_qos_depth", 200);
    if (depth < 1 || depth > 8192) throw std::invalid_argument("接收队列须为1至8192");
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
      p.position.x = state.position.x(); p.position.y = state.position.y(); p.position.z = state.position.z();
      p.orientation.x = state.orientation.x(); p.orientation.y = state.orientation.y();
      p.orientation.z = state.orientation.z(); p.orientation.w = state.orientation.w();
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
      for (const auto& p : points) { *x = p.x(); *y = p.y(); *z = p.z(); ++x; ++y; ++z; }
      cloud_pub_->publish(msg);
    });
    auto qos = rclcpp::SensorDataQoS().keep_last(depth);
    lidar_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(lidar, qos,
      [this](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg) {
        audit_.record(false, rclcpp::Time(msg->header.stamp).seconds());
        std::vector<common::Point> points;
        points.reserve(msg->points.size());
        for (const auto& p : msg->points) {
          // Livox 正常回波的返回序号为0或1；保留真实逐点时间。
          const auto tag = p.tag & 0x30;
          if ((tag != 0 && tag != 0x10) || !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
          points.push_back({static_cast<double>(msg->timebase) * 1e-9 + p.offset_time * 1e-9,
                            lidar_rotation_*Eigen::Vector3f(p.x, p.y, p.z)+lidar_translation_});
        }
        core_->on_point_cloud_callback(points);
        core_->handle_once();
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(imu, qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        const double time = rclcpp::Time(msg->header.stamp).seconds();
        audit_.record(true, time);
        common::ImuMsg sample;
        sample.timestamp = time;
        sample.angular_velocity = {msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
        sample.linear_acceleration = {msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z};
        core_->on_imu_callback(sample);
        core_->handle_once();
      });
  }
 private:
  std::string world_, body_;
  rclcpp::Time stamp_{0};
  rm_task1::InputAudit audit_;
  Eigen::Matrix3f lidar_rotation_;
  Eigen::Vector3f lidar_translation_;
  rclcpp::Node::SharedPtr core_parameters_;
  std::unique_ptr<small_point_lio::SmallPointLio> core_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  { auto node = std::make_shared<Backend>(); rclcpp::spin(node); }
  rclcpp::shutdown();
}
