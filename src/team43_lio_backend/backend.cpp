// 项目接口适配：算法保持上游原样，位姿参考点为 IMU body。
#include "small_point_lio/small_point_lio.h"
#include "sensor_input_audit.hpp"
#include "livox_pointcloud2.hpp"
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <chrono>
#include <set>

class Backend : public rclcpp::Node {
 public:
  Backend() : Node("team43_small_point_lio") {
    const auto lidar = declare_parameter<std::string>("lidar_topic", "/livox/lidar");
    const auto imu = declare_parameter<std::string>("imu_topic", "/livox/imu");
    world_ = declare_parameter<std::string>("world_frame", "camera_init");
    body_ = declare_parameter<std::string>("body_frame", "body");
    const auto depth = declare_parameter<int>("sensor_qos_depth", 200);
    if (depth < 1 || depth > 8192) throw std::invalid_argument("接收队列须为1至8192");
    sensor_depth_ = depth;
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
    lidar_topic_ = get_node_topics_interface()->resolve_topic_name(lidar);
    const auto format = declare_parameter<std::string>("lidar_format", "auto");
    if (format == "auto") {
      automatic_ = true;
      RCLCPP_INFO(get_logger(), "自动识别：等待点云发布者 %s", lidar_topic_.c_str());
      // 墙钟定时器也适用于尚未开始发布 /clock 的 Gazebo；驱动可晚启动。
      detect_lidar_format(qos);
      detection_timer_ = create_wall_timer(std::chrono::milliseconds(200),
          [this, qos]() { detect_lidar_format(qos); });
    } else if (format == "custom" || format == "pointcloud2") {
      input_ready_ = true;
      subscribe_lidar(format, qos);
    } else {
      throw std::invalid_argument("lidar_format 只支持 auto、custom 或 pointcloud2");
    }
    // 实机默认仍使用传感器 QoS；离线仿真可与可靠桥接发布端协商重传，
    // 避免共享 CPU 调度抖动造成不可重传的 IMU 丢样。
    auto imu_qos = qos;
    if (declare_parameter<bool>("imu_qos_reliable", false)) imu_qos.reliable();
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(imu, imu_qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        // 等待发布者/冲突停用时不向未就绪的核心无限累积 IMU。
        if (!input_ready_ || input_fault_) return;
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
  void subscribe_lidar(const std::string& format, const rclcpp::QoS& qos) {
    if (format == "pointcloud2") {
      cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(lidar_topic_, qos,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          if (!input_ready_ || input_fault_) return;
          detect_if_auto();
          if (!input_ready_ || input_fault_) return;
          std::vector<common::Point> points;
          try {
            // 由有界 data 决定预留量，防止非法 width/height 导致巨量分配。
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
          audit_.record(false, rclcpp::Time(msg->header.stamp).seconds());
          core_->on_point_cloud_callback(points);
          core_->handle_once();
        });
    } else if (format == "custom") {
      lidar_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(lidar_topic_, qos,
      [this](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg) {
        if (!input_ready_ || input_fault_) return;
        detect_if_auto();
        if (!input_ready_ || input_fault_) return;
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
    }
    selected_format_ = format;
    RCLCPP_INFO(get_logger(), "点云输入：%s，格式：%s%s", lidar_topic_.c_str(), format.c_str(),
                automatic_ ? "（自动识别并锁定）" : "（手动指定）");
  }

  void detect_if_auto() {
    if (automatic_) detect_lidar_format(rclcpp::SensorDataQoS().keep_last(sensor_depth_));
  }

  void detect_lidar_format(const rclcpp::QoS& qos) {
    if (input_fault_) return;
    std::set<std::string> types;
    for (const auto& endpoint : get_publishers_info_by_topic(lidar_topic_))
      types.insert(endpoint.topic_type());
    input_ready_ = false;
    if (types.empty()) {
      candidate_format_.clear();
      RCLCPP_WARN_THROTTLE(get_logger(), diagnostic_clock_, 5000,
                          "等待点云发布者：%s；自动识别尚未就绪", lidar_topic_.c_str());
      return;
    }
    std::string detected;
    if (types.size() == 1) {
      if (*types.begin() == "sensor_msgs/msg/PointCloud2") detected = "pointcloud2";
      if (*types.begin() == "livox_ros_driver2/msg/CustomMsg") detected = "custom";
    }
    if (detected.empty() || (!selected_format_.empty() && detected != selected_format_)) {
      candidate_format_.clear();
      std::string observed;
      for (const auto& type : types) observed += type + " ";
      // 选择前等待冲突消除；选定后不能把不同消息流混入已有地图/起点。
      if (!selected_format_.empty()) {
        input_fault_ = true;
        RCLCPP_ERROR(get_logger(), "点云类型冲突或改变，停止接收；需重启定位并重建坐标关联：%s [%s]",
                     lidar_topic_.c_str(), observed.c_str());
      } else {
        RCLCPP_ERROR_THROTTLE(get_logger(), diagnostic_clock_, 2000,
                             "点云类型不支持或存在冲突，等待修正：%s [%s]",
                             lidar_topic_.c_str(), observed.c_str());
      }
      return;
    }
    if (selected_format_.empty()) {
      // 连续两次发现结果一致再选择，避免 DDS 启动发现尚未完整时抢选。
      if (candidate_format_ != detected) {
        candidate_format_ = detected;
        return;
      }
      subscribe_lidar(detected, qos);
    }
    input_ready_ = true;
  }

  std::string world_, body_, lidar_topic_, selected_format_, candidate_format_;
  int sensor_depth_ = 200;
  bool automatic_ = false, input_ready_ = false, input_fault_ = false;
  rclcpp::Clock diagnostic_clock_{RCL_STEADY_TIME};
  rclcpp::Time stamp_{0};
  rm_task1::InputAudit audit_;
  Eigen::Matrix3f lidar_rotation_;
  Eigen::Vector3f lidar_translation_;
  rclcpp::Node::SharedPtr core_parameters_;
  std::unique_ptr<small_point_lio::SmallPointLio> core_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr detection_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  { auto node = std::make_shared<Backend>(); rclcpp::spin(node); }
  rclcpp::shutdown();
}
