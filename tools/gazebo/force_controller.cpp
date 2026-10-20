// 仿真专用刚体速度伺服：施加力/力矩，让物理引擎生成一致的加速度。
// 不设置位姿或速度，不连接 ROS/真实硬件；默认命令为零。
#include <algorithm>
#include <chrono>
#include <mutex>
#include <stdexcept>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Inertial.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/twist.pb.h>

namespace rm_task1 {
class ForceController : public ignition::gazebo::System,
                        public ignition::gazebo::ISystemConfigure,
                        public ignition::gazebo::ISystemPreUpdate {
 public:
  void Configure(const ignition::gazebo::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &,
                 ignition::gazebo::EntityComponentManager &ecm,
                 ignition::gazebo::EventManager &) override {
    ignition::gazebo::Model model(entity);
    link_ = ignition::gazebo::Link(model.LinkByName(ecm, "body"));
    if (!link_.Valid(ecm)) throw std::runtime_error("仿真平台缺少 body link");
    link_.EnableVelocityChecks(ecm);
    link_.EnableAccelerationChecks(ecm);
    const auto inertial = ecm.Component<ignition::gazebo::components::Inertial>(link_.Entity());
    if (!inertial) throw std::runtime_error("仿真平台缺少惯性参数");
    mass_ = inertial->Data().MassMatrix().Mass();
    transport_.Subscribe("/sim/gazebo/cmd_vel", &ForceController::Command, this);
  }

  void PreUpdate(const ignition::gazebo::UpdateInfo &info,
                 ignition::gazebo::EntityComponentManager &ecm) override {
    if (info.paused) return;
    const auto pose = link_.WorldPose(ecm);
    const auto velocity = link_.WorldLinearVelocity(ecm);
    const auto omega = link_.WorldAngularVelocity(ecm);
    const auto inertia = link_.WorldInertiaMatrix(ecm);
    if (!pose || !velocity || !omega || !inertia) return;
    ignition::math::Vector3d linear, angular;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // 断开控制器后归零，避免依赖最后一条非零命令一直运动。
      if (std::chrono::steady_clock::now()-last_command_ < std::chrono::seconds(1)) {
        linear = target_linear_;
        angular = target_angular_;
      }
    }
    const auto target_v = pose->Rot().RotateVector(linear);
    const auto target_w = pose->Rot().RotateVector(angular);
    auto force = mass_*(target_v-*velocity)/0.06;
    auto torque = *inertia*((target_w-*omega)/0.06);
    if (force.Length() > 3.) force *= 3./force.Length();
    if (torque.Length() > .1) torque *= .1/torque.Length();
    link_.AddWorldWrench(ecm, force, torque);
  }

 private:
  void Command(const ignition::msgs::Twist &msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    target_linear_.Set(std::clamp(msg.linear().x(), -1., 1.),
                       std::clamp(msg.linear().y(), -1., 1.), 0.);
    target_angular_.Set(0., 0., std::clamp(msg.angular().z(), -1., 1.));
    last_command_ = std::chrono::steady_clock::now();
  }
  ignition::gazebo::Link link_;
  ignition::transport::Node transport_;
  std::mutex mutex_;
  ignition::math::Vector3d target_linear_{0., 0., 0.};
  ignition::math::Vector3d target_angular_{0., 0., 0.};
  std::chrono::steady_clock::time_point last_command_{};
  double mass_{1.};
};
}
IGNITION_ADD_PLUGIN(rm_task1::ForceController, ignition::gazebo::System,
                    rm_task1::ForceController::ISystemConfigure,
                    rm_task1::ForceController::ISystemPreUpdate)
IGNITION_ADD_PLUGIN_ALIAS(rm_task1::ForceController, "rm_task1::ForceController")
