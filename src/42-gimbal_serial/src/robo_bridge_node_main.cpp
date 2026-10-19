#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "gimbal_serial/bridge_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int ret = 0;
  try {
    auto node = std::make_shared<gimbal_serial::BridgeNode>();
    rclcpp::spin(node);
  } catch (const gimbal_serial::SerialOpenError & e) {
    // 串口打不开：直接报错退出。只有“设备文件还不存在”才用专门的退出码，
    // launch 的 respawn:=true 也只在这个退出码时重启；被占用（EBUSY、EWOULDBLOCK）永远不重试。
    RCLCPP_FATAL(rclcpp::get_logger("robo_bridge"), "%s", e.what());
    ret = e.device_absent ? gimbal_serial::kExitDeviceAbsent : gimbal_serial::kExitFatal;
  } catch (const std::exception & e) {
    // 仅在进程边界记录参数/程序异常并退出；不在回调中吞错或假装恢复
    RCLCPP_FATAL(rclcpp::get_logger("robo_bridge"), "%s", e.what());
    ret = gimbal_serial::kExitFatal;
  }
  rclcpp::shutdown();
  return ret;
}
