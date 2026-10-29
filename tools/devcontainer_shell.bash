# 开发容器新终端加载与标准验证相同的依赖，不修改用户 .bashrc。
if [[ -f ~/.bashrc ]]; then
  source ~/.bashrc
fi
source /opt/ros/humble/setup.bash
source /opt/livox/install/setup.bash
