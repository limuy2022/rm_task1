# -*- coding: utf-8 -*-
"""``gimbal_common`` 的**本地测试桩**（test-only）。

仅当环境中**不存在**真实的 ``gimbal_common``（Ubuntu + ROS2 工作区已 colcon build）
时，由 ``conftest.py`` 把它挂到 ``sys.path`` 上，保证 ``test_ca_kalman.py`` /
``test_control_law.py`` 在 Windows 无 ROS 环境下也能直接 ``pytest`` 跑通。

⚠ 本桩**不是**产品代码：真实实现由工程师 A 在 ``src/gimbal_common`` 提供。
本桩只实现 T03 依赖的契约子集：

* ``frames.PITCH_SIGN / to_protocol_pitch / from_protocol_pitch``
* ``frames.quat_to_mat / decompose_yaw_pitch / R_opt_from_link / rotz2 / compose_T``
* ``math_utils.wrap_pi / clamp / lowpass / accel_limit``
* ``time_utils.now``
* ``log_utils.get_logger / throttled``
"""
