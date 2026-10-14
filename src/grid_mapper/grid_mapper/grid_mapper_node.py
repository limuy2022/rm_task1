"""grid_mapper_node：启动即建图落盘，并发布 `/map/grid` 与 `/map/ray_grid`（均锁存）。

对应 DESIGN §3.10 / §6.11 / §6.14。

话题（§4.3）：

| 话题 | 类型 | QoS |
|---|---|---|
| `/map/grid` | `nav_msgs/OccupancyGrid`（干净三态） | TRANSIENT_LOCAL, depth=1 |
| `/map/ray_grid` | `nav_msgs/OccupancyGrid`（含膨胀，供避障） | TRANSIENT_LOCAL, depth=1 |

两种地图来源（`grid.map_source`）：

* `config`：读 `field.yaml` 栅格化（12×12 m @ 0.05 m/cell，给定点居中）——赛前离线建图/仿真。
* `topic`：订阅自己的 SLAM `/map`（`pb2025_sentry_nav` + `small_gicp_relocalization`），
  **原样采用其 resolution/origin**，只额外按 `grid.inflation_radius_m` 生成 `ray_grid`。

⚠ 写 `map.yaml` 的 origin 一律取**栅格自身**的 `origin_xy`（见 `resolve_map_origin`），
不套 `grid.origin_xyz`——否则 topic 模式下整张图会按两者的差值平移。

时钟约定（主理人拍板 1，混合时钟）：
- `header.stamp` 用 **ROS 系统时钟** `node.get_clock().now()`（**不替换 node clock**）
- 所有 dt / 超时 / 定时器周期计算用 `gimbal_common.time_utils.now()`（= `time.monotonic()`）

⚠ **待 Ubuntu 24.04 + ROS2 Jazzy 编译验证**：本机为 Windows，无 ROS2，
本文件只做了静态一致性检查，未经 `colcon build` 与运行时验证。
"""

from __future__ import annotations

import os
import sys
from typing import Any
from typing import Dict
from typing import List
from typing import Optional
from typing import Tuple

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy
    from rclpy.qos import QoSHistoryPolicy
    from rclpy.qos import QoSProfile
    from nav_msgs.msg import OccupancyGrid
except Exception as _e:  # pragma: no cover - Windows 无 ROS2
    rclpy = None  # type: ignore[assignment]
    Node = object  # type: ignore[assignment,misc]
    OccupancyGrid = None  # type: ignore[assignment]

    class _QoSProfile:  # noqa: D401 - 占位，仅为静态检查通过
        """ROS 缺失时的占位类。"""

        def __init__(self, **kwargs: Any) -> None:
            self.kwargs = kwargs

    QoSProfile = _QoSProfile  # type: ignore[assignment]
    QoSDurabilityPolicy = None  # type: ignore[assignment]
    QoSHistoryPolicy = None  # type: ignore[assignment]

try:  # 优先用 T01 的 gimbal_common
    from gimbal_common.time_utils import now
    from gimbal_common.log_utils import get_logger, throttled
except Exception:  # pragma: no cover - Windows 无 ROS2 兜底
    from ._compat import now, get_logger, throttled

from .field_config import FieldConfig
from .grid_builder import GridMap
from .pgm_io import write_map
from .png_draw import draw_preview

# 默认参数（与 DESIGN §6.13 的 `grid.*` 命名空间一致）
DEFAULTS: Dict[str, Any] = {
    'grid.resolution': 0.05,
    'grid.width_m': 12.0,
    'grid.height_m': 12.0,
    'grid.origin_xyz': [-6.0, -6.0, 0.0],
    'grid.inflation_radius_m': 0.10,
    'grid.map_source': 'config',          # config | topic
    'grid.field_yaml': '',                # 空 = 用默认场地
    'grid.map_topic': '/map',             # map_source=topic 时订阅
    'grid.out_dir': 'maps',               # 落盘目录
    'grid.republish_period_s': 0.0,       # >0 时周期性重发（默认只发一次）
    'grid.write_preview_png': True,
}


def _resolve_field_yaml() -> str:
    """在未显式配置 `grid.field_yaml` 时，尽力定位 `field.yaml`。

    查找顺序：
      1. 环境变量 `GIMBAL_FIELD_YAML`
      2. `ament_index_python` 查到的 `gimbal_tracker_bringup` 安装 share 目录
      3. 工作区源树 `src/gimbal_tracker_bringup/config/field.yaml`
      4. 当前工作目录下的 `field.yaml`

    Returns:
        str：找到的第一个存在的文件路径；都找不到时返回空串（节点随后用默认场地）。
    """
    cands: list = []
    env = os.environ.get('GIMBAL_FIELD_YAML', '')
    if env:
        cands.append(env)
    try:  # 安装态（colcon build 之后）
        from ament_index_python.packages import get_package_share_directory
        cands.append(os.path.join(get_package_share_directory('gimbal_tracker_bringup'),
                                  'config', 'field.yaml'))
    except Exception:  # pragma: no cover - 无 ament / 包未安装
        pass
    here = os.path.dirname(os.path.abspath(__file__))     # <ws>/src/grid_mapper/grid_mapper
    ws_root = os.path.abspath(os.path.join(here, '..', '..', '..'))
    cands.append(os.path.join(ws_root, 'src', 'gimbal_tracker_bringup',
                              'config', 'field.yaml'))
    cands.append(os.path.join(os.getcwd(), 'src', 'gimbal_tracker_bringup',
                              'config', 'field.yaml'))
    cands.append(os.path.join(os.getcwd(), 'field.yaml'))
    for c in cands:
        if c and os.path.isfile(c):
            return c
    return ''


def resolve_map_origin(grid_map: Any,
                       params_origin_xyz: Any) -> Tuple[List[float], str]:
    """决定写进 `map.yaml` 的 origin，并给出不一致时的告警文本。

    **必须以栅格自身的 `origin_xy` 为准**，不能套 `grid.origin_xyz`：

    * `map_source=config`：栅格 origin 来自 `field.yaml`，可能与 `grid.origin_xyz` 不同；
    * `map_source=topic`：栅格 origin 是**外部 SLAM 地图自带**的，
      与 `grid.origin_xyz`（12×12m 窗口那个 -6,-6）毫无关系——套错会让整张图平移。

    Args:
        grid_map: `GridMap`（用其 `origin_xy`）。
        params_origin_xyz: 参数里的 `grid.origin_xyz`（仅用于比对告警）。

    Returns:
        (origin_xyz, warning)：`origin_xyz` 为 `[x, y, yaw]`；一致时 `warning` 为空串。
    """
    try:
        got = [float(grid_map.origin_xy[0]), float(grid_map.origin_xy[1]), 0.0]
    except Exception:  # pragma: no cover - 兜底
        return [0.0, 0.0, 0.0], ''
    try:
        want = [float(params_origin_xyz[0]), float(params_origin_xyz[1]),
                float(params_origin_xyz[2]) if len(params_origin_xyz) > 2 else 0.0]
    except Exception:
        return got, ''
    if abs(got[0] - want[0]) > 1e-9 or abs(got[1] - want[1]) > 1e-9:
        return got, ('grid.origin_xyz=%s 与栅格实际 origin=%s 不一致'
                     '（map_source=topic 时外部地图自带 origin）→ map.yaml 按栅格 origin 写'
                     % (want, got))
    return got, ''


class GridMapperNode(Node):  # type: ignore[misc]
    """建图节点。启动即栅格化 → 落盘 → 发布锁存话题。"""

    def __init__(self, name: str = 'grid_mapper_node') -> None:
        super().__init__(name)
        self._log = get_logger(self, name)
        self._t_start = now()
        self._params: Dict[str, Any] = {}
        for key, val in DEFAULTS.items():
            self._params[key] = self._declare(key, val)

        self._grid_map: Optional[GridMap] = None
        self._field: Optional[FieldConfig] = None

        # --- 地图来源 -------------------------------------------------------
        source = str(self._params['grid.map_source']).lower()
        if source == 'topic':
            latched = self._latched_qos()
            self.create_subscription(
                OccupancyGrid, str(self._params['grid.map_topic']),
                self._on_external_map, latched)
            self._log.info('map_source=topic: 等待外部地图 %s'
                           % str(self._params['grid.map_topic']))
        else:
            self._build_from_config()

        self._pub_grid = self.create_publisher(OccupancyGrid, '/map/grid',
                                               self._latched_qos())
        self._pub_ray = self.create_publisher(OccupancyGrid, '/map/ray_grid',
                                              self._latched_qos())
        if self._grid_map is not None:
            self._publish_all()

        period = float(self._params['grid.republish_period_s'])
        if period > 0.0:
            self.create_timer(period, self._on_republish)

    # -- 参数 -----------------------------------------------------------------
    def _declare(self, key: str, default: Any) -> Any:
        """声明参数（已由 launch 注入时直接取用）。"""
        try:
            self.declare_parameter(key, default)
            return self.get_parameter(key).value
        except Exception:  # pragma: no cover - 参数已声明
            return default

    @staticmethod
    def _latched_qos() -> Any:
        """TRANSIENT_LOCAL + depth=1 的 QoS（锁存）。"""
        if QoSProfile is None:  # pragma: no cover
            return None
        try:
            return QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            )
        except Exception:  # pragma: no cover - 不同 rclpy 版本签名差异
            return QoSProfile(depth=1)

    # -- 建图 -----------------------------------------------------------------
    def _build_from_config(self) -> None:
        """读 field.yaml → 栅格化 → 落盘 → 画预览图。"""
        path = str(self._params['grid.field_yaml'])
        if not path:
            path = _resolve_field_yaml()
        if path and os.path.isfile(path):
            field = FieldConfig.from_yaml(path)
            self._log.info('已加载 field.yaml: %s' % path)
        else:
            field = FieldConfig.from_dict({
                'grid': {
                    'resolution': float(self._params['grid.resolution']),
                    'width_m': float(self._params['grid.width_m']),
                    'height_m': float(self._params['grid.height_m']),
                    'origin_xyz': list(self._params['grid.origin_xyz']),
                    'inflation_radius_m': float(self._params['grid.inflation_radius_m']),
                },
            })
            if path:
                self._log.warn('field.yaml 不存在(%s)，使用默认场地' % path)
        self._field = field
        self._grid_map = GridMap.build(field)
        self._save_outputs()

    def _save_outputs(self) -> None:
        """写 `map.pgm` + `map.yaml` + `map_preview.png`。"""
        if self._grid_map is None:
            return
        out_dir = str(self._params['grid.out_dir'])
        origin_xyz, warn = resolve_map_origin(self._grid_map,
                                              self._params['grid.origin_xyz'])
        if warn:
            self._log.warn(warn)
        try:
            paths = write_map(out_dir, self._grid_map, origin_xyz=origin_xyz)
            self._log.info('地图已落盘: %s, %s' % (paths['pgm'], paths['yaml']))
        except Exception as exc:  # 落盘失败不能拖垮整图，但要明确报错
            self._log.error('地图落盘失败: %r' % (exc,))
            paths = {'pgm': '', 'yaml': ''}
        if bool(self._params['grid.write_preview_png']):
            try:
                boundary = self._field.boundary if self._field is not None else None
                png_path = os.path.join(out_dir, 'map_preview.png')
                draw_preview(self._grid_map, png_path, boundary=boundary)
                self._log.info('预览图已落盘: %s' % png_path)
            except Exception as exc:  # pragma: no cover
                self._log.warn('预览图绘制失败: %r' % (exc,))
        st = self._grid_map.stats()
        self._log.info('建图完成: %dx%d @ %.3fm/cell, origin=(%.2f,%.2f), '
                       'occ=%d free=%d unk=%d ray_occ=%d'
                       % (st['width'], st['height'], st['resolution'],
                          st['origin_xy'][0], st['origin_xy'][1],
                          st['n_occupied'], st['n_free'], st['n_unknown'],
                          st['n_ray_occupied']))

    # -- 回调 -----------------------------------------------------------------
    def _on_external_map(self, msg: Any) -> None:
        """`map_source=topic`：接收外部 OccupancyGrid 后建图落盘。

        ⚠ 原样采用外部地图的 `resolution` / `origin`（不重采样、不套参数），
        只按 `grid.inflation_radius_m` 额外生成「含膨胀」的 `ray_grid` 供避障；
        `/map/grid`（未膨胀）保持原始三态，方向项解析求解用它。
        """
        try:
            info = msg.info
            self._grid_map = GridMap.from_occupancy_grid(
                int(info.width), int(info.height), float(info.resolution),
                [float(info.origin.position.x), float(info.origin.position.y)],
                list(msg.data),
                inflation_radius_m=float(self._params['grid.inflation_radius_m']))
            self._save_outputs()
        except Exception as exc:
            throttled(self._log, 'error', '外部地图解析失败: %r' % (exc,),
                      'grid_mapper.ext_map_err', 1.0)

    def _on_republish(self) -> None:
        """周期性重发锁存话题（默认关闭）。"""
        self._publish_all()

    # -- 发布 -----------------------------------------------------------------
    def _publish_all(self) -> None:
        """发布 `/map/grid`（干净三态）与 `/map/ray_grid`（含膨胀）。"""
        if self._grid_map is None or OccupancyGrid is None:
            return
        try:
            self._pub_grid.publish(self._to_msg(self._grid_map, use_ray=False))
            self._pub_ray.publish(self._to_msg(self._grid_map, use_ray=True))
        except Exception as exc:  # pragma: no cover
            throttled(self._log, 'error', '地图发布失败: %r' % (exc,),
                      'grid_mapper.pub_err', 1.0)

    def _to_msg(self, gm: GridMap, use_ray: bool) -> Any:
        """GridMap → nav_msgs/OccupancyGrid。header.stamp 用 ROS 系统时钟。"""
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = float(gm.resolution)
        msg.info.width = int(gm.width)
        msg.info.height = int(gm.height)
        msg.info.origin.position.x = float(gm.origin_xy[0])
        msg.info.origin.position.y = float(gm.origin_xy[1])
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = gm.to_occupancy_data(use_ray=use_ray)
        return msg

    # -- 收尾 -----------------------------------------------------------------
    def shutdown(self) -> None:
        """退出钩子（§6.14 第 5 条）：`map_source=config` 时复写一次 maps/。"""
        try:
            if str(self._params.get('grid.map_source', 'config')).lower() == 'config':
                self._save_outputs()
        except Exception as exc:  # pragma: no cover
            self._log.warn('退出复写地图失败: %r' % (exc,))
        self._log.info('grid_mapper_node 退出，耗时 %.2fs' % (now() - self._t_start))


def main(args: Optional[list] = None) -> int:
    """节点入口。"""
    if rclpy is None:  # pragma: no cover - Windows 无 ROS2
        print('[grid_mapper_node] FATAL rclpy 不可用（需要 ROS2 Jazzy）')
        return 1
    rclpy.init(args=args)
    node = GridMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.shutdown()
        finally:
            node.destroy_node()
            rclpy.shutdown()
    return 0


if __name__ == '__main__':  # pragma: no cover
    sys.exit(main())
