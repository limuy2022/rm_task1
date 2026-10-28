"""grid_mapper_node：启动即建图落盘，并发布 `/map/grid` 与 `/map/ray_grid`（均锁存）。

对应 DESIGN §3.10 / §6.11 / §6.14。

话题（§4.3）：

| 话题 | 类型 | QoS |
|---|---|---|
| `/map/grid` | `nav_msgs/OccupancyGrid`（干净三态） | TRANSIENT_LOCAL, depth=1 |
| `/map/ray_grid` | `nav_msgs/OccupancyGrid`（含膨胀，供避障） | TRANSIENT_LOCAL, depth=1 |

两种地图来源（`grid.map_source`）：

* `config`：读 `field.yaml` 栅格化（12×12 m @ 0.05 m/cell，给定点居中）——赛前离线建图/仿真。
* `topic`：订阅自己的 SLAM `/map`（`pb2025_sentry_nav` 的 slam_toolbox 地图生产链），
  **原样采用其 resolution/origin**，只额外按 `grid.inflation_radius_m` 生成 `ray_grid`。

⚠ 写 `map.yaml` 的 origin 一律取**栅格自身**的 `origin_xy`（见 `resolve_map_origin`），
不套 `grid.origin_xyz`——否则 topic 模式下整张图会按两者的差值平移。

时钟约定（主理人拍板 1，混合时钟）：
- 外部地图原样保留 `header.stamp`；配置图才使用当前 ROS 时钟
- 所有 dt / 超时 / 定时器周期计算用 `team43_common.time_utils.now()`（= `time.monotonic()`）

运行目标为 Ubuntu 22.04 + ROS2 Humble；纯核离线检查不替代 NUC 通信与建图验证。
"""

from __future__ import annotations

import os
import math
import sys
from typing import Any
from typing import Dict
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, TransformException
from nav_msgs.msg import OccupancyGrid

from team43_common.time_utils import now
from team43_common.log_utils import get_logger, throttled
from team43_common.node_utils import declare_params
from team43_common.qos_utils import latched_qos, reliable_qos

from .field_config import FieldConfig
from .grid_builder import GridMap
from .pgm_io import write_map
from .grid_geometry import planar_quaternion_yaw, compose_grid_origin, resolve_map_origin

# 默认参数（与 DESIGN §6.13 的 `grid.*` 命名空间一致）
DEFAULTS: Dict[str, Any] = {
    'grid.resolution': 0.05,
    'grid.width_m': 12.0,
    'grid.height_m': 12.0,
    'grid.origin_xyz': [-6.0, -6.0, 0.0],
    'grid.inflation_radius_m': 0.10,
    'grid.map_source': 'config',          # config | topic
    'grid.field_yaml': '',                # config 模式必须显式指定实际场地文件
    'grid.map_topic': '/map',
    'grid.source_frame': '',             # empty: validate nonempty incoming header, then resolve TF
    'grid.output_frame': 'map',
    'grid.external_map_transient_local': True,
    'grid.output_basename': 'map',        # competition uses <team>_map
             # map_source=topic 时订阅
    'grid.out_dir': 'maps',               # 落盘目录
    'grid.republish_period_s': 0.0,       # >0 时周期性重发（默认只发一次）
    'grid.write_preview_png': True,
}


class GridMapperNode(Node):  # type: ignore[misc]
    """建图节点。启动即栅格化 → 落盘 → 发布锁存话题。"""

    def __init__(self, name: str = 'grid_mapper_node') -> None:
        super().__init__(name)
        self._log = get_logger(self, name)
        self._t_start = now()
        self._params: Dict[str, Any] = declare_params(self, DEFAULTS)

        self._grid_map: Optional[GridMap] = None
        self._field: Optional[FieldConfig] = None
        self._source_stamp = None
        self._source_load_time = None
        self._pending_external = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._output_frame = str(self._params['grid.output_frame'])
        if not self._output_frame:
            raise ValueError('grid.output_frame must be nonempty')
        basename = str(self._params['grid.output_basename'])
        if not basename or os.path.basename(basename) != basename or basename in ('.', '..'):
            raise ValueError('grid.output_basename must be a plain filename stem')


        # --- 地图来源 -------------------------------------------------------
        source = str(self._params['grid.map_source']).lower()
        if source == 'topic':
            latched = (latched_qos(1)
                       if self._params['grid.external_map_transient_local']
                       else reliable_qos(1))
            self.create_subscription(
                OccupancyGrid, str(self._params['grid.map_topic']),
                self._on_external_map, latched)
            self._log.info(f'map_source=topic: 等待外部地图 '
                           f'{self._params["grid.map_topic"]}')
        else:
            if source != 'config':
                raise ValueError('grid.map_source must be config or topic')
            self._build_from_config()

        self._pub_grid = self.create_publisher(OccupancyGrid, '/map/grid',
                                               latched_qos())
        self._pub_ray = self.create_publisher(OccupancyGrid, '/map/ray_grid',
                                              latched_qos())
        if self._grid_map is not None:
            self._publish_all()

        self.create_timer(0.1, self._retry_pending_external)
        period = float(self._params['grid.republish_period_s'])
        if period > 0.0:
            self.create_timer(period, self._on_republish)

    # -- 建图 -----------------------------------------------------------------
    def _build_from_config(self) -> None:
        """读 field.yaml → 栅格化 → 落盘 → 画预览图。"""
        path = str(self._params['grid.field_yaml'])
        if not path:
            raise ValueError('grid.map_source=config requires an explicit grid.field_yaml')
        field = FieldConfig.from_yaml(os.path.expanduser(path))
        self._log.info(f'已加载 field.yaml: {path}')
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
            self._log.warning(warn)
        try:
            stem = str(self._params['grid.output_basename'])
            paths = write_map(out_dir, self._grid_map, image_name=stem+'.pgm',
                              yaml_name=stem+'.yaml', origin_xyz=origin_xyz)
            self._log.info(f'地图已落盘: {paths["pgm"]}, {paths["yaml"]}')
        except OSError as exc:  # 磁盘/权限失败不终止地图发布，必须明确报告未成功保存。
            self._log.error(f'地图落盘失败: {exc!r}')
        if bool(self._params['grid.write_preview_png']):
            from .png_draw import draw_preview
            try:
                boundary = self._field.boundary if self._field is not None else None
                png_path = os.path.join(out_dir, 'map_preview.png')
                draw_preview(self._grid_map, png_path, boundary=boundary)
                self._log.info(f'预览图已落盘: {png_path}')
            except (ImportError, OSError) as exc:  # 可选 PNG 导出调用失败不影响 PGM/YAML；模块导入错误直接暴露。
                self._log.warning(f'预览图绘制失败: {exc!r}')
        st = self._grid_map.stats()
        self._log.info(f'建图完成: {st["width"]}x{st["height"]} '
                       f'@ {st["resolution"]:.3f}m/cell, '
                       f'origin=({st["origin_xy"][0]:.2f},{st["origin_xy"][1]:.2f}), '
                       f'occ={st["n_occupied"]} free={st["n_free"]} '
                       f'unk={st["n_unknown"]} ray_occ={st["n_ray_occupied"]}')

    # -- 回调 -----------------------------------------------------------------
    def _on_external_map(self, msg: Any) -> None:
        """Compose source map origin into output frame; never just relabel it.

        Keep only the newest pending map while the replay origin TF is not yet
        available. A later timer retries, so a single latched map received before
        /judge/replay_start is not permanently lost.
        """
        self._pending_external = msg
        self._retry_pending_external()

    def _retry_pending_external(self) -> None:
        msg = self._pending_external
        if msg is None:
            return
        try:
            source = str(msg.header.frame_id)
            expected = str(self._params['grid.source_frame'])
            if not source or (expected and source != expected):
                raise ValueError(f'地图 frame_id={source!r} 与声明来源 {expected!r} 不符')
            info = msg.info
            q = info.origin.orientation
            yaw = planar_quaternion_yaw([q.x, q.y, q.z, q.w])
            origin = [float(info.origin.position.x), float(info.origin.position.y)]
            if source != self._output_frame:
                tf = self._tf_buffer.lookup_transform(self._output_frame, source,
                    Time.from_msg(msg.header.stamp), timeout=Duration(seconds=0.0))
                tr, rq = tf.transform.translation, tf.transform.rotation
                origin, yaw = compose_grid_origin(origin, yaw,
                    [tr.x, tr.y, tr.z], [rq.x, rq.y, rq.z, rq.w])
            gm = GridMap.from_occupancy_grid(
                int(info.width), int(info.height), float(info.resolution), origin,
                list(msg.data), inflation_radius_m=float(self._params['grid.inflation_radius_m']),
                origin_yaw=yaw)
            self._grid_map = gm
            self._source_stamp = msg.header.stamp
            self._source_load_time = info.map_load_time
            self._pending_external = None
            self._save_outputs()
            self._publish_all()
        except (TransformException, ValueError, TypeError, OverflowError) as exc:
            # 外部坏地图不替换有效地图；对应时刻的 TF 尚未产生时保留消息重试。
            throttled(self._log, 'warning', f'地图暂未接入（等待有效地图/对应时刻TF）: {exc!r}',
                      'team43_grid_mapper.ext_map_err')

    def _on_republish(self) -> None:
        """周期性重发锁存话题（默认关闭）。"""
        self._publish_all()

    # -- 发布 -----------------------------------------------------------------
    def _publish_all(self) -> None:
        """发布 `/map/grid`（干净三态）与 `/map/ray_grid`（含膨胀）。"""
        if self._grid_map is None:
            return
        self._pub_grid.publish(self._to_msg(self._grid_map, use_ray=False))
        self._pub_ray.publish(self._to_msg(self._grid_map, use_ray=True))

    def _to_msg(self, gm: GridMap, use_ray: bool) -> Any:
        """GridMap → OccupancyGrid，外部图保留源stamp和栅格原点yaw。"""
        msg = OccupancyGrid()
        msg.header.frame_id = self._output_frame
        msg.header.stamp = self._source_stamp if self._source_stamp is not None else self.get_clock().now().to_msg()
        if self._source_load_time is not None:
            msg.info.map_load_time = self._source_load_time
        msg.info.resolution = float(gm.resolution)
        msg.info.width = int(gm.width)
        msg.info.height = int(gm.height)
        msg.info.origin.position.x = float(gm.origin_xy[0])
        msg.info.origin.position.y = float(gm.origin_xy[1])
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.z = math.sin(gm.origin_yaw/2)
        msg.info.origin.orientation.w = math.cos(gm.origin_yaw/2)
        msg.data = gm.to_occupancy_data(use_ray=use_ray)
        return msg

    # -- 收尾 -----------------------------------------------------------------
    def shutdown(self) -> None:
        """退出钩子（§6.14 第 5 条）：`map_source=config` 时复写一次 maps/。"""
        if str(self._params['grid.map_source']).lower() == 'config':
            self._save_outputs()
        self._log.info(f'grid_mapper_node 退出，耗时 {now() - self._t_start:.2f}s')


def main(args: Optional[list] = None) -> int:
    """节点入口。"""
    rclpy.init(args=args)
    node = GridMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # 操作者 Ctrl+C 正常停机，仍执行后面的文件/节点收尾。
        pass
    finally:
        try:
            node.shutdown()
        finally:
            node.destroy_node()
            # Humble 的 SIGINT 处理器可能已经关闭 context。
            if rclpy.ok():
                rclpy.shutdown()
    return 0


if __name__ == '__main__':  # pragma: no cover
    sys.exit(main())
