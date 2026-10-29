"""先确认估计器传感器订阅就绪，再启动会立即发布数据的仿真源。"""
import time


def wait_for_sensor_readers(node, topics, pump, timeout=15.):
    """只认可生产后端的订阅；等待期间继续处理发现事件并检查子进程。"""
    deadline = time.monotonic() + timeout
    pending = list(topics)
    while time.monotonic() < deadline:
        pending = [topic for topic in topics if not any(
            endpoint.node_name == 'team43_small_point_lio'
            and endpoint.node_namespace == '/sim/gazebo/lio'
            for endpoint in node.get_subscriptions_info_by_topic(topic))]
        if not pending:
            return
        pump()
    raise RuntimeError('Point-LIO 传感器订阅未就绪：' + ', '.join(pending))
