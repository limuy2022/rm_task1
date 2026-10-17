"""只读取/image_raw，用DICT_4X4_50列出画面中的ID；不发送云台命令。"""
import time


def main():
    import cv2
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
    from sensor_msgs.msg import Image
    if not hasattr(cv2, 'aruco'):
        print('当前OpenCV没有aruco。请保存错误，不在公用NUC安装或覆盖依赖。')
        return 1
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = (cv2.aruco.DetectorParameters() if hasattr(cv2.aruco, 'DetectorParameters')
              else cv2.aruco.DetectorParameters_create())
    detector = cv2.aruco.ArucoDetector(dictionary, params) if hasattr(cv2.aruco, 'ArucoDetector') else None
    rclpy.init()
    node = Node('id_reader', namespace='team47_check')
    latest, seen_ids, count = [None], set(), 0
    qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     history=QoSHistoryPolicy.KEEP_LAST)
    node.create_subscription(Image, '/image_raw', lambda msg: latest.__setitem__(0, msg), qos)
    print('将一张码平整地放在镜头前，持续读取最多10秒……', flush=True)
    deadline = time.monotonic() + 10
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            msg, latest[0] = latest[0], None
            if msg is None:
                continue
            count += 1
            enc = str(msg.encoding).lower()
            channels = 3 if enc in ('rgb8', 'bgr8') else 1
            if enc not in ('rgb8', 'bgr8', 'mono8', '8uc1'):
                print(f'图像编码{enc}未被本小工具支持，请保留encoding信息。')
                return 1
            if msg.height <= 0 or msg.width <= 0 or msg.step < msg.width * channels:
                continue
            buffer = np.frombuffer(msg.data, dtype=np.uint8)
            if buffer.size < msg.height * msg.step:
                continue
            rows = buffer[:msg.height * msg.step].reshape(msg.height, msg.step)[:, :msg.width * channels]
            image = rows.reshape(msg.height, msg.width, channels) if channels == 3 else rows
            gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY if enc == 'rgb8' else cv2.COLOR_BGR2GRAY) if channels == 3 else image
            _, ids, _ = (detector.detectMarkers(gray) if detector is not None
                         else cv2.aruco.detectMarkers(gray, dictionary, parameters=params))
            if ids is not None:
                for mid in ids.flatten():
                    mid = int(mid)
                    if mid not in seen_ids:
                        seen_ids.add(mid)
                        print(f'识别到 ArUco ID：{mid}', flush=True)
        print(f'检查结束：处理图像{count}帧，发现ID={sorted(seen_ids)}')
        if not count:
            print('未收到图像：检查/image_raw发布者、ROS域和QoS。')
        elif not seen_ids:
            print('收到图像但未识别码：检查画面清晰度、码是否完整、字典是否为DICT_4X4_50。')
        return 0 if seen_ids else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
