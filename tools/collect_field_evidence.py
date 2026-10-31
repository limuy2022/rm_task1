#!/usr/bin/env python3
"""只读采集现场接口、时间、位姿、TF和运行参数，生成可交接压缩包。

不打开串口，不启动算法/驱动，不发布控制、评分、TF或传感器消息，
只调用 ListParameters/GetParameters。实物尺寸与机械限位保留为待填项。
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import shutil
import socket
import struct
import sys
import tarfile
import time

ROOT = Path(__file__).resolve().parents[1]
# 只观察赛题一相关的既有接口；名称可通过 --topic 逐项覆盖。
STREAMS = {
    'lidar': ('/red_standard_robot1/livox/lidar', ('sensor_msgs/msg/PointCloud2', 'livox_ros_driver2/msg/CustomMsg')),
    'lidar_imu': ('/red_standard_robot1/livox/imu', ('sensor_msgs/msg/Imu',)),
    'gimbal_imu': ('/red_standard_robot1/gimbal', ('sensor_msgs/msg/Imu',)),
    'gimbal_state': ('/gimbal/state', ('team43_interfaces/msg/GimbalState',)),
    'lio_odom': ('/red_standard_robot1/aft_mapped_to_init', ('nav_msgs/msg/Odometry',)),
    'base_raw': ('/gimbal/base_pose_raw', ('nav_msgs/msg/Odometry',)),
    'base_map': ('/gimbal/base_pose', ('nav_msgs/msg/Odometry',)),
    'corners': ('/team43_tracking/marker_corners', ('team43_interfaces/msg/MarkerCorners',)),
    'processing_corners': ('/perception/marker_corners', ('team43_interfaces/msg/MarkerCorners',)),
    'marker': ('/perception/marker_cam', ('team43_interfaces/msg/MarkerObservation',)),
    'tracking_marker': ('/team43_tracking/marker_cam', ('team43_interfaces/msg/MarkerObservation',)),
    'target': ('/estimation/target_state', ('team43_interfaces/msg/TargetState',)),
    'advice': ('/chassis/advice', ('team43_interfaces/msg/ChassisAdvice',)),
    'epoch': ('/task1/gimbal_reset_epoch', ('std_msgs/msg/UInt64',)),
    'tracking_status': ('/team43_tracking/status', ('std_msgs/msg/String',)),
    'mapping_status': ('/team43/cloud_mapping/status', ('std_msgs/msg/String',)),
    'source_map': ('/team43/slam_map', ('nav_msgs/msg/OccupancyGrid',)),
    'map': ('/map/grid', ('nav_msgs/msg/OccupancyGrid',)),
    'camera_info': ('/camera_info', ('sensor_msgs/msg/CameraInfo',)),
    'image': ('/image_raw', ('sensor_msgs/msg/Image',)),
    'tick': ('/judge/tick', ('std_msgs/msg/Header',)),
    'replay_start': ('/judge/replay_start', ('std_msgs/msg/Header',)),
    'tf': ('/tf', ('tf2_msgs/msg/TFMessage',)),
    'tf_static': ('/tf_static', ('tf2_msgs/msg/TFMessage',)),
}
OWN_NODES = (
    '/robo_bridge', '/team43_tracking/visual_servo_node',
    '/team43_tracking/aruco_detector_node', '/team43_tracking/marker_pnp_node',
    '/aruco_detector_node', '/marker_pnp_node', '/gimbal_kinematics_node',
    '/base_pose_provider_node', '/map_frame_broadcaster_node', '/target_estimator_node',
    '/judge_io_node', '/team43/cloud_mapping_node', '/red_standard_robot1/team43_small_point_lio',
    '/grid_mapper_node', '/chassis_advisor_node', '/gimbal_bearing_node', '/gimbal_controller_node',
)
# 现场驱动仅查询与输入解释直接相关的公开参数，不打开参数指向的文件。
DRIVER_NODES = {
    '/livox_driver_node': ['use_sim_time','xfer_format','multi_topic','frame_id','publish_freq','user_config_path'],
    '/livox_lidar_publisher': ['use_sim_time','xfer_format','multi_topic','frame_id','publish_freq','user_config_path'],
    '/hik_camera': ['use_sim_time','camera_info_url','camera_frame','frame_id','mfs_path','exposure_time','gain'],
}
SCALARS = {1: ('b',1), 2: ('B',1), 3: ('h',2), 4: ('H',2), 5: ('i',4), 6: ('I',4), 7: ('f',4), 8: ('d',8)}
MANUAL = '''# 此文件仅供补充实测记录，不能作为ROS参数文件加载。
# null表示未确认；程序不会从IMU、TF或默认配置自动猜出这些值。
record_type: field_evidence_manual_measurements
marker_black_outer_side_m: null
marker_measurement_photo: null
lidar_model_and_serial: null
lidar_input_frame_definition: null
lidar_points_already_transformed: null
lidar_to_imu_R_row_major: null
lidar_origin_in_imu_m: null
lidar_extrinsic_source: null
imu_actual_acceleration_full_scale_and_unit: null
imu_actual_gyro_full_scale_and_unit: null
imu_full_scale_source: null
base_center_definition: null
body_imu_in_base_xyz_m: null
body_imu_in_base_rpy_rad: null
yaw_mount_in_base_xyz_m: null
yaw_mount_in_base_rpy_rad: null
yaw_to_pitch_xyz_m: null
pitch_to_G_xyz_m: null
pitch_to_G_rpy_rad: null
camera_link_in_G_xyz_m: null
camera_link_in_G_rpy_rad: null
camera_optical_center_proxy_and_uncertainty: null
joint_zero_definition: null
joint_zero_rad: null
measured_yaw_limits_rad: null
measured_pitch_limits_rad: null
alignment_window_and_known_mechanical_pose: null
odom_from_imu_world_rpy_rad: null
alignment_epoch_and_reset_history: null
floor_z_in_current_lio_world_m: null
floor_three_points_and_lio_session: null
lowest_impassable_obstacle_m: null
map_extent_and_ground_tolerance_basis: null
command_yaw_direction_test: null
command_pitch_direction_test: null
measured_stop_behavior_and_time: null
validated_tracking_range: null
tf_edge_owners_and_basis: null
judge_interface_and_clock_confirmation: null
'''


def safe_json(value):
    """保留非有限数的缺口为null，避免输出非法JSON，不将其变成0。"""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(k): safe_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [safe_json(v) for v in value]
    if isinstance(value, (bytes, bytearray)):
        return list(value)
    return value


def write_json(path, value):
    path.write_text(json.dumps(safe_json(value), ensure_ascii=False, indent=2, allow_nan=False)+'\n')


def cloud_snapshot(msg, type_name, limit=128):
    """抽取真实坐标/标签/时间，不保存整帧data，不自动认定单位。"""
    header_s = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
    info = {'type': type_name, 'header_stamp_s': header_s, 'frame_id': msg.header.frame_id,
            'time_semantics_confirmed': False, 'points': []}
    timestamps = []
    if type_name == 'livox_ros_driver2/msg/CustomMsg':
        info.update(timebase_ns=msg.timebase, declared_points=msg.point_num, actual_points=len(msg.points))
        total = len(msg.points)
        for i in sample_indices(total, limit):
            p = msg.points[i]
            raw = msg.timebase + p.offset_time
            timestamps.append(raw)
            info['points'].append(dict(index=i,x=p.x,y=p.y,z=p.z,tag=p.tag,line=p.line,
                                       offset_time_ns=p.offset_time,absolute_time_ns=raw))
    else:
        info.update(width=msg.width, height=msg.height, point_step=msg.point_step,
                    row_step=msg.row_step, is_bigendian=msg.is_bigendian, data_bytes=len(msg.data),
                    fields=[dict(name=f.name, offset=f.offset, datatype=f.datatype, count=f.count) for f in msg.fields])
        fields = {}
        duplicates = set()
        for field in msg.fields:
            if field.name in fields:
                duplicates.add(field.name)
            fields[field.name] = field
        required = {'x':7,'y':7,'z':7,'tag':2,'timestamp':8}
        info['required_schema_matches_official'] = all(
            name not in duplicates and name in fields and fields[name].datatype==kind and fields[name].count==1
            and fields[name].offset+SCALARS[kind][1]<=msg.point_step for name,kind in required.items())
        total = msg.width * msg.height
        if msg.point_step<=0 or msg.row_step<msg.width*msg.point_step or len(msg.data)<msg.height*msg.row_step:
            info['sample_error'] = '点步长、行步长或数据长度无效'
            return safe_json(info)
        for i in sample_indices(total, limit):
            base = (i//msg.width)*msg.row_step + (i%msg.width)*msg.point_step
            point = {'index': i}
            for name in ('x','y','z','intensity','tag','line','timestamp','offset_time','time','t'):
                f = fields.get(name)
                if f is None or name in duplicates or f.count!=1 or f.datatype not in SCALARS:
                    continue
                fmt, size = SCALARS[f.datatype]
                if f.offset+size>msg.point_step:
                    continue
                point[name] = struct.unpack_from(('>' if msg.is_bigendian else '<')+fmt,msg.data,base+f.offset)[0]
            if 'timestamp' in point:
                timestamps.append(point['timestamp'])
            info['points'].append(point)
    info['sampled_points'] = len(info['points'])
    finite = [x for x in timestamps if math.isfinite(x)]
    info['nonfinite_sampled_times'] = len(timestamps)-len(finite)
    if finite:
        lo, hi = min(finite), max(finite)
        info.update(raw_time_min=lo, raw_time_max=hi, assumed_ns_span_s=(hi-lo)*1e-9,
                    assumed_ns_first_minus_header_s=lo*1e-9-header_s,
                    assumed_ns_last_minus_header_s=hi*1e-9-header_s)
    return safe_json(info)


def sample_indices(total, limit):
    if total<=0:
        return []
    if limit==1:
        return [0]
    return sorted({i*(total-1)//(min(total,limit)-1) for i in range(min(total,limit))}) if total>1 else [0]


def copy_source_configs(output):
    paths = ['vehicle43.yaml','src/team43_bringup/config/camera_team43.yaml',
             'src/team43_bringup/config/vehicle_template.yaml','src/team43_bringup/config/tracker_params.yaml',
             'src/team43_lio_backend/config/small_point_lio_mid360.yaml',
             'src/team43_grid_mapper/config/cloud_mapping.yaml','src/team43_control/config/visual_tracking.yaml',
             'src/team43_perception/config/tracker_defaults.yaml']
    saved = []
    for name in paths:
        source = ROOT/name
        if source.resolve().is_relative_to(ROOT.resolve()) and source.is_file():
            target = output/'source_configs'/name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source,target)
            saved.append(name)
    return saved


class Collector:
    def __init__(self, args, output):
        import rclpy
        from rclpy.parameter import Parameter
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from rcl_interfaces.srv import GetParameters, ListParameters
        from rosidl_runtime_py.utilities import get_message
        from rosidl_runtime_py.convert import message_to_ordereddict
        self.ros, self.get_message, self.to_dict = rclpy, get_message, message_to_ordereddict
        self.GetParameters, self.ListParameters = GetParameters, ListParameters
        self.args, self.output = args, output
        self.node = rclpy.create_node('team43_field_evidence_'+str(os.getpid()), enable_rosout=False,
                                      parameter_overrides=[Parameter('use_sim_time',value=args.use_sim_time)])
        self.sensor = QoSProfile(depth=5,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.latched = QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.streams = {role:dict(topic=topic,expected=list(types),selected_type=None,received=0,
                                  valid_received=0,header_samples=0,backwards=0,duplicates=0,
                                  first_stamp_s=None,last_stamp_s=None,min_age_s=None,max_age_s=None,
                                  enabled=(role!='image' or args.with_image_headers),status='waiting_for_publisher')
                        for role,(topic,types) in STREAMS.items()}
        for role, topic in args.topic:
            self.streams[role]['topic'] = topic
        self.subscriptions, self.last_stamp, self.graph, self.parameters, self.clients = {}, {}, {}, {}, {}
        self.cloud_samples, self.camera_infos, self.map_infos = [], {}, {}
        self.files = {}
        for name in ('measurements.jsonl','graph_history.jsonl','headers.csv','tf_edges.csv'):
            self.files[name] = (output/name).open('x',newline='')
        self.header_writer = csv.writer(self.files['headers.csv'])
        self.tf_writer = csv.writer(self.files['tf_edges.csv'])
        self.header_writer.writerow(['role','topic','type','stamp_s','receipt_ros_s','age_s','frame_id','valid'])
        self.tf_writer.writerow(['topic','parent','child','stamp_s','receipt_ros_s','x','y','z','qx','qy','qz','qw'])
        self.last_graph_signature = None

    def event(self, file, value):
        self.files[file].write(json.dumps(safe_json(value),ensure_ascii=False,allow_nan=False,separators=(',',':'))+'\n')

    def header(self, role, msg, receipt):
        state = self.streams[role]
        h = msg if state['selected_type']=='std_msgs/msg/Header' else getattr(msg,'header',None)
        if h is None:
            return
        ns = h.stamp.sec*1_000_000_000+h.stamp.nanosec
        stamp_s, age = ns*1e-9, (receipt-ns)*1e-9
        valid = bool(getattr(msg,'valid',True))
        self.header_writer.writerow([role,state['topic'],state['selected_type'],stamp_s,receipt*1e-9,age,h.frame_id,valid])
        # 无效角点的时间可为检测节点当前时间，不混入真实图像延迟统计。
        if not valid:
            return
        previous = self.last_stamp.get(role)
        if previous is not None:
            state['backwards'] += int(ns<previous)
            state['duplicates'] += int(ns==previous)
        self.last_stamp[role] = ns
        state['header_samples'] += 1
        if state['first_stamp_s'] is None:
            state['first_stamp_s'] = stamp_s
        state['last_stamp_s'] = stamp_s
        state['min_age_s'] = age if state['min_age_s'] is None else min(state['min_age_s'],age)
        state['max_age_s'] = age if state['max_age_s'] is None else max(state['max_age_s'],age)

    def callback(self, role):
        def receive(msg):
            state = self.streams[role]
            if state['status']!='subscribed':
                return
            receipt = self.node.get_clock().now().nanoseconds
            state['received'] += 1
            state['valid_received'] += int(bool(getattr(msg,'valid',True)))
            kind = state['selected_type']
            if role in ('tf','tf_static'):
                for tf in msg.transforms:
                    p,q=tf.transform.translation,tf.transform.rotation
                    self.tf_writer.writerow([state['topic'],tf.header.frame_id,tf.child_frame_id,
                        tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9,receipt*1e-9,p.x,p.y,p.z,q.x,q.y,q.z,q.w])
                return
            self.header(role,msg,receipt)
            if role=='lidar':
                if len(self.cloud_samples)<self.args.cloud_frames:
                    self.cloud_samples.append(cloud_snapshot(msg,kind,self.args.cloud_samples))
                return
            if role in ('source_map','map'):
                self.map_infos[role] = dict(header=self.to_dict(msg.header),info=self.to_dict(msg.info),
                    cells=len(msg.data),unknown_cells=msg.data.count(-1),free_cells=msg.data.count(0),
                    occupied_cells=msg.data.count(100))
                return
            if role=='camera_info':
                self.camera_infos[role] = self.to_dict(msg)
                return
            if role=='image':
                data = dict(height=msg.height,width=msg.width,encoding=msg.encoding,step=msg.step,
                            header=self.to_dict(msg.header),image_payload_saved=False)
            else:
                data = self.to_dict(msg)
            self.event('measurements.jsonl',dict(role=role,topic=state['topic'],type=kind,
                       receipt_ros_s=receipt*1e-9,receipt_monotonic_s=time.monotonic(),data=data))
        return receive

    def refresh(self):
        graph = {}
        for role,state in self.streams.items():
            endpoints = self.node.get_publishers_info_by_topic(state['topic'])
            graph[role] = dict(topic=state['topic'],publishers=[dict(
                node=p.node_namespace.rstrip('/')+'/'+p.node_name,type=p.topic_type,
                gid=bytes(p.endpoint_gid).hex(),reliability=p.qos_profile.reliability.name,
                durability=p.qos_profile.durability.name,depth=p.qos_profile.depth) for p in endpoints])
            if not state['enabled']:
                state['status']='not_requested'
                continue
            types = {p.topic_type for p in endpoints}
            selected = state['selected_type']
            if selected and types and types!={selected}:
                state['status']='type_changed_or_conflict'
                continue
            if selected:
                # 同类型发布者消失/恢复可继续观察；回调本身不会产生伪样本。
                if state['status']!='type_changed_or_conflict':
                    state['status']='subscribed'
                continue
            if len(types)!=1 or not types.issubset(state['expected']):
                state['status']='waiting_for_publisher' if not types else 'unsupported_type_or_conflict'
                continue
            kind = next(iter(types))
            try:
                cls = self.get_message(kind)
                qos = self.latched if role in ('replay_start','tf_static','epoch','map','source_map') else self.sensor
                self.subscriptions[role] = self.node.create_subscription(cls,state['topic'],self.callback(role),qos)
                state['selected_type'],state['status'] = kind,'subscribed'
                state.pop('type_support_error',None)
            except (ImportError,AttributeError,ValueError,RuntimeError) as exc:
                state['status']='type_support_unavailable'
                state['type_support_error']=str(exc)
        self.graph = graph
        signature = json.dumps(graph,sort_keys=True)
        if signature!=self.last_graph_signature:
            self.event('graph_history.jsonl',dict(receipt_monotonic_s=time.monotonic(),graph=graph))
            self.last_graph_signature=signature
        self.query_parameters()

    def query_parameters(self):
        available = {namespace.rstrip('/')+'/'+name for name,namespace in self.node.get_node_names_and_namespaces()}
        for name in (*OWN_NODES,*DRIVER_NODES):
            if name not in available or name in self.parameters:
                continue
            self.parameters[name] = dict(status='waiting_for_service',source='live_parameter_service')
        for name,entry in list(self.parameters.items()):
            if entry['status']!='waiting_for_service':
                continue
            service = self.GetParameters if name in DRIVER_NODES else self.ListParameters
            suffix = '/get_parameters' if name in DRIVER_NODES else '/list_parameters'
            key = name+suffix
            if key not in self.clients:
                self.clients[key] = self.node.create_client(service,key)
            client = self.clients[key]
            if not client.service_is_ready():
                continue
            entry['status']='request_pending'
            request = service.Request()
            if name in DRIVER_NODES:
                request.names = DRIVER_NODES[name]
                self.request_values(name,request.names,client)
            else:
                request.depth=0
                future=client.call_async(request)
                def listed(result,name=name):
                    try:
                        names=sorted(result.result().result.names)
                        if len(names)>512:
                            raise ValueError('节点参数超过512项，拒绝无界采集')
                        self.request_values(name,names)
                    except Exception as exc:
                        self.parameters[name].update(status='read_error',error=str(exc))
                future.add_done_callback(listed)

    def request_values(self, name, names, client=None):
        if client is None:
            key=name+'/get_parameters'
            client=self.node.create_client(self.GetParameters,key)
            self.clients[key]=client
        request=self.GetParameters.Request()
        request.names=names
        future=client.call_async(request)
        def received(result):
            try:
                values=result.result().values
                if len(values)!=len(names):
                    raise ValueError('参数响应数量与请求不一致')
                fields={1:'bool_value',2:'integer_value',3:'double_value',4:'string_value',5:'byte_array_value',
                        6:'bool_array_value',7:'integer_array_value',8:'double_array_value',9:'string_array_value'}
                parameters={}
                for key,value in zip(names,values):
                    raw=getattr(value,fields[value.type]) if value.type in fields else None
                    if value.type==5:
                        raw=[v[0] if isinstance(v,bytes) else v for v in raw]
                    elif value.type>=6:
                        raw=list(raw)
                    parameters[key]=dict(type=int(value.type),value=raw)
                self.parameters[name].update(status='collected',parameters=parameters)
            except Exception as exc:
                self.parameters[name].update(status='read_error',error=str(exc))
        future.add_done_callback(received)

    def save(self, interrupted, error):
        for entry in self.parameters.values():
            if entry['status'] in ('waiting_for_service','request_pending'):
                entry['status']='not_returned_in_collection_window'
        summary = dict(collector_completed=not interrupted and error is None,interrupted=interrupted,error=error,
                       semantics_and_accuracy_verified=False,seconds_requested=self.args.seconds,
                       collector_uses_sim_time=self.args.use_sim_time,streams=self.streams,
                       not_received=[role for role,s in self.streams.items() if s['enabled'] and s['received']==0],
                       parameter_nodes_not_seen=sorted(set((*OWN_NODES,*DRIVER_NODES))-set(self.parameters)),
                       limitations=['采到消息不证明精度、真实同步、量程、尺寸或机械限位',
                                    'Humble Python回调不提供逐条TF的发布者GID，边归属须另确认',
                                    'best-effort观察到tick不证明正式reliable订阅匹配',
                                    '源码配置快照不等于已运行节点的参数；以live_parameters为运行读数',
                                    '时间单位的纳秒计算仅为官方格式假设，不自动确认设备时钟'])
        write_json(self.output/'summary.json',summary)
        write_json(self.output/'graph.json',self.graph)
        write_json(self.output/'live_parameters.json',self.parameters)
        write_json(self.output/'cloud_samples.json',self.cloud_samples)
        write_json(self.output/'camera_info.json',self.camera_infos)
        write_json(self.output/'map_metadata.json',self.map_infos)
        for stream in self.files.values():
            stream.close()
        self.node.destroy_node()
        return summary


def parse_args(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=15.,help='采样窗口秒数，默认15，范围1～600')
    parser.add_argument('--output-dir',type=Path,help='必须为不存在的新目录；默认自动生成')
    parser.add_argument('--topic',action='append',default=[],metavar='角色=/话题',help='覆盖STREAMS中的一个话题，如 lidar=/livox/lidar')
    parser.add_argument('--with-image-headers',action='store_true',help='额外接收原图但只记头；有全图接收开销，默认关闭')
    parser.add_argument('--use-sim-time',action='store_true',help='仅在仿真采样时使用/clock，实机不加')
    parser.add_argument('--cloud-frames',type=int,default=5,help='最多保存几帧点云摘要，默认5，范围1～30')
    parser.add_argument('--cloud-samples',type=int,default=128,help='每帧最多抽样点数，默认128，范围1～2048')
    args=parser.parse_args(argv)
    if not math.isfinite(args.seconds) or not 1<=args.seconds<=600:
        parser.error('seconds须为1～600的有限数')
    if not 1<=args.cloud_frames<=30 or not 1<=args.cloud_samples<=2048:
        parser.error('cloud-frames/cloud-samples超出范围')
    overrides=[]
    for text in args.topic:
        role,separator,topic=text.partition('=')
        if not separator or role not in STREAMS or not topic.startswith('/'):
            parser.error('--topic须为已知角色=/绝对话题；角色：'+','.join(STREAMS))
        overrides.append((role,topic))
    args.topic=overrides
    return args


def main(argv=None):
    args=parse_args(argv)
    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
    except ImportError as exc:
        print('请在已source ROS 2 Humble的NUC终端运行；不需要pip安装。原因：'+str(exc),file=sys.stderr)
        return 2
    output=(args.output_dir or ROOT/'build/offline'/('field_evidence_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid()))).expanduser().absolute()
    try:
        output.mkdir(parents=True,exist_ok=False)
    except OSError as exc:
        print('不能创建新采样目录，保留原文件：'+str(exc),file=sys.stderr)
        return 2
    (output/'manual_measurements.yaml').write_text(MANUAL)
    configs=copy_source_configs(output)
    environment={key:os.environ.get(key) for key in ('ROS_DISTRO','ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','RMW_IMPLEMENTATION')}
    write_json(output/'session.json',dict(start_utc=datetime.now(timezone.utc).isoformat(),hostname=socket.gethostname(),
               python=sys.version,environment=environment,source_configs=configs,
               operations=['DDS订阅','图发现','ListParameters','GetParameters','本队文件快照'],
               opens_serial=False,starts_driver_or_controller=False,publishes_control_or_judge=False))
    # 目录内容是观测证据，不自动生成可启用运动的整车配置。
    interrupted,error,collector=False,None,None
    try:
        rclpy.init(args=[])
        collector=Collector(args,output)
        start=time.monotonic()
        next_refresh,next_progress=start,start
        print('只读采样开始，保持已有驱动；不会开启运动。目录：'+str(output),flush=True)
        while time.monotonic()-start<args.seconds:
            now=time.monotonic()
            if now>=next_refresh:
                collector.refresh()
                next_refresh=now+.5
            if now>=next_progress:
                print('已收到：'+', '.join(f'{role}={s["received"]}' for role,s in collector.streams.items() if s['received']),flush=True)
                next_progress=now+5.
            rclpy.spin_once(collector.node,timeout_sec=.05)
        collector.refresh()
    except (KeyboardInterrupt,ExternalShutdownException):
        interrupted=True
    except Exception as exc:
        error=type(exc).__name__+': '+str(exc)
    finally:
        if collector is not None:
            summary=collector.save(interrupted,error)
        else:
            summary=dict(collector_completed=False,interrupted=interrupted,error=error)
            write_json(output/'summary.json',summary)
        if rclpy.ok():
            rclpy.shutdown()
    (output/'README.md').write_text('''# 现场只读采样证据

summary.json给出采样状态与缺样本，未开赛/未启动节点无数据不等于故障，也不等于验收通过。
measurements.jsonl保存IMU完整数值/四元数、位姿、角点/PnP、epoch、状态和接收到的评分Header。
headers.csv记录源时间/接收年龄；cloud_samples.json只保存点云元数据和抽样点，不保存整帧data。
tf_edges.csv保存实际父子边及变换；graph.json/graph_history.jsonl保存发布节点、类型、QoS、GID。
Humble Python不提供逐消息TF来源GID，不能直接把边和发布者逐一配对。
live_parameters.json是本次只读参数服务响应，type=0/value=null表示该参数未声明；并非确认值为0。
source_configs是本队源码文件参考，不能替代live_parameters证明实际生效配置。
camera_info.json/map_metadata.json仅为收到的元数据，没有图像或地图完整payload。
manual_measurements.yaml中的尺寸、外参、零位、限位等必须实测；null不代表0，不能直接加载为ROS参数。
本采样器不发布控制/评分/TF/传感器消息，不打开串口，不读取参数路径指向的外部配置文件。
补填手工测量后请重新压缩目录；本次自动生成的tar.gz不会随目录后续修改更新。
''')
    archive=Path(str(output)+'.tar.gz')
    try:
        with tarfile.open(archive,'x:gz') as tar:
            tar.add(output,arcname=output.name)
    except OSError as exc:
        print('数据目录已保留，压缩失败：'+str(exc),file=sys.stderr)
        return 1
    print('采样目录：'+str(output)+'\n交接压缩包：'+str(archive),flush=True)
    if error:
        print('采样异常，已有数据已保存：'+error,file=sys.stderr)
        return 1
    print('未收到的角色：'+', '.join(summary.get('not_received',[])),flush=True)
    print('采集完成不代表精度/安装参数已确认；实测待填项见manual_measurements.yaml。',flush=True)
    return 130 if interrupted else 0


if __name__=='__main__':
    raise SystemExit(main())
