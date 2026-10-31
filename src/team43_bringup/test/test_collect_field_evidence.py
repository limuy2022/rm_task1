"""现场采样器的真实DDS/只读参数服务及交接产物回归，不接触硬件。"""
import csv
import importlib.util
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import tarfile
import time

import yaml
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField, Imu
from tf2_msgs.msg import TFMessage
from std_msgs.msg import UInt64

ROOT=Path(__file__).resolve().parents[3]
SCRIPT=ROOT/'tools/collect_field_evidence.py'
spec=importlib.util.spec_from_file_location('collect_field_evidence',SCRIPT)
collector=importlib.util.module_from_spec(spec)
spec.loader.exec_module(collector)


def make_cloud(big_endian=False):
    msg=PointCloud2(height=2,width=1,point_step=26,row_step=32,is_bigendian=big_endian)
    msg.header.stamp.sec=100
    msg.header.frame_id='livox_frame'
    msg.fields=[PointField(name=n,offset=o,datatype=d,count=1) for n,o,d in
                [('x',0,7),('y',4,7),('z',8,7),('tag',16,2),('timestamp',18,8)]]
    data=bytearray(64)
    for row in range(2):
        struct.pack_into(('>' if big_endian else '<')+'fff',data,row*32,1.+row,2.,3.)
        data[row*32+16]=16
        struct.pack_into(('>' if big_endian else '<')+'d',data,row*32+18,100000000000.+row*1000000.)
    msg.data=data
    return msg


@pytest.mark.parametrize('big_endian',[False,True])
def test_cloud_snapshot_keeps_raw_units_coordinates_and_padded_layout(big_endian):
    result=collector.cloud_snapshot(make_cloud(big_endian))
    assert result['required_schema_matches_official']
    assert not result['time_semantics_confirmed']
    assert [p['x'] for p in result['points']]==[1.,2.]
    assert result['assumed_ns_span_s']==pytest.approx(.001)
    assert result['points'][0]['timestamp']==100000000000.
    msg=make_cloud(big_endian)
    msg.data=msg.data[:-1]
    assert 'sample_error' in collector.cloud_snapshot(msg)


def test_cloud_snapshot_does_not_hide_nonfinite_or_invent_missing_time():
    msg=make_cloud()
    data=bytearray(msg.data)
    struct.pack_into('<d',data,18,float('nan'))
    msg.data=data
    result=collector.cloud_snapshot(msg)
    assert result['nonfinite_sampled_times']==1
    assert result['points'][0]['timestamp'] is None
    msg.fields=[f for f in msg.fields if f.name!='timestamp']
    result=collector.cloud_snapshot(msg)
    assert not result['required_schema_matches_official']
    assert 'assumed_ns_span_s' not in result


@pytest.fixture
def source(monkeypatch):
    # 父进程与采样子进程必须使用同一通信范围，不能依赖外部shell的设置。
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    domain=170+os.getpid()%40
    context=Context()
    rclpy.init(context=context,domain_id=domain)
    # 使用明确允许读取的后端节点名，只有测试参数服务，不启动定位算法。
    node=rclpy.create_node('team43_small_point_lio',namespace='/red_standard_robot1',context=context)
    node.declare_parameter('extrinsic_T',[.1,.2,.3])
    node.declare_parameter('test_bytes',[b'x',b'y'])
    executor=SingleThreadedExecutor(context=context)
    executor.add_node(node)
    try:
        yield node,executor,domain
    finally:
        executor.shutdown()
        node.destroy_node()
        context.shutdown()


def test_collects_late_publishers_live_parameters_tf_and_archive(source,tmp_path):
    node,executor,domain=source
    output=tmp_path/'pointcloud2'
    process=subprocess.Popen([sys.executable,'-B',str(SCRIPT),'--seconds','3.5','--output-dir',str(output)],
        env=dict(os.environ,ROS_DOMAIN_ID=str(domain),ROS_LOCALHOST_ONLY='1'),
        stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    started=time.monotonic()
    publishers=None
    try:
        while process.poll() is None and time.monotonic()-started<12.:
            if publishers is None and time.monotonic()-started>.8:
                lidar=node.create_publisher(PointCloud2,
                    '/red_standard_robot1/livox/lidar',qos_profile_sensor_data)
                imu=node.create_publisher(Imu,'/red_standard_robot1/gimbal',qos_profile_sensor_data)
                tf=node.create_publisher(TFMessage,'/tf',10)
                latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
                epoch=node.create_publisher(UInt64,'/task1/gimbal_reset_epoch',latched)
                publishers=(lidar,imu,tf,epoch)
            if publishers:
                now=node.get_clock().now()
                msg=make_cloud()
                lidar.publish(msg)
                im=Imu()
                im.header.stamp=now.to_msg()
                im.header.frame_id='gimbal_link'
                im.orientation.w=1.
                im.linear_acceleration.z=9.81
                imu.publish(im)
                t=TransformStamped()
                t.header.stamp=now.to_msg()
                t.header.frame_id='evidence_world'
                t.child_frame_id='evidence_sensor'
                t.transform.translation.x=.12
                t.transform.rotation.w=1.
                tf.publish(TFMessage(transforms=[t]))
                epoch.publish(UInt64(data=123456789))
            executor.spin_once(timeout_sec=.03)
        stdout,_=process.communicate(timeout=3.)
        assert process.returncode==0,stdout
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        if publishers:
            for p in publishers: node.destroy_publisher(p)
    summary=json.loads((output/'summary.json').read_text())
    assert summary['collector_completed'] and not summary['semantics_and_accuracy_verified']
    assert summary['streams']['lidar']['selected_type']=='sensor_msgs/msg/PointCloud2'
    assert summary['streams']['lidar']['received']>0
    assert summary['streams']['image']['status']=='not_requested'
    assert 'tick' in summary['not_received']
    live=json.loads((output/'live_parameters.json').read_text())
    assert live['/red_standard_robot1/team43_small_point_lio']['parameters']['extrinsic_T']['value']==[.1,.2,.3]
    assert live['/red_standard_robot1/team43_small_point_lio']['parameters']['test_bytes']['value']==[120,121]
    clouds=json.loads((output/'cloud_samples.json').read_text())
    assert clouds and not clouds[0]['time_semantics_confirmed']
    events=[json.loads(line) for line in (output/'measurements.jsonl').read_text().splitlines()]
    assert any(e['role']=='gimbal_imu' and e['data']['orientation']['w']==1. for e in events)
    assert any(e['role']=='epoch' and e['data']['data']==123456789 for e in events)
    with (output/'tf_edges.csv').open() as f:
        assert any(r['child']=='evidence_sensor' and float(r['x'])==.12 for r in csv.DictReader(f))
    manual=yaml.safe_load((output/'manual_measurements.yaml').read_text())
    assert manual['marker_black_outer_side_m']==.075
    assert manual['lidar_points_already_transformed'] is False
    assert manual['adopted_floor_model']=='height_approximation'
    assert manual['body_imu_in_base_xyz_m'] is None
    assert manual['joint_zero_rad'] is None
    assert 'floor_three_points_and_lio_session' not in manual
    with tarfile.open(str(output)+'.tar.gz') as tar:
        assert output.name+'/summary.json' in tar.getnames()
        assert output.name+'/manual_measurements.yaml' in tar.getnames()


def test_interrupt_preserves_partial_results_and_never_overwrites(source,tmp_path):
    _,executor,domain=source
    output=tmp_path/'partial'
    command=[sys.executable,'-B',str(SCRIPT),'--seconds','30','--output-dir',str(output)]
    process=subprocess.Popen(command,env=dict(os.environ,ROS_DOMAIN_ID=str(domain),ROS_LOCALHOST_ONLY='1'),
                             stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    try:
        deadline=time.monotonic()+10.
        while time.monotonic()<deadline and not (output/'headers.csv').exists():
            assert process.poll() is None
            executor.spin_once(timeout_sec=.05)
        assert (output/'headers.csv').exists()
        process.send_signal(signal.SIGINT)
        stdout,_=process.communicate(timeout=8.)
        assert process.returncode==130,stdout
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
    summary=json.loads((output/'summary.json').read_text())
    assert summary['interrupted'] and not summary['collector_completed']
    assert Path(str(output)+'.tar.gz').exists()
    original=(output/'summary.json').read_bytes()
    repeated=subprocess.run(command,env=dict(os.environ,ROS_DOMAIN_ID=str(domain),ROS_LOCALHOST_ONLY='1'),
                            capture_output=True,text=True,timeout=5.)
    assert repeated.returncode==2
    assert (output/'summary.json').read_bytes()==original
