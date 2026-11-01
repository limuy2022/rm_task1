"""比赛链审查缺陷回归：实际控制/时间缓存/写盘/launch，不访问硬件。"""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest
import yaml
from launch import LaunchContext
from launch_ros.actions import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

ROOT = Path(__file__).resolve().parents[3]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT/path)
    mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
    return mod


SERVO = load('match_servo_harness', 'tools/verify_visual_servo.py')
TIMING = load('match_timing_harness', 'tools/verify_timing_offline.py')


def joints(node, clock, yaw=0., pitch=0.):
    node.on_joints(NS(header=NS(stamp=TIMING.stamp(clock.ros), frame_id='base_link'),
                      name=['yaw', 'pitch'], position=[yaw, pitch]))


def match_controller():
    node, clock, params, sent = SERVO.create_harness(SERVO.CAMERA)
    params.update(operating_mode='match', motion_enabled=True, calibrated_epoch=42,
                  joint_limits_confirmed=True, joint_yaw_limits_rad=[-1., 1.],
                  joint_pitch_limits_rad=[-.5, .8])
    node.on_epoch(NS(data=42)); node.tick(); clock.advance(.02)
    node.on_imu(SERVO.imu_message(clock)); joints(node, clock)
    node.on_corners(SERVO.corners_message(clock, 700., 520.)); node.tick()
    assert sent and not node.fault
    return node, clock, params, sent


def feed(node, clock, yaw=0., pitch=0., center=700., world_angle=0.):
    clock.advance(.02)
    node.on_imu(SERVO.imu_message(clock, (math.cos(world_angle/2), 0., 0., math.sin(world_angle/2))))
    joints(node, clock, yaw, pitch)
    node.on_corners(SERVO.corners_message(clock, center, 520.)); node.tick()


def test_match_uses_relative_joints_not_world_rotation_or_trial_time():
    node, clock, _, sent = match_controller()
    clock.advance(200.)
    feed(node, clock, world_angle=math.radians(70))
    assert not node.fault and abs(sent[-1].angular.z) > 0


def test_measured_joint_boundary_blocks_outward_but_allows_inward():
    node, clock, _, sent = match_controller()
    feed(node, clock, yaw=-.99, center=700.)
    assert sent[-1].angular.z == 0.
    for _ in range(15): feed(node, clock, yaw=-.99, center=580.)
    assert not node.fault and sent[-1].angular.z > 0.
    feed(node, clock, yaw=-1.01)
    assert node.fault == '关节超出实测工作范围' and SERVO._is_zero(sent[-1])


def test_match_feedback_stale_and_epoch_change_stop():
    node, clock, _, sent = match_controller()
    clock.advance(.25)
    node.on_imu(SERVO.imu_message(clock)); node.on_corners(SERVO.corners_message(clock, 700., 520.)); node.tick()
    assert node.fault == '关节反馈中断' and SERVO._is_zero(sent[-1])
    node, clock, _, sent = match_controller()
    node.on_epoch(NS(data=43)); feed(node, clock)
    assert node.fault and SERVO._is_zero(sent[-1])


def test_unconfirmed_limits_do_not_enable_match_motion():
    node, clock, params, sent = match_controller()
    params['joint_limits_confirmed'] = False
    feed(node, clock)
    assert node.fault and SERVO._is_zero(sent[-1])


@pytest.mark.parametrize('epoch', [0., 1000., 1793400000.])
def test_ten_hz_pose_gap_accepts_float_rounding_but_rejects_missing_data(epoch):
    from team43_common.time_buffer import PoseHistory
    h = PoseHistory()
    h.put(epoch+.1, np.eye(4)); h.put(epoch+.2, np.eye(4))
    assert h.sample(epoch+.15, .1) is not None
    h.clear(); h.put(epoch+.1, np.eye(4)); h.put(epoch+.201, np.eye(4))
    assert h.sample(epoch+.15, .1) is None


def test_failed_map_staging_preserves_both_files(tmp_path, monkeypatch):
    from team43_grid_mapper import pgm_io as io
    pgm, meta = str(tmp_path/'43_map.pgm'), str(tmp_path/'43_map.yaml')
    io.write_map_pair(pgm, meta, np.zeros((2, 2), np.int8))
    before = [Path(p).read_bytes() for p in (pgm, meta)]
    stage = io._stage_bytes
    def fail_yaml(path, content):
        if path == meta: raise OSError('模拟磁盘写满')
        return stage(path, content)
    monkeypatch.setattr(io, '_stage_bytes', fail_yaml)
    with pytest.raises(OSError): io.write_map_pair(pgm, meta, np.full((3, 3), 100, np.int8))
    assert [Path(p).read_bytes() for p in (pgm, meta)] == before
    assert sorted(p.name for p in tmp_path.iterdir()) == ['43_map.pgm', '43_map.yaml']


def test_map_second_replace_failure_rolls_back(tmp_path, monkeypatch):
    from team43_grid_mapper import pgm_io as io
    pgm, meta = str(tmp_path/'43_map.pgm'), str(tmp_path/'43_map.yaml')
    io.write_map_pair(pgm, meta, np.zeros((2, 2), np.int8))
    before = [Path(p).read_bytes() for p in (pgm, meta)]
    replace = io.os.replace
    def fail_yaml(src, dst):
        if dst == meta: raise OSError('模拟第二次提交失败')
        return replace(src, dst)
    monkeypatch.setattr(io.os, 'replace', fail_yaml)
    with pytest.raises(OSError): io.write_map_pair(pgm, meta, np.full((3, 3), 100, np.int8))
    assert [Path(p).read_bytes() for p in (pgm, meta)] == before


def grid(value=0):
    msg = OccupancyGrid(); msg.header.frame_id = 'map'
    msg.info.width = msg.info.height = 2; msg.info.resolution = .05
    msg.info.origin.orientation.w = 1.; msg.data = [value]*4
    return msg


def test_final_map_rearmed_after_resumed_tick_and_changed_map(tmp_path, monkeypatch):
    from team43_judge_io import judge_io_node as module
    app = TIMING.judge_node(str(tmp_path))
    app._on_grid(grid()); app._n_tick = 1
    assert app._save_map('final') and app._map_saved_final
    tick = Header(); tick.stamp = TIMING.stamp(1000.); tick.frame_id = '1'
    app._on_tick(tick)
    assert not app._map_saved_final
    monkeypatch.setattr(module.time, 'monotonic', lambda: app._t_last_tick_mono+.75)
    app._on_watchdog(); assert app._map_saved_final
    app._on_grid(grid(100)); assert not app._map_saved_final
    app._on_watchdog(); assert app._map_saved_final
    from team43_grid_mapper.pgm_io import read_pgm
    assert (read_pgm(app._map_pgm)[0] == 0).all()
    app._on_grid(grid(100)); assert app._map_saved_final
    app._writer.close()


def test_queue_delay_is_included_even_when_callback_is_fast(tmp_path):
    app = TIMING.judge_node(str(tmp_path))
    warnings, errors = [], []
    app._log = NS(warning=warnings.append, error=errors.append, info=lambda _: None)
    now = [1000.2]
    app.get_clock = lambda: NS(now=lambda: NS(nanoseconds=round(now[0]*1e9)))
    tick = Header(); tick.frame_id = '1'; tick.stamp = TIMING.stamp(1000.)
    app._on_tick(tick)
    assert app._last_tick_timing['stamp_to_flush_s'] >= .199
    assert app._last_tick_timing['callback_s'] < .05
    assert any('端到端' in text for text in warnings)
    warnings.clear(); tick.stamp = TIMING.stamp(1000.2); tick.frame_id = '2'
    app._on_tick(tick); assert not warnings
    tick.stamp = TIMING.stamp(1001.); tick.frame_id = '3'; app._on_tick(tick)
    assert any('时钟异常' in text for text in errors)
    app._writer.close()


@pytest.mark.parametrize('target_id', [-1, 0, 49])
def test_match_launch_starts_without_confirmation_epoch_or_target(tmp_path, target_id):
    mod = load('match_launch_regression', 'src/team43_bringup/launch/match.launch.py')
    driver = tmp_path/'livox.json'
    # 使用CLI生成与现场相同的JSON，避免测试复制驱动配置。
    from team43_bringup.livox_config import main
    main(['--output', str(driver)])
    camera = ROOT/'src/team43_bringup/config/camera_team43.yaml'
    context = LaunchContext()
    config = tmp_path/'measured_values.yaml'
    data=yaml.safe_load((ROOT/'src/team43_bringup/config/vehicle_template.yaml').read_text())
    data['/**']['ros__parameters']={k:v for k,v in data['/**']['ros__parameters'].items() if not k.startswith('grid.')}
    config.write_text(yaml.safe_dump(data))
    context.launch_configurations.update(target_id=str(target_id),run_dir=str(tmp_path),
        map_yaml=str(ROOT/'maps/43_map.yaml'),
        vehicle_config=str(config), livox_config=str(driver),
        camera_config=str(camera), controller_params=str(ROOT/'src/team43_control/config/visual_tracking.yaml'),
        lio_config=str(ROOT/'src/team43_lio_backend/config/small_point_lio_mid360.yaml'),
        port='/never_opened_in_test', output_dir=str(tmp_path/'output'))
    actions = mod._setup(context)
    nodes = [x for x in actions if isinstance(x, Node)]
    executable = [x.node_executable for x in nodes]
    from launch_ros.utilities import evaluate_parameters
    for node in nodes:
        # 单模块创建后延后到父context执行；这里验证没有悬空的子launch参数。
        values = evaluate_parameters(context, node._Node__parameters)
        if node.node_executable in ('aruco_detector_node', 'marker_pnp_node'):
            selected = next(v['perception.target_digit'] for v in reversed(values)
                            if isinstance(v, dict) and 'perception.target_digit' in v)
            assert selected == target_id
    assert executable.count('aruco_detector_node') == executable.count('marker_pnp_node') == 1
    assert executable.count('robo_bridge_node') == 1
    assert not {'saved_map_node','cloud_mapping_node','corridor_map_node'} & set(executable)
    # robo_bridge 挂的是设备缺席重试处理器（on_exit 为可调用对象），
    # 而非其余关键进程的 _critical（on_exit 为含 Shutdown 的动作列表）。
    from launch.actions import RegisterEventHandler
    bridge = next(x for x in nodes if x.node_executable == 'robo_bridge_node')
    from launch.events.process import ProcessExited
    from launch.actions import TimerAction
    event = ProcessExited(action=bridge, returncode=mod._EXIT_DEVICE_ABSENT,
                          name='bridge', cmd=['unused'], cwd=None, env=None, pid=1)
    watch = [a.event_handler for a in actions if isinstance(a, RegisterEventHandler)
             and a.event_handler.matches(event)]
    assert len(watch) == 1
    assert any(isinstance(a, TimerAction) for a in watch[0].handle(event, context))
    assert 'match_session_gate' not in executable and 'visual_servo_node' in executable
    assert 'judge_io_node' not in executable
    snapshot = tmp_path/'vehicle_runtime.yaml'
    runtime = yaml.safe_load(snapshot.read_text())['/**']['ros__parameters']
    assert runtime['chassis.dir_mode'] == 'direct'
    assert runtime['chassis.enable_avoidance'] is False
    assert runtime['chassis.obstacle_guard_enabled'] is True
    assert runtime['chassis.obstacle_floor_valid'] is True
    assert runtime['judge.map_write_enabled'] is False
    assert (tmp_path/'output/43_map.pgm').read_bytes() == (ROOT/'maps/43_map.pgm').read_bytes()
    assert runtime['kinematics.require_confirmation'] is False
    assert runtime['kinematics.bind_current_epoch'] is True
    assert runtime['kinematics.calibration_confirmed'] is False
    assert runtime['kinematics.calibrated_epoch'] == -1
    assert runtime['estimation.geometry_confirmed'] is True
    ready = actions
    controller = next(x for x in ready if isinstance(x, Node) and x.node_executable == 'visual_servo_node')
    from launch_ros.utilities import evaluate_parameters
    configs = evaluate_parameters(context, controller._Node__parameters)
    overrides = configs[-1]
    assert overrides['motion_enabled'] is True and overrides['continuous'] is True
    assert overrides['operating_mode'] == 'match'
    context.launch_configurations.update(motion_enabled='false', continuous='false')
    stopped = next(x for x in mod._ready(context) if isinstance(x, Node))
    stopped_overrides = evaluate_parameters(context, stopped._Node__parameters)[-1]
    assert stopped_overrides['motion_enabled'] is False and stopped_overrides['continuous'] is False
    assert overrides['bind_current_epoch'] is True
    assert overrides['calibrated_epoch'] == -1



def test_simulator_rejects_default_domain_and_only_publishes_test_topics():
    from team43_judge_io.judge_sim import validate_sim_environment
    for environ in ({}, {'ROS_LOCALHOST_ONLY': '1', 'ROS_DOMAIN_ID': '0'},
                    {'ROS_LOCALHOST_ONLY': '0', 'ROS_DOMAIN_ID': '87'}):
        with pytest.raises(ValueError): validate_sim_environment(environ)
    validate_sim_environment({'ROS_LOCALHOST_ONLY': '1', 'ROS_DOMAIN_ID': '87'})


def test_lio_defaults_agree_with_driver_topics():
    from team43_common.config_loader import load_ros_params
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions
    expected = {'lidar_topic': '/red_standard_robot1/livox/lidar',
                'imu_topic': '/red_standard_robot1/livox/imu'}
    config = load_ros_params(ROOT/'src/team43_lio_backend/config/small_point_lio_mid360.yaml')
    for name, topic in expected.items(): assert config[name] == topic
    for path in ('src/team43_lio_backend/launch/lio.launch.py', 'src/team43_bringup/launch/lio_only.launch.py'):
        module = load('lio_defaults_'+Path(path).stem, path)
        args = [a for a in module.generate_launch_description().entities if isinstance(a, DeclareLaunchArgument)]
        for action in args:
            if action.name in expected:
                assert perform_substitutions(LaunchContext(), action.default_value) == expected[action.name]


def test_shared_processing_has_no_second_detector_and_monitors_critical_nodes(tmp_path):
    processing = load('match_processing_regression', 'src/team43_bringup/launch/processing.launch.py')
    from launch.actions import RegisterEventHandler
    context = LaunchContext()
    context.launch_configurations.update(vehicle_config=str(ROOT/'src/team43_bringup/config/vehicle_field.yaml'),
        require_confirmation='false',with_map='false',
        camera_config=str(ROOT/'src/team43_bringup/config/camera_team43.yaml'),
        target_id='3', submission='true', team_id='43', output_dir=str(tmp_path/'output'),
        with_perception='false', shutdown_on_exit='true')
    actions = processing._setup(context)
    nodes = [a for a in actions if isinstance(a, Node)]
    executables = [a.node_executable for a in nodes]
    assert 'aruco_detector_node' not in executables and 'marker_pnp_node' not in executables
    assert executables.count('judge_io_node') == 1
    assert 'grid_mapper_node' not in executables
    assert len([a for a in actions if isinstance(a, RegisterEventHandler)]) == len(nodes)


def test_live_observer_checks_format_without_fixed_synthetic_state():
    observer = load('live_observer_regression', 'tools/observe_submission.py')
    line = '3.1,1.2,3.0,1.2,0.0,0.0,0.2,0.1,1793400000.100000'
    assert observer.check_line(line, fixed_state=False) == 1793400000.1
    with pytest.raises(ValueError, match='固定状态'): observer.check_line(line)
    with pytest.raises(ValueError, match='六位'): observer.check_line(line[:-1], fixed_state=False)


def test_atomic_map_preserves_read_permissions(tmp_path):
    import stat
    from team43_grid_mapper.pgm_io import write_pgm
    path = tmp_path/'43_map.pgm'
    write_pgm(path, np.zeros((2, 2), np.int8))
    assert stat.S_IMODE(path.stat().st_mode) == 0o644
    path.chmod(0o640)
    write_pgm(path, np.full((2, 2), 100, np.int8))
    assert stat.S_IMODE(path.stat().st_mode) == 0o640


def test_height_approximation_and_launch_use_same_lio_geometry(tmp_path):
    from team43_bringup.lio_geometry import approximate_floor
    from team43_common.config_loader import load_ros_params
    cfg = load_ros_params(ROOT/'src/team43_lio_backend/config/small_point_lio_mid360.yaml')
    floor, confirmed, source = approximate_floor(cfg)
    up = np.asarray(cfg['output_world_up']); up /= np.linalg.norm(up)
    assert floor == pytest.approx(up @ np.asarray(cfg['extrinsic_T'])-cfg['lidar_height_m'])
    assert confirmed and source == 'height_approximation'
    assert approximate_floor({}) == (0., False, 'manual')
    with pytest.raises(ValueError): approximate_floor({**cfg, 'output_world_up': [0.,0.,0.]})
    launch = load('approximate_floor_launch', 'src/team43_bringup/launch/cloud_mapping.launch.py')
    context = LaunchContext()
    context.launch_configurations.update(lio_params_file=str(ROOT/'src/team43_lio_backend/config/small_point_lio_mid360.yaml'),
        params_file=str(ROOT/'src/team43_grid_mapper/config/cloud_mapping.yaml'),
        floor_z='auto', floor_confirmed='auto', output_dir=str(tmp_path))
    node = launch._setup(context)[0]
    from launch_ros.utilities import evaluate_parameters
    over = evaluate_parameters(context, node._Node__parameters)[-1]
    assert over['floor_z'] == pytest.approx(floor)
    assert over['floor_confirmed'] and over['floor_source'] == 'height_approximation'


def test_stale_replay_cannot_lock_map_submission_or_reset_base_source(tmp_path):
    """旧锁存起点先到、新起点后到：三条实际回调必须一致选择新轮。"""
    from team43_localization.base_pose_provider_node import BasePoseProviderNode
    clock, mapper = TIMING.map_node()
    clock.now = 2000.
    mapper._replay_not_before = 2000.
    mapper._on_base_pose(TIMING.odom(2000., (9., 10., 0.)))
    judge = TIMING.judge_node(str(tmp_path/'submission'))
    judge._replay_not_before = 2000.
    resets = []
    base = BasePoseProviderNode.__new__(BasePoseProviderNode)
    base._replay_not_before = 2000.
    base._last_replay_start = None
    base._last_published_stamp = 2000.
    base._source = NS(reset=lambda: resets.append(True))
    base._log = TIMING.LOG
    def deliver(t):
        msg = Header(stamp=TIMING.stamp(t))
        mapper._on_origin_freeze(msg)
        judge._on_replay_start(msg)
        base._on_replay_start(msg)
    try:
        deliver(1990.)
        assert mapper._origin_stamp is None and not mapper.frozen
        assert not judge._origin_frozen and not judge._pub_origin_freeze.messages
        assert not resets and base._last_published_stamp == 2000.
        deliver(2000.)
        assert mapper.frozen and mapper._origin_stamp == 2000.
        assert judge._origin_frozen and judge._replay_stamp == 2000.
        assert len(judge._pub_origin_freeze.messages) == len(resets) == 1
        deliver(2000.)
        deliver(2001.)
        assert mapper._origin_stamp == judge._replay_stamp == 2000.
        assert len(judge._pub_origin_freeze.messages) == len(resets) == 1
    finally:
        judge._writer.close()


def test_direct_match_binds_first_epoch_but_still_stops_on_reset():
    from team43_localization.gimbal_kinematics import GimbalKinematics, SynchronizedKinematics
    params = yaml.safe_load((ROOT/'src/team43_bringup/config/vehicle_template.yaml').read_text())['/**']['ros__parameters']
    model = GimbalKinematics({k.removeprefix('kinematics.'): v for k,v in params.items()
                             if k.startswith('kinematics.')}, require_confirmation=False)
    sync = SynchronizedKinematics(model, calibrated_epoch=-1, bind_current_epoch=True)
    sync.observe_epoch(123)
    assert sync.enabled and sync.calibrated_epoch == 123
    sync.observe_epoch(123)
    assert sync.enabled
    sync.observe_epoch(456)
    assert sync.fault and not sync.enabled
    node, clock, knobs, sent = match_controller()
    knobs.update(bind_current_epoch=True, calibrated_epoch=-1)
    node.tick()
    assert not node.fault


def test_direct_match_imu_before_latched_epoch_waits_without_fault():
    node, clock, knobs, sent = SERVO.create_harness(SERVO.CAMERA)
    knobs.update(operating_mode='match', motion_enabled=True, bind_current_epoch=True,
                 calibrated_epoch=-1, joint_limits_confirmed=True,
                 joint_yaw_limits_rad=[-1., 1.], joint_pitch_limits_rad=[-.5, .8])
    node.tick()
    clock.advance(.02)
    node.on_imu(SERVO.imu_message(clock))
    node.tick()
    assert not node.fault and not sent
    node.on_epoch(NS(data=999))
    feed(node, clock)
    assert not node.fault and sent


def test_tick_before_explicit_start_does_not_invent_origin(tmp_path):
    clock, mapper = TIMING.map_node()
    judge = TIMING.judge_node(str(tmp_path/'output'))
    try:
        mapper._on_base_pose(TIMING.odom(1000., (2., -1., 0.)))
        tick = Header(stamp=TIMING.stamp(1000.04), frame_id='0')
        judge._on_tick(tick)
        assert not judge._origin_frozen and not judge._pub_origin_freeze.messages
        assert mapper._origin_stamp is None and not mapper.frozen
        assert judge._writer.rows == 1  # 不等待另一个话题而错过本拍写盘。
        explicit = Header(stamp=TIMING.stamp(1000.02), frame_id='record')
        mapper._on_origin_freeze(explicit)
        judge._on_replay_start(explicit)
        clock.now = 1000.12
        mapper._on_base_pose(TIMING.odom(1000.1, (2., -.9, 0.)))
        assert mapper.frozen
        assert mapper._origin_stamp == judge._replay_stamp == 1000.02
        assert mapper._T_map_odom[1,3] == pytest.approx(.98)
    finally:
        judge._writer.close()


def test_match_bridge_watch_respawns_device_absent_and_shuts_down_on_fatal():
    """退出码3（设备不存在）→延迟重发该节点；其它退出码→全链 Shutdown。"""
    from launch.actions import EmitEvent, RegisterEventHandler, TimerAction
    from launch.events import Shutdown
    mod = load('match_launch_respawn', 'src/team43_bringup/launch/match.launch.py')
    made = []

    def factory():
        node = Node(package='team43_serial_bridge', executable='robo_bridge_node', name='robo_bridge')
        made.append(node)
        return node

    watch = mod._respawn_on_device_absent(factory(), factory)
    assert isinstance(watch, RegisterEventHandler)
    from launch.events.process import ProcessExited
    def exited(code):
        return ProcessExited(action=made[0], returncode=code, name='bridge',
                             cmd=['unused'], cwd=None, env=None, pid=1)
    retry = watch.event_handler.handle(exited(mod._EXIT_DEVICE_ABSENT), LaunchContext())
    assert any(isinstance(a, TimerAction) for a in retry) and len(made) == 2
    fatal = watch.event_handler.handle(exited(1), LaunchContext())
    assert any(isinstance(a, EmitEvent) and isinstance(a.event, Shutdown) for a in fatal)
