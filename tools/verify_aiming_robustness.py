"""Degraded synthetic scenes -> real detector -> real PnP -> production image PID.

三层退化回归（真 ROS 2 Humble 容器，无相机/串口/电机）：

1. **检测层**：`aim_sim.render_scene` 的退化图像喂给**生产** ArUcoDetectorNode
   （真 rclpy 节点 + 发布录制），断言每档命中率；
2. **PnP 层**：生产 MarkerPnpNode 解位姿，误差界随档位放宽；
3. **闭环层**：检测 → 生产 visual_servo（AST harness，真 AxisPID）→ 理想
   角速度植物积分云台姿态 → 下一帧渲染。断言退化下仍收敛（或按丢失宽限
   语义优雅停转：指令归零、不误报 fault）。

用法：python3 -B tools/verify_aiming_robustness.py [--quick]
--quick 每档 4 姿态×3帧、闭环 90 帧；默认静态每姿态12帧。
结果 JSON 打印并写入 build/offline/aiming_robustness.json；失败退出码 1。
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import zlib

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
sys.path.insert(0, str(ROOT / 'src' / 'gimbal_perception'))
sys.path.insert(0, str(ROOT / 'src' / 'gimbal_control'))

import cv2  # noqa: E402
import rclpy  # noqa: E402

import aim_sim  # noqa: E402
import verify_perception_offline as VP  # noqa: E402
from verify_visual_servo import create_harness, corners_message, imu_message  # noqa: E402

QUICK = '--quick' in sys.argv
CAMERA = VP.CAMERA

_ZERO_CMD = SimpleNamespace(angular=SimpleNamespace(z=0.0, y=0.0))


def _stable_seed(name: str, salt: int) -> int:
    return salt + zlib.crc32(name.encode('utf-8')) % 100000

# (名字, 退化 kwargs, 命中率下限, PnP 位置误差界 = z × 系数)
# 阈值策略：除遮挡外全部 0.99+（修正斜置姿态的 180° 翻转后确定性全过）。
# 遮挡语义 = 从码边向内咬入深度/码边长（实测 OpenCV 4.5.4 / DICT_4X4_50：
# 码本体一旦被咬，检测基本必死——ArUco 对码遮挡近似二值，非渲染缺陷）：
# occlusion_pad 只吞 quiet zone，3m 小码姿态会失败 → 0.70；
# occlusion_heavy 咬码必丢 → 期望 0（丢失语义由闭环 burst/sustained_loss 覆盖）。
# occlusion_pad 的 PnP 可能被 2px 重投影防线拒绝（贴边暗块拉偏 cornerSubPix，
# 生产安全过滤在正确工作），不断言位置。
SCENARIOS = [
    ('clean',            {},                                                     1.00, 0.020),
    ('sensor_noise',     {'noise_sigma_px': 6.0},                                1.00, 0.030),
    ('texture',          {'background': 'texture'},                              0.99, 0.030),
    ('exposure',         {'gain': 0.80, 'bias': -15.0},                          0.99, 0.040),
    ('motion_blur',      {'motion_blur_px': 9.0, 'blur_dir_deg': 25.0},          0.99, 0.050),
    ('defocus',          {'defocus_px': 1.8},                                    0.99, 0.040),
    ('occlusion_pad',    {'occlusion_frac': 0.0, 'occlusion_side': 1,
                          'occlusion_u': 0.5},                                   0.70, None),
    ('occlusion_heavy',  {'occlusion_frac': 0.25},                               0.00, None),
    ('combined',         {'noise_sigma_px': 3.0, 'motion_blur_px': 5.0,
                          'blur_dir_deg': -40.0,
                          'background': 'texture'},                              0.99, 0.060),
]

POSES = [
    ((0.00, 0.00, 2.00), None),
    ((0.15, -0.10, 1.50), None),
    # 斜置视角：必须在正立基准上叠加倾斜（aim_sim.tilted_rvec），
    # 直接给小角度 rvec 会得到 180° 翻转的码、解出不同 ID。
    # 大角度姿态由 test_pnp_solver 的 0.65rad 用例覆盖。
    ((-0.20, 0.12, 2.50), aim_sim.tilted_rvec(0.10, -0.12, 0.18)),
    ((0.05, 0.05, 3.00), None),
]

# 闭环档位（遮挡走专门的 burst / sustained_loss 场景；重遮挡静态-only）
CLOSED_LOOP = ['clean', 'sensor_noise', 'texture', 'exposure',
               'motion_blur', 'defocus', 'combined', 'burst']
BURST_OCCLUSION_FRAC = 0.95
BURST_FRAMES = (28, 50)      # 该区间目标被完全遮挡


def run_static(detector, pnp, failures, report):
    for name, degr, hit_min, pnp_factor in SCENARIOS:
        rng = np.random.default_rng(_stable_seed(name, 1234))
        poses = POSES
        repeats = 3 if QUICK else 12
        hits, pnp_ok = 0, 0
        err_max = 0.0
        for sample in range(len(poses) * repeats):
            tvec, rvec = poses[sample % len(poses)]
            image, _gt = aim_sim.render_scene(CAMERA, 3, tvec, rvec, rng=rng, **degr)
            corners, result = VP.feed(detector, pnp, image, 10 + sample)
            if corners.valid and corners.marker_id == 3:
                hits += 1
            if result.valid:
                pnp_ok += 1
                err = float(np.linalg.norm(
                    np.array([result.x, result.y, result.z]) - np.array(tvec)))
                if not math.isfinite(err):
                    failures.append('%s PnP 误差非有限值' % name)
                if err > err_max:
                    err_max = err
                if pnp_factor is not None and err > max(.03, pnp_factor * tvec[2]):
                    failures.append('%s PnP 误差 %.3fm 超过该距离误差界，姿态 %s' % (name, err, tvec))
        count = len(poses) * repeats
        rate, pnp_rate = hits / count, pnp_ok / count
        row = dict(samples=count, hit_rate=rate, pnp_valid=pnp_ok, pnp_valid_rate=pnp_rate,
                   pnp_err_max_m=round(err_max, 4) if pnp_ok else None)
        report['static'][name] = row
        if rate < hit_min:
            failures.append('%s 检测命中率 %.2f < %.2f' % (name, rate, hit_min))
        if pnp_factor is not None and pnp_rate < .95:
            failures.append('%s PnP 有效率 %.2f < 0.95' % (name, pnp_rate))
        print('static %-16s hit=%.2f pnp_err<=%s' %
              (name, rate, ('%.3fm' % err_max) if pnp_ok else 'n/a'), flush=True)


def check_loss_commands(commands, name, failures):
    """按丢失时的发送索引取窗，要求实际出现三零帧，随后静默。"""
    zero = [abs(m.angular.z) <= 1e-9 and abs(m.angular.y) <= 1e-9 for m in commands]
    first_zero = next((i for i, value in enumerate(zero) if value), len(zero))
    stopped = (first_zero <= 2 and len(zero) - first_zero == 3
               and all(zero[first_zero:]))
    if not stopped:
        failures.append('%s 未完成最多两帧宽限、三零帧及静默：发送零帧序列 %s' % (name, zero))
    return dict(stop_frames=sum(zero), grace_frames=first_zero, stopped=stopped)


def check_settled(errors, name, failures):
    """最后半秒必须持续有新观测并保持精度，不能用丢失前的旧误差通过。"""
    tail = errors[-15:]
    valid = [e for e in tail if e is not None and math.isfinite(e)]
    settled = len(tail) == 15 and len(valid) >= 14 and max(valid, default=math.inf) < 8.
    if not settled:
        failures.append('%s 最后半秒未持续收敛：有效观测 %d/15，最大误差 %s' %
                        (name, len(valid), max(valid, default=None)))
    return settled


def _optical_from_state(yaw, pitch, target):
    cy, sy, cp, sp = (math.cos(yaw), math.sin(yaw),
                      math.cos(pitch), math.sin(pitch))
    R = np.array([[cy * cp, -sy, cy * sp],
                  [sy * cp, cy, sy * sp],
                  [-sp, 0., cp]])
    body = R.T @ target
    return np.array([-body[1], -body[2], body[0]])


def run_closed_loop(detector, failures, report):
    target = np.array([3., 0.25, 0.12])
    marker_id = 3
    detector._target_digit = marker_id

    scenario_degr = dict((s[0], s[1]) for s in SCENARIOS)
    for name in CLOSED_LOOP:
        degr = {} if name == 'burst' else scenario_degr[name]
        rng = np.random.default_rng(_stable_seed(name, 4321))
        node, clock, params, sent = create_harness(CAMERA)
        params.update(motion_enabled=True, run_seconds=10.)
        node.tick()

        yaw = pitch = 0.0
        first_error = error = None
        errors = []
        burst = name == 'burst'
        burst_i0 = burst_i1 = None      # 丢失起止时刻的 sent 长度（帧号≠索引：
        frames = range(90 if QUICK else 150)  # 静默期 sent 不增长）
        for k in frames:
            dt = 1. / 30
            clock.advance(dt)
            optical = _optical_from_state(yaw, pitch, target)
            kwargs = dict(degr)
            if burst and BURST_FRAMES[0] <= k < BURST_FRAMES[1]:
                kwargs['occlusion_frac'] = BURST_OCCLUSION_FRAC
            if burst and k == BURST_FRAMES[0]:
                burst_i0 = len(sent)
            elif burst and k == BURST_FRAMES[1]:
                burst_i1 = len(sent)
            image, _gt = aim_sim.render_scene(CAMERA, marker_id, optical,
                                              rng=rng, **kwargs)
            corners, ids = detector._detect(image)
            picked = detector._pick(corners, ids)
            q = (math.cos(yaw / 2) * math.cos(pitch / 2),
                 -math.sin(yaw / 2) * math.sin(pitch / 2),
                 math.cos(yaw / 2) * math.sin(pitch / 2),
                 math.sin(yaw / 2) * math.cos(pitch / 2))
            node.on_imu(imu_message(clock, q))
            msg = corners_message(clock, CAMERA['cx'], CAMERA['cy'])
            if picked is None:
                msg.valid = False
                msg.corners = []
            else:
                msg.corners = [SimpleNamespace(x=float(x), y=float(y))
                               for x, y in picked]
            node.on_corners(msg)
            node.tick()
            if getattr(node, 'fault', None):
                failures.append('%s 闭环出现 fault: %s' % (name, node.fault))
                break
            cmd = sent[-1] if sent else _ZERO_CMD
            yaw += cmd.angular.z * dt
            pitch += cmd.angular.y * dt
            if node.observation is not None and node.observation[2] is not None:
                error = np.asarray(node.observation[2])
                if first_error is None:
                    first_error = error.copy()
                    assert sent[-1].angular.z > 0 and sent[-1].angular.y < 0, \
                        '%s 首拍指令方向错误' % name
            errors.append(float(np.linalg.norm(error)) if picked is not None and node.observation is not None else None)

        row = {}
        if first_error is not None and error is not None:
            row = dict(initial_px=[round(float(v), 1) for v in first_error],
                       final_px=[round(float(v), 1) for v in error],
                       converged=check_settled(errors, name, failures))
            if not row['converged']:
                failures.append('%s 闭环未收敛：final=%s px'
                                % (name, row['final_px']))
        else:
            row = dict(converged=False, note='从未建立观测')
            failures.append('%s 闭环从未建立观测' % name)

        if burst:
            # 丢失期间的发送窗（按 sent 长度切，静默期不增长）：除 coast
            # 宽限（前 2 条衰减指令）外必须为零；3 零帧+静默是停止语义。
            row.update(check_loss_commands(sent[burst_i0:burst_i1], name, failures))
            if not row.get('converged'):
                failures.append('%s 遮挡恢复后未重新收敛' % name)
        report['closed_loop'][name] = row
        print('closed %-16s converged=%s final=%s' %
              (name, row.get('converged'), row.get('final_px')), flush=True)

    # 持续丢失：目标彻底遮挡后指令必须归零且不 fault
    name = 'sustained_loss'
    rng = np.random.default_rng(_stable_seed(name, 999))
    node, clock, params, sent = create_harness(CAMERA)
    params.update(motion_enabled=True, run_seconds=10.)
    node.tick()
    yaw = pitch = 0.0
    hidden_from = 30
    loss_start = None
    for k in range(60 if QUICK else 90):
        dt = 1. / 30
        clock.advance(dt)
        if k == hidden_from:
            loss_start = len(sent)
            if not any(abs(m.angular.z) > 1e-9 or abs(m.angular.y) > 1e-9 for m in sent):
                failures.append('持续丢失场景在遮挡前未建立运动')
        optical = _optical_from_state(yaw, pitch, target)
        kwargs = {'occlusion_frac': 0.95} if k >= hidden_from else {}
        image, _gt = aim_sim.render_scene(CAMERA, 3, optical, rng=rng, **kwargs)
        corners, ids = detector._detect(image)
        picked = detector._pick(corners, ids)
        q = (math.cos(yaw / 2) * math.cos(pitch / 2),
             -math.sin(yaw / 2) * math.sin(pitch / 2),
             math.cos(yaw / 2) * math.sin(pitch / 2),
             math.sin(yaw / 2) * math.cos(pitch / 2))
        node.on_imu(imu_message(clock, q))
        msg = corners_message(clock, CAMERA['cx'], CAMERA['cy'])
        if picked is None:
            msg.valid, msg.corners = False, []
        else:
            msg.corners = [SimpleNamespace(x=float(x), y=float(y)) for x, y in picked]
        node.on_corners(msg)
        node.tick()
        cmd = sent[-1] if sent else _ZERO_CMD
        yaw += cmd.angular.z * dt
        pitch += cmd.angular.y * dt
    metrics = check_loss_commands(sent[loss_start:], name, failures)
    report['closed_loop'][name] = dict(**metrics, fault=bool(getattr(node, 'fault', None)))
    if getattr(node, 'fault', None):
        failures.append('持续丢失不得 fault（丢失是正常路径）: %s' % node.fault)
    print('closed %-16s stopped=%s' % (name, metrics['stopped']), flush=True)


def main() -> int:
    print('OpenCV %s; quick=%s' % (cv2.__version__, QUICK), flush=True)
    rclpy.init()
    recording = VP.install_publish_recording()
    recording.start()
    failures: list = []
    report = {'kind': 'aiming_robustness', 'quick': QUICK,
              'static': {}, 'closed_loop': {}}
    try:
        detector, pnp = VP.pipeline(target=3)
        run_static(detector, pnp, failures, report)
        run_closed_loop(detector, failures, report)
    finally:
        recording.stop()
        VP.drain_created_nodes()
        if rclpy.ok():
            rclpy.shutdown()

    report['failures'] = failures
    report['passed'] = not failures
    out = ROOT / 'build/offline/aiming_robustness.json'
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n',
                   encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False), flush=True)
    if failures:
        print('FAIL: %d 项' % len(failures), flush=True)
        for f in failures:
            print(' - %s' % f, flush=True)
        return 1
    print('PASS', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
