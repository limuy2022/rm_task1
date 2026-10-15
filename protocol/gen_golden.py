#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成黄金向量 golden_frames.txt（PROTOCOL.md v1.4，2026-09-30）。

只用标准库，输出完全确定（无随机数、无时间戳）。
用法：
    python gen_golden.py            # 写出与本脚本同目录的 golden_frames.txt 并自检
    python gen_golden.py --out X    # 写到指定路径
    python gen_golden.py --check    # 只重新解析已有的 golden_frames.txt 并自检

脚本里有三部分互相独立：
    1) 编码：build_frame / pack_* （struct.pack）
    2) 解码：decode_frame （手工移位拆字节，不复用编码代码）
    3) 流式解析：StreamParser （逐字节状态机，行为等同 PROTOCOL.md 第 3.4 节）
每个用例都会：编码 -> decode_frame 往返比对 -> 写文件；写完后再从文件读回，
重新解码、重新跑物理检查和流式检查。任何一步失败都会 raise，脚本以非零码退出。
"""
import math
import os
import struct
import sys

# --------------------------------------------------------------------------
# 协议常量（与 PROTOCOL.md 一致）
# --------------------------------------------------------------------------
G = 9.80665
ACCEL_SCALE = 0.00718260498046875            # m/s^2 / LSB   (24 g 量程, = 24*9.80665/32768)
GYRO_SCALE = 0.00106526443603169529841533860381  # rad/s / LSB (2000 dps 量程)
Q_SCALE = 32760.0
Q_NORM_MIN = 0.5                              # 解码后 |q| 小于它的帧视为损坏
SEQ_RESET_BACK = 100                          # seq 回退这么多帧或更多，判定 MCU 已复位

HEAD = 0xAA
ADDR_PC2MCU = 0xEE
ADDR_MCU2PC = 0xFF
FUNC_CMD = 0x30
FUNC_IMU = 0x01
LEN_CMD = 8
LEN_IMU = 26
MAX_FRAME = 60

FUNC_SPEC = {  # func -> (addr, data_len)
    FUNC_CMD: (ADDR_PC2MCU, LEN_CMD),
    FUNC_IMU: (ADDR_MCU2PC, LEN_IMU),
}

DIR_OF_ADDR = {ADDR_PC2MCU: "pc2mcu", ADDR_MCU2PC: "mcu2pc"}

assert abs(ACCEL_SCALE - 24.0 * G / 32768.0) < 1e-15
assert abs(GYRO_SCALE - 2000.0 * math.pi / 180.0 / 32768.0) < 1e-15


# --------------------------------------------------------------------------
# 编码侧
# --------------------------------------------------------------------------
def checksum(buf):
    """sum = sum(buf) & 0xFF；sum_sum = 每步累加 sum 的和 & 0xFF。"""
    s1 = 0
    s2 = 0
    for b in buf:
        s1 = (s1 + b) & 0xFF
        s2 = (s2 + s1) & 0xFF
    return s1, s2


def build_frame(addr, func, data, head=HEAD, length=None):
    length = len(data) if length is None else length
    body = bytes([head, addr, func, length]) + bytes(data)
    s1, s2 = checksum(body)
    return body + bytes([s1, s2])


def pack_cmd(yaw, pitch):
    return struct.pack("<ff", yaw, pitch)


def pack_cmd_words(yaw_u32, pitch_u32):
    return struct.pack("<II", yaw_u32, pitch_u32)


def pack_imu(seq, stamp_us, accel_raw, gyro_raw, q_raw):
    return struct.pack("<HI3h3h4h", seq, stamp_us, *accel_raw, *gyro_raw, *q_raw)


def cmd_frame(yaw, pitch):
    return build_frame(ADDR_PC2MCU, FUNC_CMD, pack_cmd(yaw, pitch))


def imu_frame(seq, stamp_us, accel_raw, gyro_raw, q_raw):
    return build_frame(ADDR_MCU2PC, FUNC_IMU, pack_imu(seq, stamp_us, accel_raw, gyro_raw, q_raw))


# --------------------------------------------------------------------------
# 解码侧（独立实现：手工移位）
# --------------------------------------------------------------------------
def _u16(b, o):
    return b[o] | (b[o + 1] << 8)


def _u32(b, o):
    return b[o] | (b[o + 1] << 8) | (b[o + 2] << 16) | (b[o + 3] << 24)


def _i16(b, o):
    v = _u16(b, o)
    return v - 65536 if v >= 32768 else v


def _f32(b, o):
    return struct.unpack("<f", bytes(b[o:o + 4]))[0]


def decode_frame(frame):
    """返回 (ok, reason, values)。values 为有序 dict（ok 为 False 时为 None）。

    检查顺序：head -> 帧长与 len 字段自洽 -> func 已知 -> addr 与 func 匹配
             -> len 等于该 func 的规定值 -> 校验 -> 数值有限 / 四元数范数。
    reason 只是说明文字，使用方只需比较 accept / reject。
    0x30 的应用层检查是 isfinite；0x01 的应用层检查是四元数范数（raw/32760 后 < 0.5 视为损坏）。
    """
    n = len(frame)
    if n < 6 or frame[0] != HEAD:
        return False, "bad_head", None
    if n != frame[3] + 6 or n > MAX_FRAME:
        return False, "bad_len", None
    func = frame[2]
    if func not in FUNC_SPEC:
        return False, "unknown_func", None
    want_addr, want_len = FUNC_SPEC[func]
    if frame[1] != want_addr:
        return False, "bad_addr", None
    if frame[3] != want_len:
        return False, "bad_len", None
    s1, s2 = checksum(frame[:frame[3] + 4])
    if s1 != frame[frame[3] + 4] or s2 != frame[frame[3] + 5]:
        return False, "bad_checksum", None
    d = 4  # 数据区起点
    if func == FUNC_CMD:
        yaw = _f32(frame, d + 0)
        pitch = _f32(frame, d + 4)
        if not (math.isfinite(yaw) and math.isfinite(pitch)):
            return False, "non_finite", None
        return True, "-", {"yaw_omega": yaw, "pitch_omega": pitch}
    # FUNC_IMU
    v = {}
    v["seq"] = _u16(frame, d + 0)
    v["stamp_us"] = _u32(frame, d + 2)
    for i, k in enumerate(("ax_raw", "ay_raw", "az_raw")):
        v[k] = _i16(frame, d + 6 + 2 * i)
    for i, k in enumerate(("gx_raw", "gy_raw", "gz_raw")):
        v[k] = _i16(frame, d + 12 + 2 * i)
    for i, k in enumerate(("qw_raw", "qx_raw", "qy_raw", "qz_raw")):
        v[k] = _i16(frame, d + 18 + 2 * i)
    v["ax"] = v["ax_raw"] * ACCEL_SCALE
    v["ay"] = v["ay_raw"] * ACCEL_SCALE
    v["az"] = v["az_raw"] * ACCEL_SCALE
    v["gx"] = v["gx_raw"] * GYRO_SCALE
    v["gy"] = v["gy_raw"] * GYRO_SCALE
    v["gz"] = v["gz_raw"] * GYRO_SCALE
    v["qw"] = v["qw_raw"] / Q_SCALE
    v["qx"] = v["qx_raw"] / Q_SCALE
    v["qy"] = v["qy_raw"] / Q_SCALE
    v["qz"] = v["qz_raw"] / Q_SCALE
    qn = math.sqrt(v["qw"] ** 2 + v["qx"] ** 2 + v["qy"] ** 2 + v["qz"] ** 2)
    if not qn >= Q_NORM_MIN:
        return False, "bad_quat", None
    return True, "-", v


def fmt_value(x):
    if isinstance(x, int):
        return "%d" % x
    return "%.9g" % x


def fmt_decoded(values):
    if values is None:
        return "-"
    return " ".join("%s=%s" % (k, fmt_value(v)) for k, v in values.items())


def hexstr(frame):
    return " ".join("%02X" % b for b in frame)


# --------------------------------------------------------------------------
# 流式解析（独立实现，PROTOCOL.md 3.4 节的状态机）
# --------------------------------------------------------------------------
class StreamParser:
    def __init__(self, target_addr):
        self.addr = target_addr
        self.funcs = {f: s[1] for f, s in FUNC_SPEC.items() if s[0] == target_addr}
        self._reset()
        self.accepted = []

    def _reset(self):
        self.state = "HEAD"
        self.buf = bytearray()
        self.remain = 0

    def feed(self, data):
        for b in data:
            self._byte(b)

    def _restart_with(self, b):
        self._reset()
        if b == HEAD:
            self.buf.append(b)
            self.state = "ADDR"

    def _byte(self, b):
        if self.state == "HEAD":
            if b == HEAD:
                self.buf = bytearray([b])
                self.state = "ADDR"
        elif self.state == "ADDR":
            if b == self.addr:
                self.buf.append(b)
                self.state = "FUNC"
            else:
                self._restart_with(b)
        elif self.state == "FUNC":
            if b in self.funcs:
                self.buf.append(b)
                self.state = "LEN"
            else:
                self._restart_with(b)
        elif self.state == "LEN":
            if b == self.funcs[self.buf[2]]:
                self.buf.append(b)
                self.remain = b + 2
                self.state = "BODY"
            else:
                self._restart_with(b)
        else:  # BODY
            self.buf.append(b)
            self.remain -= 1
            if self.remain == 0:
                ok, _reason, _v = decode_frame(bytes(self.buf))
                if ok:
                    self.accepted.append(hexstr(self.buf))
                self._reset()  # 校验失败也整帧丢弃，不回扫


# --------------------------------------------------------------------------
# 物理辅助：ZYX（yaw-pitch），无 roll，云台只有 yaw 与 pitch 两个关节
# --------------------------------------------------------------------------
def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def q_zy(psi, theta):
    qz = (math.cos(psi / 2), 0.0, 0.0, math.sin(psi / 2))
    qy = (math.cos(theta / 2), 0.0, math.sin(theta / 2), 0.0)
    return qmul(qz, qy)


def q_axis_angle(axis, ang):
    n = math.sqrt(sum(a * a for a in axis))
    s = math.sin(ang / 2) / n
    return (math.cos(ang / 2), axis[0] * s, axis[1] * s, axis[2] * s)


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def pitch_of(q):
    w, x, y, z = q
    s = 2 * (w * y - z * x)
    return math.asin(max(-1.0, min(1.0, s)))


def rot_t_apply(q, v):
    """返回 R(q)^T v（world 向量 -> gimbal_link 系）。q 为 body->world 旋转。"""
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    # R 的行（body->world），R^T v = 各列对 v 的点积
    r = [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
         [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
         [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]
    return tuple(sum(r[i][j] * v[i] for i in range(3)) for j in range(3))


def rnd(x):
    """四舍五入到整数，恰好 .5 时远离零，与 C 的 lroundf 一致。

    不同于 Python 自带的 round()（就近取偶）和 C 的 lrintf（默认舍入模式下也是就近取偶）。
    固件按 lroundf 编码；float32 与 double 的差异最多带来 ±1 LSB，所以编码侧比较允许 ±1 LSB。
    """
    return int(math.floor(abs(x) + 0.5)) * (1 if x >= 0 else -1)


def imu_from_state(seq, stamp_us, psi, theta, psid, thetad):
    """由关节状态生成一帧 0x01 的原始整数（与 virtual_gimbal 的物理模型相同）。"""
    q = q_zy(psi, theta)
    f_b = (-G * math.sin(theta), 0.0, G * math.cos(theta))      # R^T (0,0,+g)
    w_b = (-psid * math.sin(theta), thetad, psid * math.cos(theta))
    accel = tuple(rnd(a / ACCEL_SCALE) for a in f_b)
    gyro = tuple(rnd(w / GYRO_SCALE) for w in w_b)
    qr = tuple(max(-32760, min(32760, rnd(c * Q_SCALE))) for c in q)
    return seq, stamp_us, accel, gyro, qr


def imu_frame_from_state(*args):
    return imu_frame(*imu_from_state(*args))


STATE_KEYS = ("psi", "theta", "psid", "thetad")


def mk_state(psi, theta, psid, thetad):
    """把关节状态先按文件里的写法（%.9g）取整一次，再用取整后的值生成帧。

    这样读回文件时由 state 文本重新生成的帧，与文件里的 hex 逐位一致。
    """
    return tuple(float("%.9g" % x) for x in (psi, theta, psid, thetad))


def fmt_state(st):
    if st is None:
        return "-"
    return " ".join("%s=%s" % (k, fmt_value(v)) for k, v in zip(STATE_KEYS, st))


def parse_state(text):
    if text == "-":
        return None
    kv = dict(t.split("=") for t in text.split())
    assert tuple(kv) == STATE_KEYS, "bad state field: %s" % text
    return tuple(float(kv[k]) for k in STATE_KEYS)


def imu_transition(prev_seq, prev_stamp, seq, stamp):
    """相邻两帧 0x01 的判定（PROTOCOL.md 第 6.3 节）。返回 (kind, dt_us, lost)。

    kind = "reset"：stamp 倒退（无符号差 >= 2^31）或 seq 回退 SEQ_RESET_BACK 帧以上，判定 MCU 已复位。
                    这一帧不计丢帧，不参与求 dt（dt_us、lost 为 None、0）。
    kind = "normal"：dt_us 用无符号减法；lost = max(0, int16(dseq) - 1)。
    串口不会乱序，所以没有“重复/乱序丢弃”这一说。
    """
    dt_us = (stamp - prev_stamp) & 0xFFFFFFFF
    dseq_s = ((seq - prev_seq + 32768) & 0xFFFF) - 32768        # 按 int16 解读
    if dt_us >= 0x80000000 or dseq_s <= -SEQ_RESET_BACK:
        return "reset", None, 0
    return "normal", dt_us, max(0, dseq_s - 1)


class Lcg:
    """确定性伪随机（仅用于加“噪声”）。"""
    def __init__(self, seed):
        self.x = seed

    def next(self, span):
        self.x = (self.x * 1103515245 + 12345) & 0x7FFFFFFF
        return (self.x >> 8) % (2 * span + 1) - span


# --------------------------------------------------------------------------
# 用例清单
# --------------------------------------------------------------------------
class Case:
    def __init__(self, name, frame, expect, reason="-", group="-", note="-", func=None, state=None):
        self.name = name
        self.state = state          # None 或 (psi, theta, psid, thetad)
        self.frame = bytes(frame)
        self.expect = expect
        self.reason = reason
        self.group = group
        self.note = note
        self.func = self.frame[2] if func is None else func
        self.dir = DIR_OF_ADDR.get(self.frame[1], "unknown") if self.frame[0] == HEAD \
            else DIR_OF_ADDR.get(self.frame[1], "unknown")


def flip(frame, index, mask=0x01):
    b = bytearray(frame)
    b[index] ^= mask
    return bytes(b)


def build_cases():
    cases = []
    A = lambda *a, **k: cases.append(Case(*a, expect="accept", **k))
    R = lambda name, frame, reason, **k: cases.append(Case(name, frame, "reject", reason=reason, **k))

    # ---- 0x30 角速度指令：正、负、零、十进制小数、大值（帧层接受，应用层再限幅）----
    A("cmd_zero", cmd_frame(0.0, 0.0), note="both zero; also the stop command")
    A("cmd_yaw_pos", cmd_frame(1.5, 0.0), note="yaw only, positive")
    A("cmd_pitch_neg", cmd_frame(0.0, -0.5), note="pitch only, negative")
    A("cmd_both_pos", cmd_frame(0.75, 0.25))
    A("cmd_both_neg", cmd_frame(-2.25, -1.0))
    A("cmd_mixed", cmd_frame(3.0, -0.125))
    A("cmd_decimal", cmd_frame(0.1, -0.3), note="values not exact in binary; compare as float32")
    A("cmd_large_frame_ok", cmd_frame(100.0, -100.0),
      note="valid at frame layer; the application must clamp to omega_max, not reject")

    def AS(name, seq, stamp, st, **k):
        """由关节状态 st=(psi, theta, psid, thetad) 生成的 0x01 帧，state 字段写入文件。"""
        st = mk_state(*st)
        A(name, imu_frame_from_state(seq, stamp, *st), state=st, **k)

    # ---- 0x01 IMU ----
    AS("imu_static_level", 0, 1000000, (0.0, 0.0, 0.0, 0.0),
       group="static", note="rest and level: accel~(0,0,+g), q=identity, gyro=0")
    AS("imu_static_tilt_pitch", 1, 1002000, (0.0, 0.2, 0.0, 0.0),
       group="static", note="rest, pitch=0.2 rad (nose down): accel~(-g*sin,0,g*cos), qy>0")
    lcg = Lcg(20260930)
    a_n = (lcg.next(3), lcg.next(3), 1365 + lcg.next(3))
    g_n = (lcg.next(2), lcg.next(2), lcg.next(2))
    A("imu_static_noisy", imu_frame(2, 1004000, a_n, g_n, (32760, 0, 0, 0)),
      group="static", note="level with small deterministic noise and gyro bias, q=identity")

    # 物理用例：绕 +z 正转（pitch=0.2 保证 gx 也非零），相邻两帧相隔 2 ms（500 Hz）
    psi0, th, psid = 0.3, 0.2, 3.0
    dt = 0.002
    AS("phys_yaw_pos_a", 100, 2000000, (psi0, th, psid, 0.0),
       group="physical_yaw_pos", note="frame 1 of 2: yaw rate +3.0 rad/s about world z, pitch fixed 0.2")
    AS("phys_yaw_pos_b", 101, 2002000, (psi0 + psid * dt, th, psid, 0.0),
       group="physical_yaw_pos", note="frame 2 of 2: gz>0 and yaw(q) larger than frame 1")
    # 物理用例：pitch 正向（低头为正）
    th0, thd = 0.1, 2.0
    AS("phys_pitch_pos_a", 200, 3000000, (0.5, th0, 0.0, thd),
       group="physical_pitch_pos", note="frame 1 of 2: pitch rate +2.0 rad/s about +y (nose down)")
    AS("phys_pitch_pos_b", 201, 3002000, (0.5, th0 + thd * dt, 0.0, thd),
       group="physical_pitch_pos", note="frame 2 of 2: gy>0 and pitch(q) larger than frame 1")

    # seq 与 stamp_us 回绕：dt = (b - a) mod 2^32 = 2000 us，seq 增量 = (b - a) mod 65536 = 1
    A("imu_wrap_a", imu_frame(65535, 4294966000, (0, 0, 1365), (0, 0, 0), (32760, 0, 0, 0)),
      group="wrap_pair", note="seq=65535, stamp_us=4294966000 (about 1296 us before the u32 wrap)")
    A("imu_wrap_b", imu_frame(0, 704, (0, 0, 1365), (0, 0, 0), (32760, 0, 0, 0)),
      group="wrap_pair", note="seq=0, stamp_us=704 : dt_us=(704-4294966000) mod 2^32=2000, dseq=1")

    # MCU 复位：seq 与 stamp 都倒退。两帧都是 accept；接收方在第二帧上判定“MCU 已复位”，
    # 重设 prev_seq、prev_stamp，重置滤波器，这一帧不计丢帧，也不参与求 dt。
    AS("imu_reset_a", 5000, 60000000, (0.0, 0.0, 0.0, 0.0),
       group="reset_pair", note="before MCU reset: seq=5000, stamp_us=60000000 (60 s uptime)")
    AS("imu_reset_b", 0, 2000000, (0.0, 0.0, 0.0, 0.0),
       group="reset_pair", note="after MCU reset: seq=0, stamp_us=2000000 (backwards): reset, not lost frames, no dt")

    # 高位与符号扩展
    A("imu_extreme_raw",
      imu_frame(32769, 2147483649, (-32768, 32767, -1), (32767, -32768, 255), (-32760, 0, 0, 0)),
      group="extreme", note="seq>=0x8000, stamp_us>=0x80000000, int16 extremes, q=(-1,0,0,0)")
    qm = tuple(max(-32760, min(32760, rnd(c * Q_SCALE))) for c in q_axis_angle((1.0, 2.0, 3.0), 0.7))
    A("imu_negative_mix", imu_frame(300, 5000000, (-1000, 2000, -3000), (-100, 200, -300), qm),
      group="extreme", note="mixed signs; accel and q are not mutually consistent, frame-layer test only")

    # ---- 反例：PC -> MCU 方向（addr=0xEE）----
    base = cmd_frame(3.0, -0.125)
    R("rej_cmd_bad_sum", flip(base, len(base) - 2), "bad_checksum", note="sum byte flipped")
    R("rej_cmd_bad_sum_sum", flip(base, len(base) - 1), "bad_checksum", note="sum_sum byte flipped")
    R("rej_cmd_data_bit_flip", flip(base, 6, 0x10), "bad_checksum", note="one data bit flipped, checksum unchanged")
    R("rej_cmd_len4", build_frame(ADDR_PC2MCU, FUNC_CMD, struct.pack("<f", 1.0)), "bad_len",
      note="len=4 with a valid checksum")
    R("rej_cmd_len12", build_frame(ADDR_PC2MCU, FUNC_CMD, struct.pack("<fff", 1.0, 2.0, 3.0)), "bad_len",
      note="len=12 with a valid checksum")
    R("rej_cmd_len0", build_frame(ADDR_PC2MCU, FUNC_CMD, b""), "bad_len", note="len=0 with a valid checksum")
    R("rej_cmd_len55_over60", build_frame(ADDR_PC2MCU, FUNC_CMD, bytes([0x11 + (i % 7) for i in range(55)])),
      "bad_len", note="len=55 -> frame 61 bytes, over the 60 byte limit")
    R("rej_cmd_nan_yaw", build_frame(ADDR_PC2MCU, FUNC_CMD, pack_cmd_words(0x7FC00000, 0x3F000000)),
      "non_finite", note="yaw=quiet NaN, pitch=0.5, valid checksum")
    R("rej_cmd_snan_pitch", build_frame(ADDR_PC2MCU, FUNC_CMD, pack_cmd_words(0x3F000000, 0x7FA00000)),
      "non_finite", note="pitch=signaling NaN")
    R("rej_cmd_posinf_pitch", build_frame(ADDR_PC2MCU, FUNC_CMD, pack_cmd_words(0x00000000, 0x7F800000)),
      "non_finite", note="pitch=+inf")
    R("rej_cmd_neginf_yaw", build_frame(ADDR_PC2MCU, FUNC_CMD, pack_cmd_words(0xFF800000, 0x00000000)),
      "non_finite", note="yaw=-inf")
    R("rej_cmd_bad_head", build_frame(ADDR_PC2MCU, FUNC_CMD, pack_cmd(1.0, 1.0), head=0xAB), "bad_head",
      note="first byte 0xAB instead of 0xAA, valid checksum")
    R("rej_cmd_unknown_func", build_frame(ADDR_PC2MCU, 0x31, pack_cmd(1.0, 1.0)), "unknown_func",
      note="func=0x31 is not defined by this kit")
    R("rej_imu_addr_ee", build_frame(ADDR_PC2MCU, FUNC_IMU, pack_imu(1, 1, (0, 0, 1365), (0, 0, 0), (32760, 0, 0, 0))),
      "bad_addr", note="0x01 sent with addr 0xEE")

    # ---- 反例：MCU -> PC 方向（addr=0xFF）----
    ib = imu_frame_from_state(400, 6000000, 0.0, 0.0, 0.0, 0.0)
    R("rej_imu_bad_sum", flip(ib, len(ib) - 2), "bad_checksum", note="sum byte flipped")
    R("rej_imu_bad_sum_sum", flip(ib, len(ib) - 1), "bad_checksum", note="sum_sum byte flipped")
    R("rej_imu_data_bit_flip", flip(ib, 20, 0x40), "bad_checksum", note="one data bit flipped, checksum unchanged")
    R("rej_imu_len24", build_frame(ADDR_MCU2PC, FUNC_IMU, bytes(ib[4:28])), "bad_len",
      note="len=24 (short) with a valid checksum")
    R("rej_imu_len28", build_frame(ADDR_MCU2PC, FUNC_IMU, bytes(ib[4:30]) + b"\x01\x02"), "bad_len",
      note="len=28 (long) with a valid checksum")
    R("rej_imu_bad_head", build_frame(ADDR_MCU2PC, FUNC_IMU, bytes(ib[4:30]), head=0xAB), "bad_head",
      note="first byte 0xAB, valid checksum")
    R("rej_imu_unknown_func", build_frame(ADDR_MCU2PC, 0x31, bytes(ib[4:30])), "unknown_func",
      note="func=0x31 is not defined by this kit")
    R("rej_cmd_addr_ff", build_frame(ADDR_MCU2PC, FUNC_CMD, pack_cmd(1.0, 1.0)), "bad_addr",
      note="0x30 sent with addr 0xFF")
    R("rej_imu_q_zero", imu_frame(500, 7000000, (0, 0, 1365), (0, 0, 0), (0, 0, 0, 0)), "bad_quat",
      note="valid checksum and len, q=(0,0,0,0): frame layer accepts, application must drop (|q|<0.5)")
    return cases


# --------------------------------------------------------------------------
# 物理与流式检查（生成时与读回文件时共用）
# --------------------------------------------------------------------------
def q_of(v):
    return (v["qw"], v["qx"], v["qy"], v["qz"])


def check_gravity(v, tol=0.1):
    """accel 应约等于 R(q)^T (0,0,+g)（静止或缓动时）。"""
    exp = rot_t_apply(q_of(v), (0.0, 0.0, G))
    for got, want, k in zip((v["ax"], v["ay"], v["az"]), exp, "xyz"):
        if abs(got - want) > tol:
            raise AssertionError("gravity check failed on a%s: %.4f vs %.4f" % (k, got, want))


def check_pair(group, va, vb):
    kind, dt_us, lost = imu_transition(va["seq"], va["stamp_us"], vb["seq"], vb["stamp_us"])
    if group == "reset_pair":
        # 两帧都是 accept；第二帧判定为 MCU 复位：不计丢帧、不参与求 dt。
        if kind != "reset" or lost != 0 or dt_us is not None:
            raise AssertionError("reset_pair: expected reset, got %s lost=%s dt=%s" % (kind, lost, dt_us))
        if not ((vb["stamp_us"] - va["stamp_us"]) & 0xFFFFFFFF) >= 0x80000000:
            raise AssertionError("reset_pair: stamp must go backwards")
        if not ((vb["seq"] - va["seq"] + 32768) & 0xFFFF) - 32768 <= -SEQ_RESET_BACK:
            raise AssertionError("reset_pair: seq must regress by >= %d" % SEQ_RESET_BACK)
        return
    if kind != "normal" or lost != 0:
        raise AssertionError("%s: transition kind=%s lost=%s" % (group, kind, lost))
    dseq = (vb["seq"] - va["seq"]) & 0xFFFF
    if dseq != 1:
        raise AssertionError("%s: dseq=%d" % (group, dseq))
    if dt_us != 2000:
        raise AssertionError("%s: dt_us=%d" % (group, dt_us))
    dt = dt_us * 1e-6
    if group == "wrap_pair":
        return
    for v in (va, vb):
        check_gravity(v)
    if group == "physical_yaw_pos":
        for v in (va, vb):
            if not v["gz"] > 0:
                raise AssertionError("yaw pair: gz must be > 0")
        d = yaw_of(q_of(vb)) - yaw_of(q_of(va))
        if not d > 0:
            raise AssertionError("yaw pair: yaw(q) must increase")
        theta = pitch_of(q_of(va))
        want = va["gz"] / math.cos(theta) * dt      # psi_dot = gz / cos(theta)
        if abs(d - want) > 0.1 * abs(want):
            raise AssertionError("yaw pair: dyaw=%.6f vs gz*dt/cos=%.6f" % (d, want))
    elif group == "physical_pitch_pos":
        for v in (va, vb):
            if not v["gy"] > 0:
                raise AssertionError("pitch pair: gy must be > 0")
        d = pitch_of(q_of(vb)) - pitch_of(q_of(va))
        if not d > 0:
            raise AssertionError("pitch pair: pitch(q) must increase")
        want = va["gy"] * dt
        if abs(d - want) > 0.1 * abs(want):
            raise AssertionError("pitch pair: dpitch=%.6f vs gy*dt=%.6f" % (d, want))


def verify_records(records):
    """records: list of (name, dir, group, expect, hex_string, decoded_string)。

    对每条记录独立地重新解码并与记录比对；再做物理检查和流式检查。
    """
    by_dir = {"pc2mcu": [], "mcu2pc": []}
    groups = {}
    for name, d, group, expect, hx, decoded, state in records:
        frame = bytes(int(t, 16) for t in hx.split())
        ok, reason, vals = decode_frame(frame)
        if (expect == "accept") != ok:
            raise AssertionError("%s: expect=%s but decode ok=%s (%s)" % (name, expect, ok, reason))
        if fmt_decoded(vals) != decoded:
            raise AssertionError("%s: decoded mismatch\n  %s\n  %s" % (name, fmt_decoded(vals), decoded))
        # state 字段：由关节状态重新生成一帧，每个 raw 与 hex 相差不超过 1 LSB
        st = parse_state(state)
        if st is not None:
            if not ok:
                raise AssertionError("%s: state on a rejected frame" % name)
            _s, _t, a_raw, g_raw, q_raw = imu_from_state(vals["seq"], vals["stamp_us"], *st)
            got = [vals[k] for k in ("ax_raw", "ay_raw", "az_raw", "gx_raw", "gy_raw", "gz_raw",
                                     "qw_raw", "qx_raw", "qy_raw", "qz_raw")]
            for g, w in zip(got, list(a_raw) + list(g_raw) + list(q_raw)):
                if abs(g - w) > 1:
                    raise AssertionError("%s: state does not reproduce hex (%s vs %s)" % (name, got, (a_raw, g_raw, q_raw)))
        elif group.startswith("physical_"):
            raise AssertionError("%s: physical case must carry a state field" % name)
        # 帧内不得出现 “AA + 目标 addr” 的伪帧头，否则流式回扫会有歧义
        for i in range(1, len(frame) - 1):
            if frame[i] == HEAD and frame[i + 1] in (ADDR_PC2MCU, ADDR_MCU2PC):
                raise AssertionError("%s: fake header inside frame at %d" % (name, i))
        by_dir[d].append((name, expect, hx))
        if group not in ("-",) and ok:
            groups.setdefault(group, []).append(vals)
    for g, lst in groups.items():
        if g == "static":
            for v in lst:
                check_gravity(v)
        elif g in ("physical_yaw_pos", "physical_pitch_pos", "wrap_pair", "reset_pair"):
            if len(lst) != 2:
                raise AssertionError("group %s must have 2 frames" % g)
            check_pair(g, lst[0], lst[1])
    # 流式：同方向全部帧拼接（前缀垃圾字节），逐字节喂状态机，输出的是通过应用层解码
    # （0x30 的 isfinite、0x01 的 q 范数）之后的帧，只应是 expect=accept 的帧，且顺序一致
    garbage = bytes([0x55, 0x00, 0xAA, 0x12, 0x99])
    for d, addr in (("pc2mcu", ADDR_PC2MCU), ("mcu2pc", ADDR_MCU2PC)):
        sp = StreamParser(addr)
        sp.feed(garbage)
        for _name, _expect, hx in by_dir[d]:
            sp.feed(bytes(int(t, 16) for t in hx.split()))
        want = [hx for _n, e, hx in by_dir[d] if e == "accept"]
        if sp.accepted != want:
            raise AssertionError("stream %s: accepted %d frames, wanted %d" % (d, len(sp.accepted), len(want)))


# --------------------------------------------------------------------------
# 输出与读回
# --------------------------------------------------------------------------
HEADER = """\
# golden_frames.txt  PROTOCOL.md v1.4  2026-09-30
# 由 protocol/gen_golden.py 确定性生成，请勿手改。
#
# 格式：文本，UTF-8，LF 换行。以 '#' 开头的行是注释；空行忽略。
# 每个用例是一个块，字段一行一个，写作 "key: value"，以单独一行 "end" 结束。
# 字段固定顺序：case dir func expect reason group hex decoded state note
#   case    用例名（[a-z0-9_]+）
#   dir     pc2mcu（addr=0xEE，0x30 指令）或 mcu2pc（addr=0xFF，0x01 IMU）
#   func    帧里的功能号，十六进制
#   expect  accept 或 reject。reject 表示接收方必须丢弃，不产生任何输出
#   reason  reject 的原因说明（bad_checksum bad_len non_finite bad_quat bad_head bad_addr unknown_func），
#           只作说明；测试只比较 accept/reject。accept 时为 '-'
#   group   成组用例的组名，无则 '-'。同一组内的用例按文件顺序相邻出现
#   hex     整帧字节，大写十六进制，空格分隔，含 AA 头和两个校验字节
#   decoded accept 时为空格分隔的 key=value；reject 时为 '-'
#           0x30: yaw_omega pitch_omega                       （rad/s，float32 精确值）
#           0x01: seq stamp_us ax_raw..qz_raw（整数）与 ax ay az（m/s^2）gx gy gz（rad/s）qw qx qy qz
#           浮点按 %.9g 打印。比较容差：绝对 1e-6（整数字段必须完全相等）
#   state   物理用例的关节状态 "psi=.. theta=.. psid=.. thetad=.."（rad、rad/s，%.9g），无则 '-'。
#           虚拟云台按同一物理模型由 state 生成的帧，每个 raw 与 hex 相差不超过 1 LSB。
#           psi=yaw，theta=pitch（低头为正），psid、thetad 是各自的关节角速度。physical_* 组必须带 state
#   note    英文说明
# 组与检查（细节见 PROTOCOL.md 第 9 节）：
#   static              accel 约等于 R(q)^T(0,0,+g)
#   physical_yaw_pos    2 帧：gz>0，q 的 yaw 增大，且 dyaw = gz/cos(pitch)*dt
#   physical_pitch_pos  2 帧：gy>0，q 的 pitch 增大，且 dpitch = gy*dt
#   wrap_pair           2 帧：seq 65535->0，stamp_us 回绕；dseq=1，dt_us=2000（无符号减法）
#   reset_pair          2 帧都是 accept：seq 5000->0，stamp_us 60000000->2000000（倒退）。
#                       接收方判定 MCU 已复位：重设 prev_seq/prev_stamp、重置滤波器，不计丢帧，不求 dt
# 流式测试：同一 dir 的全部块按文件顺序拼接，前面加垃圾字节 55 00 AA 12 99，逐字节喂给解析器，
#   输出（状态机加应用层解码之后的帧）必须恰好是该 dir 中 expect=accept 的帧，顺序相同。
"""


def write_golden(path, cases):
    lines = [HEADER.rstrip("\n"), ""]
    n_acc = n_rej = 0
    for c in cases:
        ok, reason, vals = decode_frame(c.frame)
        if c.expect == "accept":
            n_acc += 1
            if not ok:
                raise AssertionError("%s: expected accept, got %s" % (c.name, reason))
        else:
            n_rej += 1
            if ok:
                raise AssertionError("%s: expected reject but decoded ok" % c.name)
            if reason != c.reason:
                raise AssertionError("%s: reject reason %s != %s" % (c.name, reason, c.reason))
        lines += [
            "case: %s" % c.name,
            "dir: %s" % c.dir,
            "func: 0x%02X" % c.func,
            "expect: %s" % c.expect,
            "reason: %s" % (c.reason if c.expect == "reject" else "-"),
            "group: %s" % c.group,
            "hex: %s" % hexstr(c.frame),
            "decoded: %s" % fmt_decoded(vals),
            "state: %s" % fmt_state(c.state),
            "note: %s" % c.note,
            "end",
            "",
        ]
    text = "\n".join(lines)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    return n_acc, n_rej


def parse_golden(path):
    recs = []
    cur = {}
    with open(path, "r", encoding="utf-8", newline="") as f:
        raw = f.read()
    if "\r" in raw:
        raise AssertionError("golden file must use LF line endings")
    for line in raw.split("\n"):
        if not line or line.startswith("#"):
            continue
        if line == "end":
            recs.append((cur["case"], cur["dir"], cur["group"], cur["expect"], cur["hex"], cur["decoded"],
                         cur["state"]))
            cur = {}
            continue
        k, _, v = line.partition(": ")
        cur[k] = v
    if cur:
        raise AssertionError("unterminated block")
    return recs


def selftest_encoders():
    """对编码/解码做几组数值级往返，防止两边同时写错。"""
    for yaw, pitch in ((0.0, 0.0), (1.5, -0.5), (-3.25, 2.75), (123.5, -0.001)):
        ok, _r, v = decode_frame(cmd_frame(yaw, pitch))
        assert ok
        assert v["yaw_omega"] == struct.unpack("<f", struct.pack("<f", yaw))[0]
        assert v["pitch_omega"] == struct.unpack("<f", struct.pack("<f", pitch))[0]
    for s, t in ((0, 0), (65535, 4294967295), (32768, 2147483648), (1, 1)):
        ok, _r, v = decode_frame(imu_frame(s, t, (-32768, 0, 32767), (1, -1, 0), (32760, -32760, 0, 1)))
        assert ok and v["seq"] == s and v["stamp_us"] == t
        assert (v["ax_raw"], v["ay_raw"], v["az_raw"]) == (-32768, 0, 32767)
        assert (v["gx_raw"], v["gy_raw"], v["gz_raw"]) == (1, -1, 0)
        assert (v["qw_raw"], v["qx_raw"], v["qy_raw"], v["qz_raw"]) == (32760, -32760, 0, 1)
    # 取整：恰好 .5 远离零（lroundf），不是就近取偶
    assert (rnd(0.5), rnd(1.5), rnd(2.5), rnd(-0.5), rnd(-2.5), rnd(0.49), rnd(-0.51)) == (1, 2, 3, -1, -3, 0, -1)
    # IMU 相邻帧判定（第 6.3 节）
    assert imu_transition(65535, 4294966000, 0, 704) == ("normal", 2000, 0)          # 回绕
    assert imu_transition(10, 1000, 14, 9000) == ("normal", 8000, 3)                 # 丢 3 帧
    assert imu_transition(5000, 60000000, 0, 2000000)[0] == "reset"                  # seq 与 stamp 都倒退
    assert imu_transition(500, 1000000, 0, 20000000)[0] == "reset"                   # 只有 seq 倒退
    assert imu_transition(5, 60000000, 6, 2000000)[0] == "reset"                     # 只有 stamp 倒退
    assert imu_transition(7, 1000, 7, 1000) == ("normal", 0, 0)                      # 重复帧：不丢弃，也不计丢帧
    # 全零四元数：校验正确但应用层丢弃
    ok, r, _v = decode_frame(imu_frame(1, 1, (0, 0, 1365), (0, 0, 0), (0, 0, 0, 0)))
    assert (not ok) and r == "bad_quat"
    ok, r, _v = decode_frame(imu_frame(1, 1, (0, 0, 1365), (0, 0, 0), (16379, 0, 0, 0)))    # |q| 略小于 0.5
    assert (not ok) and r == "bad_quat"
    ok, r, _v = decode_frame(imu_frame(1, 1, (0, 0, 1365), (0, 0, 0), (16380, 0, 0, 0)))    # |q| = 0.5 恰好通过
    assert ok
    # 手算校验：AA EE 30 08 + 8 个 0 字节
    f = build_frame(ADDR_PC2MCU, FUNC_CMD, bytes(8))
    s1 = (0xAA + 0xEE + 0x30 + 0x08) & 0xFF
    s2 = 0
    run = 0
    for b in (0xAA, 0xEE, 0x30, 0x08):
        run = (run + b) & 0xFF
        s2 = (s2 + run) & 0xFF
    s2 = (s2 + run * 8) & 0xFF
    assert f[-2] == s1 and f[-1] == s2, (f[-2:], s1, s2)


def main(argv):
    here = os.path.dirname(os.path.abspath(__file__))
    out = os.path.join(here, "golden_frames.txt")
    check_only = False
    i = 1
    while i < len(argv):
        if argv[i] == "--out":
            out = argv[i + 1]
            i += 2
        elif argv[i] == "--check":
            check_only = True
            i += 1
        else:
            print("unknown arg", argv[i], file=sys.stderr)
            return 2
    selftest_encoders()
    if not check_only:
        cases = build_cases()
        names = [c.name for c in cases]
        assert len(names) == len(set(names)), "duplicate case name"
        n_acc, n_rej = write_golden(out, cases)
        print("wrote %s: %d cases (%d accept, %d reject)" % (out, len(cases), n_acc, n_rej))
    recs = parse_golden(out)
    verify_records(recs)
    print("self-check OK: %d records re-decoded, physical and stream checks passed" % len(recs))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
