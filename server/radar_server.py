# -*- coding: utf-8 -*-
"""
============================================================================
 无线毫米波雷达智能感知系统 V1.0 —— PC 服务端（UDP 接收 + Web 可视化）
 Wireless mmWave Radar Smart System - PC Radar Server
============================================================================
 功能概述:
   1. UDP 监听雷达节点上报数据（协议魔数 0xD03DA7A 校验，支持多节点）
   2. 内置 Web 服务，提供雷达可视化界面与 REST API（纯标准库，零依赖）
   3. 目标轨迹环形缓冲（最近 N 帧），支持导出 CSV
   4. 警戒区告警引擎：目标进入警戒区触发告警事件（界面警示 + 日志）
   5. 事件日志与控制台输出，便于二次开发与排障

 快速开始:
   python radar_server.py                  # 默认 UDP 4210, Web 8080
   python radar_server.py --simulate       # 附带内置模拟器（无硬件演示）
   python radar_server.py --port 5000 --web-port 9000

 网络协议（与固件一致，packed, little-endian）:
   uint32 magic | uint32 nodeId | uint32 frameCount | uint32 uptimeMs
   | Target[3] | uint8 flags | uint8 reserved[3]
   Target = int16 x | int16 y | int16 spd | uint8 valid
   总长度 = 41 字节

 参考来源: Arduino-ESP32-Radarproject (MIT License, Sudo_solder)
 本文件在 MIT 许可下深度重构并增强功能。
============================================================================
"""

import argparse
import json
import math
import os
import signal
import socket
import struct
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# ============================ 协议常量 ============================

PROTOCOL_MAGIC = 0x0D03DA7A
UDP_PACKET_SIZE = 41
PACKET_FMT = "<IIIIhhhBhhhBhhhBB3s"
PACKET_UNPACK = struct.Struct(PACKET_FMT).unpack

MAX_TARGETS = 3
MAX_NODES = 8

# 警戒区（毫米，与固件一致）：X ∈ [-4000, 4000]，Y ∈ [200, 4000]
ALARM_ZONE_MIN_X = -4000
ALARM_ZONE_MAX_X = 4000
ALARM_ZONE_MIN_Y = 200
ALARM_ZONE_MAX_Y = 4000

# 目标点颜色（与固件一致）
NODE_COLORS = ["#f5222d", "#faad14", "#13c2c2", "#fa8c16",
               "#52c41a", "#1677ff", "#722ed1", "#eb2f96"]

DEFAULT_UDP_PORT = 4210
DEFAULT_WEB_PORT = 8080
HISTORY_LIMIT = 2000          # 轨迹历史缓冲上限（帧）
LINK_TIMEOUT_SEC = 3.0       # 节点失联判定
ALARM_HISTORY_LIMIT = 200    # 告警事件缓冲

# ============================ 数据模型 ============================


class Target:
    """单个雷达目标（毫米坐标）"""

    __slots__ = ("x", "y", "spd", "valid")

    def __init__(self, x=0, y=0, spd=0, valid=0):
        self.x = x
        self.y = y
        self.spd = spd
        self.valid = bool(valid)

    def distance(self):
        return math.sqrt(self.x * self.x + self.y * self.y)

    def to_dict(self):
        return {"x": self.x, "y": self.y, "spd": self.spd,
                "valid": self.valid, "dist": round(self.distance(), 1)}


class RadarFrame:
    """一帧完整数据（单节点）"""

    __slots__ = ("node_id", "frame_count", "uptime_ms", "targets",
                 "flags", "recv_ts")

    def __init__(self, node_id, frame_count, uptime_ms, targets, flags, recv_ts):
        self.node_id = node_id
        self.frame_count = frame_count
        self.uptime_ms = uptime_ms
        self.targets = targets
        self.flags = flags
        self.recv_ts = recv_ts


class NodeState:
    """节点在线状态"""

    def __init__(self, node_id):
        self.node_id = node_id
        self.online = False
        self.last_packet_ts = 0.0
        self.last_frame = 0
        self.frame_rate = 0.0
        self.uptime_ms = 0

    def touch(self, frame):
        now = frame.recv_ts
        if self.last_packet_ts > 0:
            dt = now - self.last_packet_ts
            if dt > 0:
                self.frame_rate = self.frame_rate * 0.8 + (1.0 / dt) * 0.2
        self.last_packet_ts = now
        self.last_frame = frame.frame_count
        self.uptime_ms = frame.uptime_ms
        self.online = True

    def is_alive(self, now):
        return self.online and (now - self.last_packet_ts) < LINK_TIMEOUT_SEC


# ============================ 数据存储 ============================


class DataStore:
    """集中存储：节点状态、最近帧、目标轨迹、告警事件"""

    def __init__(self):
        self._lock = threading.Lock()
        self.nodes = {}
        self.latest_frames = {}                       # node_id -> Frame
        self.history = deque(maxlen=HISTORY_LIMIT)    # 全部帧（用于轨迹）
        self.alarm_history = deque(maxlen=ALARM_HISTORY_LIMIT)
        self.alarm_active = False
        self.alarm_since = 0.0
        self._node_tracks = {}   # node_id -> [deque[(ts,x,y)] x MAX_TARGETS]

    # ---- 帧写入 ----
    def ingest(self, frame):
        with self._lock:
            node = self.nodes.get(frame.node_id)
            if node is None:
                node = NodeState(frame.node_id)
                self.nodes[frame.node_id] = node
            node.touch(frame)

            self.latest_frames[frame.node_id] = frame
            self.history.append(frame)

            # 记录轨迹点
            track = self._node_tracks.setdefault(
                frame.node_id, [deque(maxlen=300) for _ in range(MAX_TARGETS)])
            for i, tgt in enumerate(frame.targets):
                if tgt.valid:
                    track[i].append((frame.recv_ts, tgt.x, tgt.y))

            # 警戒区判定
            in_zone = any(
                self._in_zone(t.x, t.y) for t in frame.targets if t.valid)
            if in_zone and not self.alarm_active:
                self.alarm_active = True
                self.alarm_since = time.time()
                self.alarm_history.append(
                    {"ts": frame.recv_ts, "node": frame.node_id, "type": "enter"})
            elif not in_zone and self.alarm_active:
                self.alarm_active = False
                self.alarm_history.append(
                    {"ts": frame.recv_ts, "node": frame.node_id, "type": "exit"})

    # ---- 警戒区判定 ----
    @staticmethod
    def _in_zone(x, y):
        return (ALARM_ZONE_MIN_X <= x <= ALARM_ZONE_MAX_X and
                ALARM_ZONE_MIN_Y <= y <= ALARM_ZONE_MAX_Y)

    # ---- 状态快照（供 Web API）----
    def snapshot(self):
        with self._lock:
            now = time.time()
            nodes = []
            for nid in sorted(self.nodes):
                node = self.nodes[nid]
                alive = node.is_alive(now)
                nodes.append({
                    "id": nid,
                    "online": alive,
                    "frame": node.last_frame,
                    "fps": round(node.frame_rate, 1) if alive else 0,
                    "uptime": node.uptime_ms,
                    "color": NODE_COLORS[(nid - 1) % len(NODE_COLORS)],
                })
            targets = []
            for nid in sorted(self.latest_frames):
                frame = self.latest_frames[nid]
                for i, tgt in enumerate(frame.targets):
                    if tgt.valid:
                        targets.append({
                            "node": nid,
                            "idx": i,
                            "x": tgt.x, "y": tgt.y,
                            "spd": tgt.spd,
                            "dist": round(tgt.distance(), 1),
                            "in_zone": self._in_zone(tgt.x, tgt.y),
                            "color": NODE_COLORS[(nid - 1) % len(NODE_COLORS)],
                        })
            tracks = []
            for nid in sorted(self._node_tracks):
                for i, pts in enumerate(self._node_tracks[nid]):
                    if pts:
                        tracks.append({
                            "node": nid, "idx": i,
                            "pts": [[round(p[1], 1), round(p[2], 1)] for p in pts],
                            "color": NODE_COLORS[(nid - 1) % len(NODE_COLORS)],
                        })
            return {
                "now": now,
                "alarm": self.alarm_active,
                "alarm_since": self.alarm_since,
                "nodes": nodes,
                "targets": targets,
                "tracks": tracks,
                "alarm_events": list(self.alarm_history)[-20:],
                "zone": {"min_x": ALARM_ZONE_MIN_X, "max_x": ALARM_ZONE_MAX_X,
                         "min_y": ALARM_ZONE_MIN_Y, "max_y": ALARM_ZONE_MAX_Y},
            }

    def export_csv(self):
        """导出轨迹数据为 CSV 文本"""
        lines = ["ts,node,target,x_mm,y_mm,speed_mm_s,distance_mm"]
        with self._lock:
            for frame in self.history:
                ts = frame.recv_ts
                for i, tgt in enumerate(frame.targets):
                    if tgt.valid:
                        lines.append("%.3f,%d,%d,%d,%d,%d,%.1f" % (
                            ts, frame.node_id, i, tgt.x, tgt.y, tgt.spd,
                            tgt.distance()))
        return "\n".join(lines)


# ============================ UDP 接收 ============================


def parse_packet(data):
    """解析 41 字节 UDP 数据包，返回 RadarFrame 或 None"""
    if len(data) < UDP_PACKET_SIZE:
        return None
    try:
        (magic, node_id, frame_count, uptime_ms,
         x0, y0, s0, v0,
         x1, y1, s1, v1,
         x2, y2, s2, v2,
         flags, _res) = PACKET_UNPACK(data[:UDP_PACKET_SIZE])
    except struct.error:
        return None
    if magic != PROTOCOL_MAGIC:
        return None
    targets = [Target(x0, y0, s0, v0),
               Target(x1, y1, s1, v1),
               Target(x2, y2, s2, v2)]
    return RadarFrame(node_id, frame_count, uptime_ms, targets, flags,
                      time.time())


def udp_listener(sock, store, stop_event):
    """UDP 接收线程"""
    while not stop_event.is_set():
        try:
            data, _addr = sock.recvfrom(2048)
        except OSError:
            if stop_event.is_set():
                break
            continue
        frame = parse_packet(data)
        if frame is not None:
            store.ingest(frame)
            print("[UDP] node=%d frame=%d targets=%d" %
                  (frame.node_id, frame.frame_count,
                   sum(1 for t in frame.targets if t.valid)))


# ============================ 模拟器 ============================


def make_sim_frame(node_id, t):
    """构造模拟数据帧：节点1 圆周运动，节点2 直线往返"""
    targets = []
    for i in range(2):
        if node_id == 1:
            x = 2500 * math.cos(0.8 * t + i * 2.1)
            y = 2500 * math.sin(0.8 * t + i * 2.1) + 3000
        else:
            x = 1000 * math.sin(0.5 * t + i)
            y = 2000 + 1500 * math.sin(0.7 * t + i * 1.7)
        targets.append(Target(int(x), int(y), 800, 1))
    targets.append(Target(0, 0, 0, 0))
    return RadarFrame(node_id, int(t * 20), int(t * 1000), targets, 1,
                      time.time())


def run_simulator(store, stop_event):
    """无硬件演示：模拟 2 个节点目标做圆周/直线运动"""
    t = 0.0
    while not stop_event.is_set():
        for n in range(1, 3):
            store.ingest(make_sim_frame(n, t))
        time.sleep(0.1)
        t += 0.1


# ============================ Web 服务 ============================


class RadarHTTPHandler(BaseHTTPRequestHandler):
    """REST API + 静态界面"""

    store = None  # 由外部注入

    # ---------- 路由 ----
    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/" or path == "/index.html":
            self._serve_index()
        elif path == "/api/status":
            self._send_json(self.store.snapshot())
        elif path == "/api/export.csv":
            self._serve_csv()
        else:
            self.send_error(404)

    # ---------- 页面 ----
    def _serve_index(self):
        html_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 "index.html")
        if os.path.exists(html_path):
            with open(html_path, "rb") as fh:
                body = fh.read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            self._send_json({"error": "index.html not found"})

    # ---------- 工具 ----
    def _send_json(self, obj):
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _serve_csv(self):
        body = self.store.export_csv().encode("utf-8-sig")
        self.send_response(200)
        self.send_header("Content-Type", "text/csv; charset=utf-8")
        self.send_header("Content-Disposition",
                         "attachment; filename=radar_export.csv")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        sys.stderr.write("[WEB] %s\n" % (fmt % args))


# ============================ 主程序 ============================


def parse_args():
    ap = argparse.ArgumentParser(description="无线毫米波雷达智能感知系统 - PC 服务端")
    ap.add_argument("--port", type=int, default=DEFAULT_UDP_PORT,
                    help="UDP 监听端口 (默认 %d)" % DEFAULT_UDP_PORT)
    ap.add_argument("--web-port", type=int, default=DEFAULT_WEB_PORT,
                    help="Web 服务端口 (默认 %d)" % DEFAULT_WEB_PORT)
    ap.add_argument("--simulate", action="store_true",
                    help="启动内置模拟器（无需硬件）")
    return ap.parse_args()


def main():
    args = parse_args()
    store = DataStore()
    stop_event = threading.Event()

    # 1. UDP 服务
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_sock.bind(("0.0.0.0", args.port))
    udp_sock.settimeout(0.5)

    t_udp = threading.Thread(target=udp_listener,
                             args=(udp_sock, store, stop_event), daemon=True)
    t_udp.start()

    # 2. 模拟器（可选）
    if args.simulate:
        t_sim = threading.Thread(target=run_simulator,
                                 args=(store, stop_event), daemon=True)
        t_sim.start()
        print("[SIM] 内置模拟器已启动（演示模式）")

    # 3. Web 服务
    RadarHTTPHandler.store = store
    httpd = ThreadingHTTPServer(("0.0.0.0", args.web_port),
                                RadarHTTPHandler)

    def shutdown(sig, frame):
        print("\n[EXIT] 正在关闭...")
        stop_event.set()
        udp_sock.close()
        httpd.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    print("=" * 60)
    print(" 无线毫米波雷达智能感知系统 V1.0 - PC 服务端")
    print("=" * 60)
    print(" UDP 监听端口 : %d" % args.port)
    print(" Web 界面     : http://127.0.0.1:%d/" % args.web_port)
    print(" 警戒区       : X[%d,%d]mm  Y[%d,%d]mm" %
          (ALARM_ZONE_MIN_X, ALARM_ZONE_MAX_X,
           ALARM_ZONE_MIN_Y, ALARM_ZONE_MAX_Y))
    print(" 按 Ctrl+C 退出")
    print("=" * 60)

    try:
        httpd.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        udp_sock.close()
        httpd.shutdown()


if __name__ == "__main__":
    main()