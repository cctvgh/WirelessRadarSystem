# -*- coding: utf-8 -*-
"""
============================================================================
 无线毫米波雷达智能感知系统 V1.0 —— 雷达数据模拟器
 Wireless mmWave Radar Smart System - Radar Data Simulator
============================================================================
 功能概述:
   模拟 RD-03D 雷达节点向 PC 服务端（radar_server.py）发送 UDP 数据包，
   用于无硬件环境下的联调、演示与界面开发。

 用法:
   python radar_simulator.py                    # 发送到 127.0.0.1:4210
   python radar_simulator.py --host 192.168.1.10 --port 4210
   python radar_simulator.py --nodes 3 --targets 3

 数据协议（与固件完全一致, packed, little-endian, 41 字节）:
   uint32 magic(0xD03DA7A) | uint32 nodeId | uint32 frameCount | uint32 uptimeMs
   | Target[3] | uint8 flags | uint8 reserved[3]
   Target = int16 x | int16 y | int16 spd | uint8 valid

 参考来源: Arduino-ESP32-Radarproject (MIT License, Sudo_solder)
============================================================================
"""

import argparse
import math
import socket
import struct
import sys
import time

# 协议常量（与固件一致）
MAGIC = 0x0D03DA7A
PACKET_FMT = "<IIIIhhhBhhhBhhhBB3s"
PACKET_SIZE = 41
MAX_TARGETS = 3

# 运动模式
MODE_CIRCLE = "circle"    # 圆周运动
MODE_PENDULUM = "pendulum"  # 直线往返


def build_packet(node_id, frame_count, uptime_ms, targets, flags=1):
    """构造 41 字节 UDP 数据包"""
    t = targets  # list of (x, y, spd, valid)
    data = struct.pack(
        PACKET_FMT,
        MAGIC, node_id, frame_count, uptime_ms,
        t[0][0], t[0][1], t[0][2], t[0][3],
        t[1][0], t[1][1], t[1][2], t[1][3],
        t[2][0], t[2][1], t[2][2], t[2][3],
        flags,
        b"\x00\x00\x00"
    )
    return data


def make_targets(node_id, t, n_active):
    """生成目标数据: 按节点与时间参数生成运动轨迹"""
    targets = []
    for i in range(MAX_TARGETS):
        if i < n_active:
            if node_id == 1:
                # 节点1: 两个目标做圆周运动（围绕不同半径）
                r = 2500.0 - i * 400.0
                x = r * math.cos(0.8 * t + i * 2.1)
                y = r * math.sin(0.8 * t + i * 2.1) + 3000.0
            elif node_id == 2:
                # 节点2: 直线往返运动
                x = 1000.0 * math.sin(0.5 * t + i)
                y = 2000.0 + 1500.0 * math.sin(0.7 * t + i * 1.7)
            elif node_id == 3:
                # 节点3: 缓慢移动（模拟低频目标）
                x = 800.0 * math.sin(0.2 * t + i)
                y = 3000.0 + 500.0 * math.sin(0.3 * t + i)
            else:
                x = 500.0 * i
                y = 1000.0 + 1000.0 * math.sin(0.4 * t)
            spd = 800
            targets.append((int(x), int(y), spd, 1))
        else:
            targets.append((0, 0, 0, 0))
    return targets


def main():
    ap = argparse.ArgumentParser(description="无线毫米波雷达数据模拟器")
    ap.add_argument("--host", default="127.0.0.1", help="接收端 IP (默认 127.0.0.1)")
    ap.add_argument("--port", type=int, default=4210, help="UDP 端口 (默认 4210)")
    ap.add_argument("--nodes", type=int, default=2, help="模拟节点数 (默认 2)")
    ap.add_argument("--targets", type=int, default=2, help="每节点活动目标数 (默认 2)")
    ap.add_argument("--fps", type=float, default=10.0, help="发送帧率 (默认 10)")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1)

    interval = 1.0 / max(args.fps, 0.1)
    frame = 0
    start = time.time()

    print("=" * 56)
    print(" 无线毫米波雷达数据模拟器")
    print("=" * 56)
    print(" 目标     : %s:%d" % (args.host, args.port))
    print(" 节点数   : %d   每节点目标: %d" % (args.nodes, args.targets))
    print(" 帧率     : %.1f fps" % args.fps)
    print(" 按 Ctrl+C 停止")
    print("=" * 56)

    try:
        while True:
            t = time.time() - start
            for nid in range(1, args.nodes + 1):
                targets = make_targets(nid, t, args.targets)
                pkt = build_packet(nid, frame + nid, int(t * 1000), targets)
                sock.sendto(pkt, (args.host, args.port))
            frame += 1
            if frame % (args.fps * 5) < 1:
                print("[SIM] 已发送 %d 帧" % frame)
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\n[SIM] 已停止，共发送 %d 帧" % frame)
    finally:
        sock.close()


if __name__ == "__main__":
    main()